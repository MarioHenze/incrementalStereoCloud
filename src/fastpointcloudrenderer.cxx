#include "fastpointcloudrenderer.h"

#include <cassert>
#include <memory>
#include <optional>

#include <utility>

#include <cgv/gui/mouse_event.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/transformations.h>
#include <cgv/render/attribute_array_binding.h>

#include <cgv_gl/gl/gl.h>

#include "conversion.h"
#include "pinholecameramodel.h"

FastPointCloudRenderer::FastPointCloudRenderer() {
  set_name("fast_point_cloud_renderer");
}

FastPointCloudRenderer::~FastPointCloudRenderer() {
  m_destructor_called = true;

  if (m_query_worker.joinable()) m_query_worker.join();
  if (m_hole_finder.joinable()) m_hole_finder.join();
}

bool FastPointCloudRenderer::init(cgv::render::context &ctx) {
  // To use the point renderer, increase the reference count
  cgv::render::ref_point_renderer(ctx, 1);

  // The point manager coordinates the data upload to the gpu
  if (!m_p_manager.init(ctx)) return false;

  // Define what the query worker and hole finder threads will do
  m_query_worker = std::thread([this] {
    while (!m_destructor_called) {
      if (!m_point_source) continue;
      m_point_source->compute_queries();
      m_point_source->remove_consumed_queries();
    }
  });

  /*m_hole_finder = std::thread([this] {
    while (!m_destructor_called) {
      if (!m_ldi || !m_point_source) continue;
      // Finding holes in the LDI requires exclusive access to the LDI
      // Therefore use a copy of the current LDI
      std::shared_ptr<LayeredDepthImage> copied_ldi;
      {
        std::scoped_lock lock{m_ldi_mutex};
        copied_ldi = std::make_shared<LayeredDepthImage>(*m_ldi.get());
      }

      auto const density_map = copied_ldi->get_density();
    }
  });*/

  mat4 const model_view_mat = ctx.get_modelview_matrix();

  const std::pair<size_t, size_t> resolution(ctx.get_width(), ctx.get_height());
  // The window matrix of the cgv viewer is exactly the LDI image plane
  // transformation
  auto const proj_mat = compute_projection(
      static_cast<float>(resolution.first) / resolution.second);
  const PinholeCameraModel view_pcm(model_view_mat, proj_mat, resolution);

  std::scoped_lock lock{m_ldi_mutex};
  m_ldi = std::make_shared<LayeredDepthImage>(view_pcm);

  // Set up a continous trigger to feed fetched points into the LDI at a steady
  // rate
  cgv::signal::connect(cgv::gui::get_animation_trigger().shoot, this,
                       &FastPointCloudRenderer::point_incorporate_slot);

  return m_ldi->is_valid();
}

void FastPointCloudRenderer::resize(unsigned int w, unsigned int h) {
  // Resize the LDI of this renderer to accomodate the new size
  std::pair<size_t, size_t> resolution(w, h);

  auto const *const ctx = get_context();
  auto const model_view_mat = ctx->get_modelview_matrix();
  auto const proj_mat = compute_projection(static_cast<float>(w) / h);

  PinholeCameraModel const pcm(model_view_mat, proj_mat, resolution);
  auto new_ldi = std::make_shared<LayeredDepthImage>(pcm);
  // Emit a point query for the changed viewport to get new points on the
  // edges of the frame
  m_point_source->queryPoints(pcm);

  std::scoped_lock lock{m_ldi_mutex};
  new_ldi->warp_reference_into(*m_ldi.get());
  m_ldi = new_ldi;
}

void FastPointCloudRenderer::init_frame(cgv::render::context &ctx) {}

void FastPointCloudRenderer::draw(cgv::render::context &ctx) {
  auto const view_ptr = find_view_as_node();
  assert(view_ptr);
  auto &p_renderer = cgv::render::ref_point_renderer(ctx);

  p_renderer.set_reference_point_size(0.005f);
  p_renderer.set_y_view_angle(float(view_ptr->get_y_view_angle()));
  p_renderer.set_attribute_array_manager(ctx, &m_p_manager);
  // TODO ? For now no group rendering is needed ...

  p_renderer.validate_and_enable(ctx);
  // TODO separate drawing method?
  // TODO check if sorted points are needed?
  glDrawArrays(GL_POINTS, 0, m_uploaded_point_count);
  p_renderer.disable(ctx);

  //static int i = 0;
  //ctx.write_frame_buffer_to_image(std::to_string(++i) + ".png");
}

void FastPointCloudRenderer::finish_draw(cgv::render::context &ctx) {
  auto const new_points_copied = incorporate_queries();
  if (new_points_copied) {
    upload_data(ctx);
  }

  // TODO scan over point density image and determine new query

  // TODO if a predicate is met, abort current query and generate new one
}

void FastPointCloudRenderer::finish_frame(cgv::render::context &) {}

void FastPointCloudRenderer::after_finish(cgv::render::context &) {}

void FastPointCloudRenderer::clear(cgv::render::context &ctx) {
  // After destruction, our reference of the point renderer will not be used
  cgv::render::ref_point_renderer(ctx, -1);
}

void FastPointCloudRenderer::create_gui() {
  add_decorator("Fast Point Cloud Renderer", "heading", "level=2");
  add_gui("file", m_filename, "file_name",
          "title='Open Point Cloud';filter='"
          "Point Clouds (apc,bpc):*.apc;*.bpc"
          "|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct"
          "|All Files:*.*'");
}

void FastPointCloudRenderer::on_set(void *member_ptr) {
  assert(member_ptr);

  if (member_ptr == &m_filename) {
    open_point_data(m_filename);
  }
}

bool FastPointCloudRenderer::self_reflect(
    cgv::reflect::reflection_handler &srh) {
  return srh.reflect_member("file_name", m_filename);
}

void FastPointCloudRenderer::update_ldi_slot() {
  auto const ctx = get_context();

  auto const width = ctx->get_width();
  auto const height = ctx->get_height();

  auto const view_transform = ctx->get_modelview_matrix();
  auto const projection =
      compute_projection(static_cast<float>(width) / height);
  PinholeCameraModel new_pcm(view_transform, projection,
                             std::make_pair(width, height));
  m_point_source->queryPoints(new_pcm);

  // The underlying LDI of the view needs to be update aswell
  auto new_ldi = std::make_shared<LayeredDepthImage>(new_pcm);
  std::scoped_lock lock{m_ldi_mutex};
  new_ldi->warp_reference_into(*m_ldi);
  m_ldi.swap(new_ldi);

  post_redraw();
}

bool FastPointCloudRenderer::handle(cgv::gui::event &e) {
  // At the end of a mouse dragging the change of frustum has happend.
  // Use this state flag to determine the end of a dragging process
  static bool still_dragging{false};

  if (e.get_kind() == cgv::gui::EventId::EID_MOUSE) {
    auto const mev = static_cast<cgv::gui::mouse_event &>(e);
    switch (mev.get_action()) {
      case cgv::gui::MA_PRESS: {
        post_redraw();
        return false;
        break;
      }
      case cgv::gui::MA_RELEASE: {
        if (still_dragging) {
          // The mouse was dragged and is now released
          still_dragging = false;

          // A mouse dragging event changes the frustum. Therefore an point
          // update is needed
          update_ldi_slot();
        }
        
        return false;
        break;
      }
      case cgv::gui::MA_WHEEL: {
        update_ldi_slot();
        return false;
      }
      case cgv::gui::MA_DRAG: {
        // Begin the dragging state
        still_dragging = true;
        return false;
        break;
      }
      default:
        break;
    }
  }
  return false;
}

void FastPointCloudRenderer::stream_help(std::ostream &os) {}

void FastPointCloudRenderer::point_incorporate_slot(double t, double dt) {
  incorporate_queries();
  auto * const ctx_ptr = get_context();
  upload_data(*ctx_ptr);
  post_redraw();
}

render_types::mat4 FastPointCloudRenderer::compute_projection(
    float const aspect) const {
  cgv::render::view *view = find_view_as_node();
  assert(view);

  mat4 P = cgv::math::perspective4(static_cast<float>(view->get_y_view_angle()),
                                   aspect, 0.01F, 100.F);

  return P;
}

void FastPointCloudRenderer::upload_data(cgv::render::context &ctx) {
  std::vector<vec3> positional_data;
  std::vector<rgb> color_data;
  {
    std::scoped_lock lock{m_ldi_mutex};

    positional_data = m_ldi->position_data();
    color_data = m_ldi->color_data();
  }

  // If no data is in the LDI, a VBO update is not necessary
  if (positional_data.empty() || color_data.empty()) {
#ifdef DEBUG
    std::err << "LDI was empty?!";
#endif  // DEBUG
    return;
  }

  auto &p_renderer = cgv::render::ref_point_renderer(ctx);
  p_renderer.set_position_array(ctx, positional_data);
  p_renderer.set_color_array(ctx, color_data);

  m_uploaded_point_count = positional_data.size();
}

void FastPointCloudRenderer::open_point_data(const std::string &filename) {
#ifndef NDEBUG
  std::cout << __func__ << ": " << filename << std::endl;
#endif

  m_point_source = std::make_shared<PointCloudSource>(filename);

  assert(m_point_source);
}

bool FastPointCloudRenderer::incorporate_queries() {
  auto const opt_query =
      m_point_source ? m_point_source->get_finished_query() : std::nullopt;

  if (opt_query.has_value()) {
    const auto &finished_query = opt_query.value();
    assert(finished_query);

    std::vector<vec3> positions;
    std::vector<rgb> colors;
    finished_query->consume_points(positions, colors);

    assert(!positions.empty());
    assert(!colors.empty());
    assert(positions.size() == colors.size());

    const mat4 transformation;
    {
      std::scoped_lock lock{m_ldi_mutex};
      auto const camera = m_ldi->get_camera();

      const_cast<mat4 &>(transformation) =
          camera.get_sensor() * camera.get_projection() * camera.get_view();
    }

    // The points of the query are in world coordinates. We need to
    // transform them into LDI window coordinates.
    std::transform(positions.cbegin(), positions.cend(), positions.begin(),
                   [&transformation](decltype(positions)::value_type pos)
                       -> decltype(positions)::value_type {
                     auto p{transformation * pos.lift()};
                     p /= p.w();
                     return vec3(p.x(), p.y(), p.z());
                   });

    {
      std::scoped_lock lock{m_ldi_mutex};
      m_ldi->add_transformed_points(positions, colors);
    }
	// Because new points were copied
    return true;
  } else {
    // There were no new points
    return false;
  }
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
