#include "fastpointcloudrenderer.h"

#include <utility>

#include <cgv/render/attribute_array_binding.h>

#include "pinholecameramodel.h"

FastPointCloudRenderer::FastPointCloudRenderer() {
  set_name("fast point cloud renderer");
}

bool FastPointCloudRenderer::init(cgv::render::context &ctx) {
  cgv::render::view *view = find_view_as_node();
  assert(view);

  // Build the PinholeCamera from the parameters of the view
  auto const eye = view->get_eye();
  auto const direction = view->get_view_dir();
  auto const direction_up = view->get_view_up_dir();
  auto const direction_ortho = cgv::math::cross(direction, direction_up);

  mat4 projection_center;
  projection_center.set_col(0, direction.lift());
  projection_center.set_col(1, direction_up.lift());
  projection_center.set_col(2, direction_ortho.lift());
  projection_center.set_col(3, eye.lift());

  // TODO: compute mapping matrix
  mat3 mapping_matrix;

  std::pair<size_t, size_t> resolution(ctx.get_width(), ctx.get_height());
  PinholeCameraModel view_pcm(projection_center, mapping_matrix, resolution);

  m_ldi = LayeredDepthImage(view_pcm);

  return m_ldi.is_valid();
}

void FastPointCloudRenderer::resize(unsigned int w, unsigned int h) {
    std::pair<size_t, size_t> resolution(w, h);

}

void FastPointCloudRenderer::init_frame(cgv::render::context &) {}

void FastPointCloudRenderer::draw(cgv::render::context &) {}

void FastPointCloudRenderer::finish_draw(cgv::render::context &) {}

void FastPointCloudRenderer::finish_frame(cgv::render::context &) {}

void FastPointCloudRenderer::after_finish(cgv::render::context &) {}

void FastPointCloudRenderer::clear(cgv::render::context &) {}

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

void FastPointCloudRenderer::open_point_data(const std::string &filename) {
  m_point_source = std::make_shared<PointCloudSource>(filename);
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
