#include "fastpointcloudrenderer.h"

#include <cassert>
#include <optional>

#include <utility>

#include <cgv/math/ftransform.h>
#include <cgv/math/transformations.h>
#include <cgv/render/attribute_array_binding.h>

#include <cgv_gl/gl/gl.h>

#include "conversion.h"
#include "pinholecameramodel.h"

FastPointCloudRenderer::FastPointCloudRenderer()
{
    set_name("fast_point_cloud_renderer");
}

FastPointCloudRenderer::~FastPointCloudRenderer()
{
    m_destructor_called = true;

    if (m_query_worker.joinable())
        m_query_worker.join();
    if (m_hole_finder.joinable())
        m_hole_finder.join();
}

bool FastPointCloudRenderer::init(cgv::render::context &ctx)
{
    // Define what the query worker and hole finder threads will do
    m_query_worker = std::thread([this] {
        // TODO know when to stop
        while (!m_destructor_called) {
            if (!m_point_source)
                continue;
            m_point_source->compute_queries();
            m_point_source->remove_consumed_queries();
        }
    });

    //m_hole_finder = std::thread([] {});

	
    mat4 const model_view_mat = ctx.get_modelview_matrix();

	// The window matrix of the cgv viewer is exactly the LDI image plane
	// transformation
	auto const win_mat = ctx.get_window_matrix();

    const std::pair<size_t, size_t> resolution(ctx.get_width(), ctx.get_height());
    const PinholeCameraModel view_pcm(model_view_mat, win_mat, resolution);

    m_ldi = LayeredDepthImage(view_pcm);

    {// Create the LDI shader
        m_ldi_shader.build_dir(ctx, "./src");
        assert(m_ldi_shader.is_linked());
    }

	// For the raw handle m_ldi_vbo a new buffer needs to be generated
	glGenBuffers(1, &m_ldi_vbo);

    return m_ldi.is_valid();
}

void FastPointCloudRenderer::resize(unsigned int w, unsigned int h)
{
    std::pair<size_t, size_t> resolution(w, h);

    // TODO resize ldi;
}

void FastPointCloudRenderer::init_frame(cgv::render::context &) {
	const auto ldi_data = m_ldi.interleave_data();

	// Ensure that ldi data is present and a valid buffer handle was generated
	if (!ldi_data.empty() && m_ldi_vbo != 0)
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ldi_vbo);
		glBufferData(
			GL_ELEMENT_ARRAY_BUFFER,
			ldi_data.size() * sizeof(LayeredDepthImage::buffer_type::value_type),
			ldi_data.data(),
			GL_STREAM_DRAW);
	}
}

void FastPointCloudRenderer::draw(cgv::render::context & ctx) {
    assert(m_ldi_shader.is_linked());
    m_ldi_shader.enable(ctx);

	// Determine the translational distance between both the current cgv viewer
	// eye position and the projective center of the LDI
    vec3 const center_distance;
    {
		auto view = find_view_as_node();
		assert(view);

        const_cast<vec3 &>(center_distance) =
			m_ldi.get_camera().get_projective_origin() - view->get_eye();
    }

    {
        bool uniform_assignment_successful{true};

        uniform_assignment_successful
            = uniform_assignment_successful
              && m_ldi_shader.set_uniform(ctx,
                                          "SOURCE_LDI_P",
                                          m_ldi.get_camera().get_projection(),
                                          true);
		assert(uniform_assignment_successful);

		const mat4 inv_target_ldi_p = cgv::math::inv(ctx.get_window_matrix());

        uniform_assignment_successful
            = uniform_assignment_successful
              && m_ldi_shader.set_uniform(ctx,
                                          "INV_TARGET_LDI_P",
                                          inv_target_ldi_p,
                                          true);
		assert(uniform_assignment_successful);
        uniform_assignment_successful = uniform_assignment_successful
                                        && m_ldi_shader
                                               .set_uniform(ctx,
                                                            "CENTER_DISTANCE",
                                                            center_distance,
                                                            true);
        assert(uniform_assignment_successful);
		if (!uniform_assignment_successful)
			std::terminate();
    }

	if (0 != m_ldi_vbo)
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ldi_vbo);
		glDrawArrays(GL_POINTS, 0, m_ldi.point_count());
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

    /*// Draw only if data is present
    if (m_vbo_ldi_data.is_created()) {
        GLuint vbo_handle{0};
        m_vbo_ldi_data.put_id(vbo_handle);
        glBindBuffer(GL_ARRAY_BUFFER,
                     vbo_handle);

        glDrawArrays(GL_POINTS, 0, m_ldi.point_count());
    }*/

    m_ldi_shader.disable(ctx);
}

void FastPointCloudRenderer::finish_draw(cgv::render::context &ctx)
{
    // Try to incorporate newly queried points into the LDI
    auto const opt_query = m_point_source
                               ? m_point_source->get_finished_query()
                               : decltype(
                                   m_point_source->get_finished_query())();

    if (opt_query.has_value()) {
        const auto &finished_query = opt_query.value();
        assert(finished_query);

        std::vector<vec3> positions;
        std::vector<rgb> colors;
        finished_query->consume_points(positions, colors);

        assert(!positions.empty());
        assert(!colors.empty());


		// The points of the query are in world coordinates. We need to
		// transform them into LDI window coordinates.
		std::transform(positions.cbegin(),
			positions.cend(),
			positions.begin(),
			[this, &ctx](decltype(positions)::value_type pos)
			-> decltype(positions)::value_type{
				const mat4 transformation {ctx.get_window_matrix()};
				auto p{transformation * pos.lift()};
				p /= p.w();
				return vec3(p.x(), p.y(), p.z());
			});

        m_ldi.add_transformed_points(positions, colors);

        auto const interleaved_buffer = m_ldi.interleave_data();

        if (interleaved_buffer.empty())
            return;

        if (m_vbo_ldi_data.is_created()) {
            auto const success = m_vbo_ldi_data
                                     .replace(ctx,
                                              0,
                                              interleaved_buffer.data(),
                                              interleaved_buffer.size());
            assert(success);
        } else {
            auto const success = m_vbo_ldi_data
                                     .create(ctx,
                                             interleaved_buffer.data(),
                                             interleaved_buffer.size());
            assert(success);
        }
    }

    // TODO scan over point density image and determine new query

    // TODO if a predicate is met, abort current query and generate new one
}

void FastPointCloudRenderer::finish_frame(cgv::render::context &) {}

void FastPointCloudRenderer::after_finish(cgv::render::context &) {}

void FastPointCloudRenderer::clear(cgv::render::context &) {}

void FastPointCloudRenderer::create_gui()
{
    add_decorator("Fast Point Cloud Renderer", "heading", "level=2");
    add_gui("file",
            m_filename,
            "file_name",
            "title='Open Point Cloud';filter='"
            "Point Clouds (apc,bpc):*.apc;*.bpc"
            "|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct"
            "|All Files:*.*'");
}

void FastPointCloudRenderer::on_set(void *member_ptr)
{
    assert(member_ptr);

    if (member_ptr == &m_filename) {
        open_point_data(m_filename);
    }
}

bool FastPointCloudRenderer::self_reflect(cgv::reflect::reflection_handler &srh)
{
    return srh.reflect_member("file_name", m_filename);
}

mat4 FastPointCloudRenderer::compute_view() const
{
    const cgv::render::view *view = find_view_as_node();
    assert(view);

    // Build the PinholeCamera from the parameters of the view
    // The translation of the camera
    auto const eye = view->get_eye();
    // The direction from camera to -z
    auto const direction = view->get_view_dir();
    auto const direction_up = view->get_view_up_dir();
    // Direction to right
    auto const direction_ortho = cgv::math::cross(direction, direction_up);

    mat4 V;
    V.set_col(0, direction.lift());
    V.set_col(1, direction_up.lift());
    V.set_col(2, direction_ortho.lift());
    V.set_col(3, eye.lift());

    // coordinate axis are directions -> w = 0
    // eye point is a point -> w = 1
    V.set_row(3, {0, 0, 0, 1});

    return V;
}

render_types::mat4 FastPointCloudRenderer::compute_projection(
    float const aspect) const
{
    cgv::render::view *view = find_view_as_node();
    assert(view);

    mat4 P = cgv::math::perspective4(static_cast<float>(
                                         view->get_y_view_angle()),
                                     aspect,
                                     1.F,
                                     100.F);

    return P;
}

void FastPointCloudRenderer::open_point_data(const std::string &filename)
{
#ifndef NDEBUG
    std::cout << __func__ << ": " << filename << std::endl;
#endif

    //m_point_source = std::make_shared<PointCloudSource>(filename);
    point_cloud pc;
    pc.add_point({1,1,1});
    m_point_source = std::make_shared<PointCloudSource>(pc);
    assert(m_point_source);

    // Initially grab all points of the cloud
    m_point_source->queryPoints();
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
