#include "fastpointcloudrenderer.h"

#include <cassert>

#include <utility>

#include <cgv/math/ftransform.h>
#include <cgv/render/attribute_array_binding.h>

#include <cgv_gl/gl/gl.h>

#include "pinholecameramodel.h"

FastPointCloudRenderer::FastPointCloudRenderer()
{
    set_name("fast point cloud renderer");
}

FastPointCloudRenderer::~FastPointCloudRenderer()
{
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
        while (this->is_visible()) {
            if (!m_point_source)
                continue;
            m_point_source->compute_queries();
        }
    });

    //m_hole_finder = std::thread([] {});

    const auto aspect = static_cast<float>(ctx.get_width()) / ctx.get_height();

    mat4 const MV = compute_view();
    mat4 const P = compute_projection(aspect);

    std::pair<size_t, size_t> resolution(ctx.get_width(), ctx.get_height());
    PinholeCameraModel view_pcm(MV, P, resolution);

    m_ldi = LayeredDepthImage(view_pcm);

    {// Create the LDI shader
        m_ldi_shader.build_dir(ctx, ".");
        assert(m_ldi_shader.is_linked());
    }

    return m_ldi.is_valid();
}

void FastPointCloudRenderer::resize(unsigned int w, unsigned int h)
{
    std::pair<size_t, size_t> resolution(w, h);

    // TODO resize ldi;
}

void FastPointCloudRenderer::init_frame(cgv::render::context &) {}

void FastPointCloudRenderer::draw(cgv::render::context & ctx) {
    assert(m_ldi_shader.is_linked());
    m_ldi_shader.enable(ctx);

    // Compute aspect ratio
    auto const aspect = static_cast<float>(ctx.get_width()) / ctx.get_height();
    vec3 const center_distance;
    {
        auto _ = compute_view().col(3);
        _ /= _.w();
        vec3 const target_center(_.x(), _.y(), _.z());

        const_cast<vec3 &>(center_distance)
            = m_ldi.get_camera().get_projective_origin() - target_center;
    }

    m_ldi_shader.set_uniform(ctx,
                             "SOURCE_LDI_P",
                             m_ldi.get_camera().get_projection(),
                             true);
    m_ldi_shader.set_uniform(ctx,
                             "INV_TARGET_LDI_P",
                             cgv::math::inv(compute_projection(aspect)),
                             true);
    m_ldi_shader.set_uniform(ctx,
                             "CENTER_DISTANCE",
                             center_distance,
                             true);

    // Draw only if data is present
    if (m_vbo_ldi_data.is_created()) {
        glBindBuffer(GL_ARRAY_BUFFER,
                     *static_cast<GLuint *>(m_vbo_ldi_data.handle));

        auto const point_count = m_ldi.point_count();
        assert(std::numeric_limits<GLsizei>::max() >= point_count);
        glBufferData(GL_ARRAY_BUFFER,
                     point_count * 6,
                     m_ldi.interleave_data().data(),
                     GL_STREAM_DRAW);

        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(point_count));
    }

    m_ldi_shader.disable(ctx);
}

void FastPointCloudRenderer::finish_draw(cgv::render::context &) {
    if (m_current_query) {
        // Insert queried point data
        std::vector<float> positions;
        std::vector<float> colors;

        m_current_query->consume_points(positions,colors);
        m_ldi.add_global_points(positions, colors);
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

mat4 FastPointCloudRenderer::compute_view() const
{
    cgv::render::view *view = find_view_as_node();
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
                                     1.f,
                                     100.f);

    return P;
}

void FastPointCloudRenderer::open_point_data(const std::string &filename)
{
    m_point_source = std::make_shared<PointCloudSource>(filename);
    assert(m_point_source);

    m_current_query = m_point_source->queryPoints();
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
