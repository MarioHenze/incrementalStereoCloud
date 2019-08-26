#include "fastpointcloudrenderer.h"

#include <utility>

#include <cgv/render/attribute_array_binding.h>

#include "pinholecameramodel.h"

FastPointCloudRenderer::FastPointCloudRenderer()
{
    set_name("fast point cloud renderer");
}

bool FastPointCloudRenderer::init(cgv::render::context &ctx)
{
    mat4 const MVP = compute_mvp();

    // TODO: compute mapping matrix
    mat3 mapping_matrix;

    std::pair<size_t, size_t> resolution(ctx.get_width(), ctx.get_height());
    PinholeCameraModel view_pcm(MVP, mapping_matrix, resolution);

    m_ldi = LayeredDepthImage(view_pcm);

    {// Create the LDI shader
        m_ldi_shader.build_dir(ctx, "ldi");
        assert(m_ldi_shader.is_linked());
    }

    {// Collect drawing state into VAO
        assert(m_vao.is_created());

        m_vbo_positions.create(ctx);
        m_vbo_color.create(ctx);
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

    m_ldi_shader.set_uniform(ctx,
                             "LDI_INVERSE_MVP",
                             cgv::math::inv(m_ldi.get_camera().get_mvp()),
                             true);
    m_ldi_shader.set_uniform(ctx,
                             "LDI_INVERSE_MAPPING",
                             cgv::math::inv(m_ldi.get_camera().get_mapping()),
                             true);
    m_ldi_shader.set_uniform(ctx,
                             "LDI_TARGET_VIEWPORT",
                             compute_mvp(),
                             true);


    m_ldi_shader.disable(ctx);
}

void FastPointCloudRenderer::finish_draw(cgv::render::context &) {}

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

cgv::render::render_types::mat4 FastPointCloudRenderer::compute_mvp() const
{
    cgv::render::view *view = find_view_as_node();
    assert(view);

    // Build the PinholeCamera from the parameters of the view
    auto const eye = view->get_eye();
    auto const direction = view->get_view_dir();
    auto const direction_up = view->get_view_up_dir();
    auto const direction_ortho = cgv::math::cross(direction, direction_up);

    mat4 MVP;
    MVP.set_col(0, direction.lift());
    MVP.set_col(1, direction_up.lift());
    MVP.set_col(2, direction_ortho.lift());
    MVP.set_col(3, eye.lift());

    return MVP;
}

void FastPointCloudRenderer::open_point_data(const std::string &filename)
{
    m_point_source = std::make_shared<PointCloudSource>(filename);
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
