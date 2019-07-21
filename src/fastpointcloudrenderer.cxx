#include "fastpointcloudrenderer.h"

FastPointCloudRenderer::FastPointCloudRenderer()
{
    set_name("fast point cloud renderer");
}

bool FastPointCloudRenderer::init(cgv::render::context &)
{

}

void FastPointCloudRenderer::resize(unsigned int w, unsigned int h)
{

}

void FastPointCloudRenderer::init_frame(cgv::render::context &)
{

}

void FastPointCloudRenderer::draw(cgv::render::context &)
{

}

void FastPointCloudRenderer::finish_draw(cgv::render::context &)
{

}

void FastPointCloudRenderer::finish_frame(cgv::render::context &)
{

}

void FastPointCloudRenderer::after_finish(cgv::render::context &)
{

}

void FastPointCloudRenderer::clear(cgv::render::context &)
{

}

void FastPointCloudRenderer::create_gui()
{
    add_decorator("Fast Point Cloud Renderer",
                  "heading",
                  "level=2");
    add_gui("file",
            m_filename,
            "file_name",
            "title='Open Point Cloud';filter='"
            "Point Clouds (apc,bpc):*.apc;*.bpc"
            "|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct"
            "|All Files:*.*'"
            );
}

void FastPointCloudRenderer::on_set(void *member_ptr)
{
    assert(member_ptr);

    if (member_ptr == &m_filename)
    {
        open_point_data(m_filename);
    }
}

void FastPointCloudRenderer::open_point_data(const std::string &filename)
{
    m_point_source = std::make_shared<PointCloudSource>(filename);
}

extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
