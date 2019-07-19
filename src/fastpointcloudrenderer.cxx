#include "fastpointcloudrenderer.h"

FastPointCloudRenderer::FastPointCloudRenderer()
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

//extern cgv::base::object_registration<FastPointCloudRenderer> fpcr("");
