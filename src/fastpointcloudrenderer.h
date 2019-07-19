#ifndef FASTPOINTCLOUDRENDERER_H
#define FASTPOINTCLOUDRENDERER_H

#include <cgv/base/node.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/drawable.h>
#include <point_cloud/point_cloud.h>

class FastPointCloudRenderer:
        public cgv::base::node,
        public cgv::gui::provider,
        public cgv::render::drawable
{
protected:
    std::string m_filename;

public:
    FastPointCloudRenderer();

    /// this method is called after creation or recreation of the context,
    /// return whether all necessary functionality is supported
    virtual bool init(cgv::render::context&) override;
    /// callback to anounce resizing of the output window
    virtual void resize(unsigned int w, unsigned int h) override;
    /// this method is called in one pass over all drawables before the draw
    /// method
    virtual void init_frame(cgv::render::context&) override;
    /// overload to draw the content of this drawable
    virtual void draw(cgv::render::context&) override;
    /// this method is called when the current drawable is left in a tree
    /// traversal that calls the draw method
    virtual void finish_draw(cgv::render::context&) override;
    /// this method is called in one pass over all drawables after drawing
    virtual void finish_frame(cgv::render::context&) override;
    /// this method is called in one pass over all drawables after finish frame
    virtual void after_finish(cgv::render::context&) override;
    /// clear all objects living in the context like textures or display lists
    virtual void clear(cgv::render::context&) override;

    virtual void create_gui() override;

private:

};

#endif // FASTPOINTCLOUDRENDERER_H
