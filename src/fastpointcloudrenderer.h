#ifndef FASTPOINTCLOUDRENDERER_H
#define FASTPOINTCLOUDRENDERER_H

#include <atomic>
#include <memory>
#include <thread>

#include <cgv/base/node.h>
#include <cgv/gui/provider.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/render/context.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/vertex_buffer.h>

#include <cgv_gl/renderer.h>
#include <cgv_gl/gl/gl.h>

#include "pointcloudsource.h"
#include "layereddepthimage.h"

using cgv::render::render_types;

class FastPointCloudRenderer:
        public cgv::base::node,
        public cgv::gui::provider,
        public cgv::render::drawable
{
protected:
    std::string m_filename;

public:
    FastPointCloudRenderer();
    /// There are threads as members for which copy construction won't work
    FastPointCloudRenderer(FastPointCloudRenderer const &other) = delete;
    /// All members are still trivially movable, therefore force default
    FastPointCloudRenderer(FastPointCloudRenderer &&other) = default;
    ~FastPointCloudRenderer() override;

    /// Member threads are not copyable
    FastPointCloudRenderer &operator=(const FastPointCloudRenderer &other)
        = delete;
    /// Members are trivially moveable
    FastPointCloudRenderer &operator=(FastPointCloudRenderer &&other) = default;

    /// this method is called after creation or recreation of the context,
    /// return whether all necessary functionality is supported
    bool init(cgv::render::context&) override;
    /// callback to anounce resizing of the output window
    void resize(unsigned int w, unsigned int h) override;
    /// this method is called in one pass over all drawables before the draw
    /// method
    void init_frame(cgv::render::context&) override;
    /// overload to draw the content of this drawable
    void draw(cgv::render::context&ctx) override;
    /// this method is called when the current drawable is left in a tree
    /// traversal that calls the draw method
    void finish_draw(cgv::render::context&) override;
    /// this method is called in one pass over all drawables after drawing
    void finish_frame(cgv::render::context&) override;
    /// this method is called in one pass over all drawables after finish frame
    void after_finish(cgv::render::context&) override;
    /// clear all objects living in the context like textures or display lists
    void clear(cgv::render::context&) override;

    void create_gui() override;

    void on_set(void* member_ptr) override;

    bool self_reflect(cgv::reflect::reflection_handler& srh) override;

private:
    /**
     * @brief compute_model_view computes a model view transformation for the
     * current view
     * @return a model view transformation
     */
    [[nodiscard]] mat4 compute_view() const;

    /**
     * @brief compute_projection retrieves the projection matrix of the current
     * view
     * @return a projection matrix
     */
    [[nodiscard]] mat4 compute_projection(const float aspect) const;

    //! The LDI with possibly a representative subset of all points
    LayeredDepthImage m_ldi;

    //! The Point source, which will be used to fill areas in the LDI with
    //! insufficient density of points
    std::shared_ptr<PointCloudSource> m_point_source;

    //! The shader program to render the LDI
    cgv::render::shader_program m_ldi_shader{true};

    //! The VBO containing all point positions and colors of the LDI
    cgv::render::vertex_buffer m_vbo_ldi_data;

    //! Represent the current query on the point cloud source
    std::shared_ptr<PointCloudQuery> m_current_query;

    //! Gives an abort condition for worker threads
    std::atomic<bool> m_destructor_called{false};

    /**
     * @brief m_query_worker resolves all pending queries
     */
    std::thread m_query_worker;

    /**
     * @brief m_hole_finder generates queries for regions with insufficient
     * point density in the LDI
     */
    std::thread m_hole_finder;

    void open_point_data(std::string const & filename);
};

#endif // FASTPOINTCLOUDRENDERER_H
