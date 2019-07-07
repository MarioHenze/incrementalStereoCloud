#ifndef LAYEREDDEPTHIMAGE_H
#define LAYEREDDEPTHIMAGE_H

#include <tuple>
#include <utility>
#include <vector>

#include <cgv/render/render_types.h>

#include "pinholecameramodel.h"

using mat4 = cgv::render::render_types::mat4;
using rgb = cgv::render::render_types::rgb;

class LayeredDepthImage
{
public:
    LayeredDepthImage(PinholeCameraModel pcm);

    void warp_reference_into(PinholeCameraModel pcm,
                             std::vector<rgb> color,
                             std::vector<float> depth);

private:
    /**
     * Borrowing from the LDI concept define depth pixel as a point with color
     * and depth.
     */
    struct point_t
    {
        cgv::render::render_types::rgb color;
        float Z;
        size_t splat_index;
    };

    using layer_t = std::vector<point_t>;

    std::vector<layer_t> m_layered_points;
    PinholeCameraModel m_camera;
};

#endif // LAYEREDDEPTHIMAGE_H
