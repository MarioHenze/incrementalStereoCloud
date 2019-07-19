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
        rgb color;
        float depth;
        size_t splat_index;
    };

    /**
     * On every ray, there are a some colored points with different depths
     */
    using ray_t = std::vector<point_t>;

    /**
     * @brief m_layered_points stores all rays going from the center of
     * projection through every pixel
     */
    std::vector<ray_t> m_layered_points;

    /**
     * @brief m_camera holds configuration of projection details of the LDI
     */
    PinholeCameraModel m_camera;
};

#endif // LAYEREDDEPTHIMAGE_H
