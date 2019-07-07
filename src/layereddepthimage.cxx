#include "layereddepthimage.h"

#include <cgv/math/inv.h>

LayeredDepthImage::LayeredDepthImage(PinholeCameraModel pcm):
    m_layered_points(pcm.resolution.first * pcm.resolution.second),
    m_camera(pcm)
{

}

void LayeredDepthImage::warp_reference_into(PinholeCameraModel pcm,
                                            std::vector<rgb> color,
                                            std::vector<float> depth)
{
    {
        size_t pixel_count = pcm.resolution.first * pcm.resolution.second;
        assert(pcm.is_valid());
        assert(color.size() == pixel_count);
        assert(depth.size() == pixel_count);
    }

    const mat4 transfer_matrix =
            pcm.projection_center *
            cgv::math::inv(m_camera.projection_center);

}
