#include "layereddepthimage.h"

#include <pmmintrin.h>

#include <boost/align/is_aligned.hpp>

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

void LayeredDepthImage::add_global_points(const std::vector<float> &points,
                                          const std::vector<float> &colors)
{
    // the supplied points should be 4 component vectors with w = 1
    assert(!(points.size() % 4));
    // the supplied colors should be rgb vectors
    assert(!(colors.size() % 3));

    /*// the vector should be sse aligned
    boost::alignment::is_aligned(points.data(), 16);

    auto const mat = m_camera.projection_center;
    __m128 first_row =  _mm_set_ps(mat(0,0), mat(0,1), mat(0,2), mat(0,3));
    __m128 second_row = _mm_set_ps(mat(1,0), mat(1,1), mat(1,2), mat(1,3));
    __m128 third_row =  _mm_set_ps(mat(2,0), mat(2,1), mat(2,2), mat(2,3));
    __m128 fourth_row = _mm_set_ps(mat(3,0), mat(3,1), mat(3,2), mat(3,3));

    // transform the global points into the LDIs projective viewspace
    for (size_t i = 0; i < points.size() / 4; i++) {
        __m128 point = _mm_load_ps(points.data() + i * 4);

        __m128 a = _mm_mul_ps(first_row, point);
        __m128 b = _mm_mul_ps(second_row, point);
        __m128 c = _mm_mul_ps(third_row, point);
        __m128 d = _mm_mul_ps(fourth_row, point);

        __m128 ab = _mm_hadd_ps(a, b);
        __m128 cd = _mm_hadd_ps(c, d);

        __m128 xyzw = _mm_hadd_ps(ab, cd);
    }*/

    for (size_t i = 0; i < points.size(); i++) {
        // the given points are in global space and need to be transformed into
        // the clip space of the LDI camera
        vec4 point = m_camera.projection_center *
                vec4(4,points.data() + i * 4);
        // TODO clip points outside of view frustum
        vec3 p(3, point / point.w());

        // insert the corresponding color at the calculated z value
        auto & ray = m_layered_points.at(to_index(p.x(), p.y()));
        // ensure depth monotonicity by sorted insertion
        auto const first_greater_depth = std::find_if(
                    ray.cbegin(),
                    ray.cend(),
                    [p](point_t const point){
            return point.depth > p.z();
        });

        point_t new_point;
        new_point.color = rgb(colors.at(i*3),
                              colors.at(i*3+1),
                              colors.at(i*3+2));
        new_point.depth = p.z();
        new_point.splat_index = 1;

        ray.insert(first_greater_depth, new_point);
    }
}

size_t LayeredDepthImage::to_index(size_t x, size_t y) const
{
    const size_t index = y * m_camera.resolution.first + x;
    assert(m_camera.resolution.first * m_camera.resolution.second >= index);
    return index;
}
