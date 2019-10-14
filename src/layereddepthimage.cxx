#include "layereddepthimage.h"

#include <algorithm>
#include <limits>

#include <pmmintrin.h>

#include <boost/align/is_aligned.hpp>

#include <cgv/math/inv.h>

LayeredDepthImage::LayeredDepthImage(const PinholeCameraModel &pcm)
    : m_layered_points(pcm.get_resolution().first *
                       pcm.get_resolution().second),
      m_camera(pcm)
{

}

void LayeredDepthImage::warp_reference_into(PinholeCameraModel const &pcm,
                                            const std::vector<rgb> &color,
                                            const std::vector<float>& depth)
{
    {
        size_t pixel_count = pcm.get_resolution().first
                             * pcm.get_resolution().second;
        assert(pcm.is_valid());
        assert(color.size() == pixel_count);
        assert(depth.size() == pixel_count);
    }


    // TODO McMillan morph
}

void LayeredDepthImage::add_global_points(const std::vector<float> &points,
                                          const std::vector<float> &colors)
{
    assert(colors.size() == points.size());
    // the supplied points should be 3 component vectors
    assert(!(points.size() % 3));
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

    // Boolean predicate if a vector component lies in the unit cube
    auto const is_unit = [](float const &f) {
        return (-1 <= f) && (1 >= f);
    };

    for (size_t i = 0; i < (points.size() / 3); i++) {
        // the given points are in global space and need to be transformed into
        // the clip space of the LDI camera
        vec4 point = m_camera.get_vp()
                     * vec3(points.at(3 * i),
                            points.at(3 * i + 1),
                            points.at(3 * i + 2))
                           .lift();

        vec3 const p(3, point / point.w());
        // Clip points outside of view frustum
        if (!is_unit(p.x()) || !is_unit(p.y()) || !is_unit(p.z()))
            continue;

        rgb const c(colors.at(3 * i),
                    colors.at(3 * i + 1),
                    colors.at(3 * i + 2));
        point_t new_point;
        new_point.depth = p.z();
        new_point.color = c;

        m_layered_points.at(to_index(p.x(), p.y())).insert(new_point);
    }

    // After we've ensured, that the supplied containers are valid and the data
    // was copied over, we can update the point count of the LDI
    m_point_count += points.size() / 4;
}

std::vector<float> LayeredDepthImage::interleave_data() const
{
    assert(is_valid());

    // This will contain all the point data in an position/color interleaved
    // fashion.
    std::vector<float> data;

    for(size_t i = 0; i < m_layered_points.size(); ++i) {
        auto const ray = m_layered_points.at(i);

        // Retrieve the location of the ray on the LDI image plane
        auto const integer_location = to_coord(i);

        assert(std::numeric_limits<float>::max() >= integer_location.first);
        assert(std::numeric_limits<float>::max() >= integer_location.second);

        auto const location
            = std::make_pair(static_cast<float>(integer_location.first),
                             static_cast<float>(integer_location.second));

        auto const buffer = ray.to_buffer(location);
        std::move(buffer.cbegin(), buffer.cend(), std::back_inserter(data));
    }

    return data;
}

bool LayeredDepthImage::is_valid() const
{
    // TODO can the data also be validated?
    return m_camera.is_valid();
}

PinholeCameraModel LayeredDepthImage::get_camera() const
{
    return m_camera;
}

size_t LayeredDepthImage::point_count() const
{
    return m_point_count;
}

size_t LayeredDepthImage::bytes_per_point() const
{
    // The functions assumption are based on the following types
    static_assert(std::is_same<decltype(point_t::color), rgb>::value);
    static_assert(std::is_same<decltype(point_t::depth), float>::value);

    // vec3 for position and vec3 for color => 3 + 3
    return vec3::size() * 2 * sizeof(float);
}

size_t LayeredDepthImage::to_index(const int x, const int y) const
{
    const size_t index = y * m_camera.get_resolution().first + x;
    assert(
        m_camera.get_resolution().first *
        m_camera.get_resolution().second >= index);
    return index;
}

std::pair<size_t, size_t> LayeredDepthImage::to_coord(const size_t index) const
{
    assert(index <=
           m_camera.get_resolution().first *
           m_camera.get_resolution().second);
    return {index % m_camera.get_resolution().first,
            index / m_camera.get_resolution().second};
}

size_t LayeredDepthImage::count_points() const
{
    size_t count{0};

    for (auto const &ray : m_layered_points) {
        count += ray.point_count();
    }

    return count;
}

vec3 morphing_equation(const mat4 &perspective_source,
                       const mat4 &perspective_target,
                       const vec3 &to_source_center,
                       const vec3 &source_position)
{
    // depth == r in morphing equation
    auto const r = source_position.z();
    // the generalized disparity therefore is
    // TODO what happens with w?
    auto delta = (perspective_source * source_position.lift()).length() / r;

    auto const target_per_inv = cgv::math::inv(perspective_target);

    auto morphed_position = delta * target_per_inv * to_source_center.lift()
                            + target_per_inv * perspective_source
                                  * source_position.lift();

    // perspective divide
    morphed_position /= morphed_position.w();

    return vec3(morphed_position.x(),
                morphed_position.y(),
                morphed_position.z());
}
