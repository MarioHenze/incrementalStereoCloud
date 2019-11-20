#include "layereddepthimage.h"

#include <algorithm>
#include <limits>

#include <pmmintrin.h>

//#include <boost/align/is_aligned.hpp>

#include <cgv/math/inv.h>

#include "conversion.h"

LayeredDepthImage::LayeredDepthImage(const PinholeCameraModel &pcm)
    : m_layered_points(pcm.get_resolution().first *
                       pcm.get_resolution().second),
      m_camera(pcm)
{

}

cgv::math::mat<int> LayeredDepthImage::get_density() const
{
	assert(is_valid());

	auto const width = m_camera.get_resolution().first;
	auto const height = m_camera.get_resolution().second;
	assert(width * height == m_layered_points.size());

	cgv::math::mat<int> density(height, width);
	
	for (int i = 0; i < m_layered_points.size(); ++i)
	{
		auto const coord = to_coord(i);
		density(coord.second, coord.first) = 
			m_layered_points[i].point_count();
	}

	return density;
}

void LayeredDepthImage::warp_reference_into(const LayeredDepthImage& ldi)
{
	// Determine the arguments for the morphing equation
	auto const other_perspective = ldi.get_camera().get_projection();
	auto const this_perspective = get_camera().get_projection();
	auto const other_origin = ldi.get_camera().get_projective_origin();

	// Retrieve data buffers from other LDI for easy access
	auto const other_points = ldi.position_data();
	auto const other_color = ldi.color_data();
	assert(other_points.size() == other_color.size());

	for (size_t i = 0; i < other_points.size(); i++)
	{
		auto const morphed_point = morphing_equation(
			other_perspective,
			this_perspective,
			other_origin,
			other_points[i]);
		
		// If the morphed point is outside of our target camera, skip further
		// computations
		auto const resolution = get_camera().get_resolution();
		if (
			morphed_point.x() < 0.f ||
			morphed_point.y() < 0.f ||
			morphed_point.x() >= resolution.first ||
			morphed_point.y() >= resolution.second ||
			isnan(morphed_point.x()) ||
			isnan(morphed_point.y()))
			continue;
		else
		{
			point_t const new_point =
			{
				other_color[i],
				morphed_point.z()
			};
			m_layered_points[to_index(morphed_point.x(), morphed_point.y())].insert(
				new_point
			);
		}

	}
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
    auto const is_unit = [](float const &f) { return (-1 <= f) && (1 >= f); };

    for (size_t i = 0; i < (points.size() / 3); i++) {
        // the given points are in global space and need to be transformed into
        // the clip space of the LDI camera
        vec4 point = m_camera.get_projection() * m_camera.get_view()
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
    m_point_count += points.size() / 3;
}

void LayeredDepthImage::add_transformed_points(const std::vector<vec3> &points,
                                               const std::vector<rgb> &colors)
{
    assert(colors.size() == points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        auto const &p = points.at(i);

        // Skip points outside the view frustum
		// TODO are the pixel bounds reasonable?
		// width 900 -> pixel position from 0 to 899?
        if (0 > p.x() || 0 > p.y() || 0 > p.z()
            || m_camera.get_resolution().first < p.x()
            || m_camera.get_resolution().second < p.y())
            continue;

        point_t new_point;
        new_point.color = colors.at(i);
        new_point.depth = p.z();
        m_layered_points.at(to_index(p.x(), p.y())).insert(new_point);
    }

    m_point_count += points.size();
}

std::vector<float> LayeredDepthImage::interleave_data() const
{
    assert(is_valid());

    // This will contain all the point data in an position/color interleaved
    // fashion.
    std::vector<float> data;

    for(size_t i = 0; i < m_layered_points.size(); ++i) {
        auto const & ray = m_layered_points[i];

        if (ray.point_count() == 0)
            continue;

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

std::vector<vec3> LayeredDepthImage::position_data() const
{
	std::vector<vec3> buffer;

	auto const to_world = cgv::math::inv(
		m_camera.get_sensor() *
		m_camera.get_projection() * 
		m_camera.get_view());

	for (size_t i = 0; i < m_layered_points.size(); ++i) {
		auto const & ray = m_layered_points[i];
		auto const location = to_coord(i);

		for (auto const& point : ray.underlying_data()) {
			vec4 window_pos(location.first, location.second, point.depth, 1);
			auto const gpos = to_world * window_pos;

			buffer.emplace_back(
				gpos.x() / gpos.w(),
				gpos.y() / gpos.w(),
				gpos.z() / gpos.w()
			);
		}
	}

	return buffer;
}

std::vector<rgb> LayeredDepthImage::color_data() const
{
	std::vector<rgb> buffer;

	for (const auto& ray : m_layered_points) {
		for (const auto& point : ray.underlying_data()) {
			buffer.emplace_back(point.color);
		}
	}

	return buffer;
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

size_t LayeredDepthImage::to_index(const int x, const int y) const
{
    assert(x >= 0);
    assert(y >= 0);
    assert(static_cast<size_t>(x) <= m_camera.get_resolution().first);
    assert(static_cast<size_t>(y) <= m_camera.get_resolution().second);

    const size_t index = static_cast<size_t>(y)
                             * m_camera.get_resolution().first
                         + static_cast<size_t>(x);
    return index;
}

std::pair<size_t, size_t> LayeredDepthImage::to_coord(const size_t index) const
{
	auto const resolution = m_camera.get_resolution();
    assert(index <=
           resolution.first *
           resolution.second);
	std::pair<size_t, size_t> const coord{
		index % resolution.first,
		index / resolution.first };
	assert(coord.first < resolution.first);
	assert(coord.second < resolution.second);
	return coord;
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
