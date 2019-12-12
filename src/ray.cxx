#include "ray.h"

#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>

std::vector<float> point_t::to_buffer() const
{
    return {depth, color.R(), color.G(), color.B()};
}

bool point_t::operator>(const point_t other) const
{
    return depth > other.depth;
}

std::vector<point_t> const& Ray::underlying_data() const
{
	return m_points;
}

size_t Ray::point_count() const
{
    return m_points.size();
}

std::vector<float> Ray::to_buffer(std::pair<float, float> location) const
{
    std::vector<float> buffer;
	buffer.reserve(
		m_points.size()
		* (sizeof(decltype(decltype(m_points)::value_type::depth)) 
			+ sizeof(decltype(decltype(m_points)::value_type::color))));

    for (auto const &p : m_points) {
        buffer.push_back(location.first);
        buffer.push_back(location.second);

        auto const pbuf = p.to_buffer();
        std::move(pbuf.cbegin(), pbuf.cend(), std::back_inserter(buffer));
    }

    return buffer;
}

void Ray::insert(const point_t point)
{
    auto const predicate = [&point](decltype(m_points)::value_type other) {
        return other > point;
    };

    /*// ensure depth monotonicity by sorted insertion
    auto const first_greater_depth = std::find_if(m_points.cbegin(),
                                                  m_points.cend(),
                                                  predicate);

    m_points.insert(first_greater_depth, point);*/
    m_points.push_back(point);
}
