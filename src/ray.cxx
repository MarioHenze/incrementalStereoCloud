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

size_t Ray::point_count() const
{
    return m_points.size();
}

std::vector<float> Ray::to_buffer() const
{
    std::vector<float> buffer(m_points.size());
    // The location of the ray on the image plane of the pinhole camera
    auto const location = get_location();

    for(auto const & p: m_points) {
        buffer.push_back(location.first);
        buffer.push_back(location.second);

        auto const pbuf = p.to_buffer();
        std::move(pbuf.cbegin(), pbuf.cend(), std::back_inserter(buffer));
    }

    return buffer;
}

void Ray::insert(const point_t point)
{
    // ensure depth monotonicity by sorted insertion
    auto const first_greater_depth
        = std::find_if(m_points.cbegin(),
                       m_points.cend(),
                       std::bind2nd(std::greater<point_t>(), point));

    m_points.insert(first_greater_depth, point);
}
