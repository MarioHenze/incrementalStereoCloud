#include "ray.h"

Ray::Ray(const size_t x, const size_t y) :
    m_location{x, y}
{

}

std::vector<point_t> &Ray::container()
{
    return m_points;
}

const std::vector<point_t> &Ray::container() const
{
    return m_points;
}
