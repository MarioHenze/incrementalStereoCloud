#include "pinholecameramodel.h"

#include <cgv/math/det.h>

#include <utility>

PinholeCameraModel::PinholeCameraModel(mat4 projection_center,
                                       mat3 mapping_matrix,
                                       std::pair<size_t, size_t> resolution)
    : m_projection_center(std::move(projection_center)),
      m_mapping_matrix(std::move(mapping_matrix)),
      m_resolution(std::move(resolution))
{

}

vec3 PinholeCameraModel::get_ray_direction(size_t u, size_t v) const
{
    assert(is_valid());
    assert(u < m_resolution.first);
    assert(v < m_resolution.second);
    return m_mapping_matrix * vec3(u,v,1);
}

mat4 PinholeCameraModel::get_mvp() const
{
    assert(is_valid());
    return m_projection_center;
}

mat3 PinholeCameraModel::get_mapping() const
{
    assert(is_valid());
    return m_mapping_matrix;
}

std::pair<size_t, size_t> PinholeCameraModel::get_resolution() const
{
    assert(is_valid());
    return m_resolution;
}

bool PinholeCameraModel::is_valid() const
{
    return m_resolution.first > 0 &&
            m_resolution.second > 0 &&
            m_mapping_matrix.col(0).length() > 0 &&
            m_mapping_matrix.col(1).length() > 0 &&
            m_mapping_matrix.col(2).length() > 0;
            // TODO Test for rank(mapping_matrix) != 3
}
