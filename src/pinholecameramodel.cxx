#include "pinholecameramodel.h"

#include <cgv/math/det.h>

#include <utility>

PinholeCameraModel::PinholeCameraModel(mat4 view,
                                       mat4 projection,
                                       std::pair<size_t, size_t> resolution)
    : m_view(std::move(view)),
      m_projection(std::move(projection)),
      m_resolution(std::move(resolution))
{

}

mat4 PinholeCameraModel::get_view() const
{
    return m_view;
}

vec3 PinholeCameraModel::get_projective_origin() const
{
    auto const translation = get_view().col(3);
    const_cast<vec4 &>(translation) /= translation.w();
    return vec3(translation.x(), translation.y(), translation.z());
}

mat4 PinholeCameraModel::get_projection() const
{
    return m_projection;
}

std::pair<size_t, size_t> PinholeCameraModel::get_resolution() const
{
    assert(is_valid());
    return m_resolution;
}

bool PinholeCameraModel::is_valid() const
{
    return m_resolution.first > 0 && m_resolution.second > 0;
}
