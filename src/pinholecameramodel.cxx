#include "pinholecameramodel.h"

#include <cgv/math/det.h>

#include <utility>

PinholeCameraModel::PinholeCameraModel(mat4 view,
                                       mat4 projection,
                                       std::pair<size_t, size_t> resolution)
    : m_view(view),
      m_projection(projection),
      m_resolution(resolution)
{

}

mat4 PinholeCameraModel::get_view() const
{
    return m_view;
}

vec3 PinholeCameraModel::get_projective_origin() const
{
    auto const translation = get_projection().col(3);
    const_cast<vec4 &>(translation) /= translation.w();
    return vec3(translation.x(), translation.y(), translation.z());
}

mat4 PinholeCameraModel::get_vp() const
{
    // TODO is this correct? Application order is model -> view -> projection
    return  m_projection * m_view;
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
