#include "pinholecameramodel.h"

#include <cgv/math/det.h>
#include <cgv/math/ftransform.h>

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

mat4 PinholeCameraModel::get_sensor() const
{
	auto const resolution = get_resolution();
	return independant_viewport(resolution.first, resolution.second);
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

mat4 independant_viewport(float width, float height)
{
	// To convert independant from window position, a simple scaling
	// transformation is sufficient.

	//	w/2	0	0	w/2
	//	0	h/2	0	h/2
	//	0	0	1	0
	//	0	0	0	1

	auto const half_width = width / 2.f;
	auto const half_height = height / 2.f;
	mat4 ret = cgv::math::identity4<float>();

	ret(0, 0) = half_width;
	ret(0, 3) = half_width;
	ret(1, 1) = half_height;
	ret(1, 3) = half_height;

	return ret;
}
