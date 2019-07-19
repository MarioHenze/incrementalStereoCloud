#include "pinholecameramodel.h"

#include <cgv/math/det.h>

PinholeCameraModel::PinholeCameraModel(mat4 projection_center,
                                       mat3 mapping_matrix,
                                       std::pair<size_t, size_t> resolution) :
    projection_center(projection_center),
    mapping_matrix(mapping_matrix),
    resolution(resolution)
{

}

vec3 PinholeCameraModel::get_ray_direction(size_t u, size_t v) const
{
    assert(is_valid());
    assert(u < resolution.first);
    assert(v < resolution.second);
    return mapping_matrix * vec3(u,v,1);
}

bool PinholeCameraModel::is_valid() const
{
    return resolution.first > 0 &&
            resolution.second > 0 &&
            mapping_matrix.col(0).length() > 0 &&
            mapping_matrix.col(1).length() > 0 &&
            mapping_matrix.col(2).length() > 0;
            // TODO Test for rank(mapping_matrix) != 3
}
