#ifndef PINHOLECAMERAMODEL_H
#define PINHOLECAMERAMODEL_H

#include <utility>
#include <cgv/render/render_types.h>

using mat3 = cgv::render::render_types::mat3;
using mat4 = cgv::render::render_types::mat4;
using vec3 = cgv::render::render_types::vec3;

class PinholeCameraModel
{
public:
    mat4 projection_center;
    /**
     * @brief mapping_matrix Maps a given image plane coordinate into a ray
     * direction from center of projection
     */
    mat3 mapping_matrix;
    std::pair<size_t, size_t> resolution;

    PinholeCameraModel(mat4 projection_center,
                       mat3 mapping_matrix,
                       std::pair<size_t, size_t> resolution);

    /**
     * @brief get_ray_direction computes the ray direction from center of
     * projection through the pixel
     * @param u coordinate of pixel in image plane
     * @param v coordinate of pixel in image plane
     * @return the ray direction
     */
    vec3 get_ray_direction(size_t u, size_t v) const;

    bool is_valid() const;
};

#endif // PINHOLECAMERAMODEL_H
