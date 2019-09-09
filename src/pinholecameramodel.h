#ifndef PINHOLECAMERAMODEL_H
#define PINHOLECAMERAMODEL_H

#include <cgv/render/render_types.h>
#include <utility>

using mat3 = cgv::render::render_types::mat3;
using mat4 = cgv::render::render_types::mat4;
using vec3 = cgv::render::render_types::vec3;

class PinholeCameraModel
{
public:
    PinholeCameraModel() = default;

    PinholeCameraModel(mat4 m_projection_center,
                       mat3 m_mapping_matrix,
                       std::pair<size_t, size_t> m_resolution);

    /**
     * @brief get_ray_direction computes the ray direction from center of
     * projection through the pixel
     * @param u coordinate of pixel in image plane
     * @param v coordinate of pixel in image plane
     * @return the ray direction
     */
    [[nodiscard]] vec3 get_ray_direction(size_t u, size_t v) const;

    /**
     * @brief get_mvp returns a transformation from global to LDI local
     * coordinate system
     * @return the transformation
     */
    [[nodiscard]] mat4 get_mvp() const;

    /**
     * @brief get_mapping returns a transformation from x,y + depth on ray to
     * LDI local coordinates
     * @return the mapping transformation
     */
    [[nodiscard]] mat3 get_mapping() const;

    /**
     * @brief get_resolution returns the width and height of the LDI
     * @return a pair with width and height
     */
    [[nodiscard]] std::pair<size_t, size_t> get_resolution() const;

    /**
     * @brief is_valid checks, wether the supplied configuration yields a valid
     * transformation
     * @return true if configuration is not degenerate, false otherwise
     */
    [[nodiscard]] bool is_valid() const;

private:
    /**
     * @brief m_projection_center contains a transformation from global
     * coordinates into the clip space of the LDI
    */
    mat4 m_projection_center;

    /**
     * @brief m_mapping_matrix Maps a given image plane coordinate into a ray
     * direction from center of projection
     */
    mat3 m_mapping_matrix;
    std::pair<size_t, size_t> m_resolution;
};

#endif // PINHOLECAMERAMODEL_H
