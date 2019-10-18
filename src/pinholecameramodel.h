#ifndef PINHOLECAMERAMODEL_H
#define PINHOLECAMERAMODEL_H

#include <cgv/render/render_types.h>
#include <utility>

using mat3 = cgv::render::render_types::mat3;
using mat4 = cgv::render::render_types::mat4;
using vec3 = cgv::render::render_types::vec3;
using vec4 = cgv::render::render_types::vec4;

class PinholeCameraModel
{
public:
    PinholeCameraModel() = default;

    PinholeCameraModel(mat4 view,
                       mat4 projection,
                       std::pair<size_t, size_t> m_resolution);

    /**
     * @brief get_projection retrieves the projective transformation of the
     * camera
     * @return a matrix which transforms to the perspective of the pinhole
     * camera
     *
     * This function gives the mapping matrix used in the process of morphing
     * points from one LDI to another.
     */
    [[nodiscard]] mat4 get_projection() const;

    /**
     * @brief get_model_view retrieves the view transformation for the camera
     * @return a matrix which transforms to the camera space
     */
    [[nodiscard]] mat4 get_view() const;

    /**
     * @brief get_projective_origin retrieves the world location of the cameras
     * projective center
     * @return the location of the camera
     */
    [[nodiscard]] vec3 get_projective_origin() const;


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
     * @brief m_view contains the view transformation of the camera
     */
    mat4 m_view;

    /**
     * @brief m_projection contains the projection part of the camera
     */
    mat4 m_projection;

    /**
     * @brief m_resolution contains the horizontal and vertical resolution of
     * the pinhole camera model
     */
    std::pair<size_t, size_t> m_resolution;
};

#endif // PINHOLECAMERAMODEL_H
