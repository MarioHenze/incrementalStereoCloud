#ifndef LAYEREDDEPTHIMAGE_H
#define LAYEREDDEPTHIMAGE_H

#include <tuple>
#include <utility>
#include <vector>

#include <cgv/render/render_types.h>

#include "pinholecameramodel.h"
#include "ray.h"

using vec4 = cgv::render::render_types::vec4;
using mat4 = cgv::render::render_types::mat4;
using rgb = cgv::render::render_types::rgb;

/**
 * @brief morphing_equation morphs a given point from one LDI to another
 * @param perspective_source the perspective transformation of the source LDI
 * @param perspective_target the perspective transformation of the target LDI
 * @param to_target_center is the vector from the target center of projection to
 * the source center of projection
 * @param source_position the point position in the source LDI
 * @return the point position in the new LDI
 */
vec3 morphing_equation(const mat4 &perspective_source,
                       const mat4 &perspective_target,
                       const vec3 &to_source_center,
                       const vec3 &source_position);

class LayeredDepthImage
{
public:
    LayeredDepthImage() = default;

    // TODO Reflect behavoir, when new LDI is resized
    LayeredDepthImage(LayeredDepthImage const &other) = default;

    explicit LayeredDepthImage(const PinholeCameraModel& pcm);

    void warp_reference_into(const PinholeCameraModel &pcm,
                             const std::vector<rgb> &color,
                             const std::vector<float>& depth);

    /**
     * @brief add_global_points inserts the given points into the LDI
     * @param points the 3 component vectors of position
     * @param colors the 3 component vectors of colors
     *
     * This function converts the global positions into the specific LDI view
     * space. This results in more color points with depth on every ray.
     * The vec4 position should have a w component of 1 because perspective
     * transformation and perspective divide is happening inside.
     */
    void add_global_points(std::vector<float> const &points,
                           std::vector<float> const &colors);

    /**
     * @brief add_transformed_points inserts the given points into the LDI
     * @param points the 3 component vectors of position
     * @param colors the 3 component vectors of colors
     *
     * This function assumes, that the supplied points were already processed
     * into the LDI local coordinate system. Therefore it just copies all LDI-
     * visible points.
     */
    void add_transformed_points(std::vector<vec3> const &points,
                                std::vector<rgb> const &colors);

    /**
     * @brief interleave_data retrieves the LDI information in an OpenGL
     * friendly way for uploading
     * @return the interleaved position & color data
     *
     * The returned buffer contains at first a 3 component position vector and
     * then a 3 component color vector.
     */
    [[nodiscard]] std::vector<float> interleave_data() const;

    /**
     * @brief is_valid checks if the LDI configuration and data is valid
     * @return true if the LDI can be considered in working state, false
     * otherwise
     */
    [[nodiscard]] bool is_valid() const;

    /**
     * @brief get_camera retrieves the MVP transformation of the pinhole camera
     * @return the MVP transformation
     */
    [[nodiscard]] PinholeCameraModel get_camera() const;

    /**
     * @brief point_count retrieves the current amount of points in the LDI
     * @return the amount of points
     */
    [[nodiscard]] size_t point_count() const;

    /**
     * @brief bytes_per_point retrieves the count of bytes necessary to store
     * all data of a single point
     * @return byte count of one point
     */
    [[nodiscard]] size_t bytes_per_point() const;

private:
    /**
     * @brief to_index returns the index for a given 2D position
     * @param x horizontal position
     * @param y vertical position
     * @return linear index
     */
    [[nodiscard]] size_t to_index(const int x, const int y) const;

    /**
     * @brief to_coord retrieves the two dimensional position of a given index
     * @param index of the ray
     * @return the x and y position of the ray
     */
    [[nodiscard]] std::pair<size_t, size_t> to_coord(size_t const index) const;

    /**
     * @brief count_points counts all points along all rays in the LDI
     * @return the actual amount of points in the LDI
     *
     * This function can be used as an expensive check to see, if the adhoc
     * point count differs from the actual point count in the LDI.
     */
    [[nodiscard]] size_t count_points() const;

    /**
     * @brief m_layered_points stores all rays going from the center of
     * projection through every pixel
     */
    std::vector<Ray> m_layered_points;

    //! The amount of points in the LDI(-subset) of the point source
    size_t m_point_count{0};

    /**
     * @brief m_camera holds configuration of projection details of the LDI
     */
    PinholeCameraModel m_camera;
};

#endif // LAYEREDDEPTHIMAGE_H
