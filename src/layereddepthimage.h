#ifndef LAYEREDDEPTHIMAGE_H
#define LAYEREDDEPTHIMAGE_H

#include <tuple>
#include <utility>
#include <vector>

#include <cgv/math/mat.h>
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
	size_t m_point_count{ 0 };

	/**
	 @brief m_camera holds configuration of projection details of the LDI

	 As all points are stored in an camera model relative way, a change of this
	 results in the invalidation of the entire LDI. If a change of camera is
	 necessary, a new LDI with updated camera needs to be created and the current
	 LDI needs to be warped into.
	 */
	PinholeCameraModel const m_camera;

public:
	//! In the interleaved data buffer the first 3 floats form the logical 
	//! position
	static constexpr size_t positional_components = 3;
	static constexpr size_t position_offset = 0;

	//! In the interleaved data buffer the next 3 floats form a logical RGB
	//! color
	static constexpr size_t color_components = 3;
	static constexpr size_t color_offset = 3;

	//! Every entry after the stride begins a new point
	static constexpr size_t stride = positional_components + color_components;

	using buffer_type = std::vector<float>;

	/*
	Compile-time constant for the number of bytes necessary to store a point
	in a buffer. Is composed of the bytes necessary for color and depth and
	two additional values for the x and y position on the LDI plane.
	*/
	static constexpr size_t bytes_per_point{
		(decltype(m_layered_points)::value_type::value_type::byte_count)
		+ 2 * sizeof(buffer_type::value_type) };

	LayeredDepthImage() = default;

    // TODO Reflect behavoir, when new LDI is resized
    LayeredDepthImage(LayeredDepthImage const &other) = default;

    explicit LayeredDepthImage(const PinholeCameraModel& pcm);

	/**
	@brief get_density returns an image with the amount of points on every 
	pixel of the LDI
	*/
	cgv::math::mat<int> get_density() const;

	/**
	@brief warp_reference_into warps another LDI into the current LDI frame
	@param ldi is the LDI which should be merged into the current LDI
	*/
    void warp_reference_into(const LayeredDepthImage& ldi);

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
    [[nodiscard]] buffer_type interleave_data() const;

	/**
	@brief position_data retrives the LDI points in world coordinates,
	suitable for direct rendering in OpenGL
	@return a raw float buffer with the x, y, z position of the LDI points
	*/
	[[nodiscard]] std::vector<vec3> position_data() const;

	/**
	@brief color_data retrives the LDI point colors, suitable for direct 
	rendering in OpenGL
	@return a raw float buffer with the r, g, b colors of the LDI points
	*/
	[[nodiscard]] std::vector<rgb> color_data() const;

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
};

#endif // LAYEREDDEPTHIMAGE_H
