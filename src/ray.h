#ifndef RAY_H
#define RAY_H

#include <cstddef>
#include <utility>
#include <vector>

#include <cgv/render/render_types.h>

using rgb = cgv::render::render_types::rgb;

/**
 * Borrowing from the LDI concept define depth pixel as a point with color
 * and depth.
 */
struct point_t
{
    rgb color;
    float depth{};
    size_t splat_index{};

	static constexpr size_t byte_count{sizeof (rgb) + sizeof(depth)};

    /**
     * @brief to_buffer transforms the point compound into a float buffer
     * @return a buffer containing depth and then RGB color
     */
    [[nodiscard]] std::vector<float> to_buffer() const;

    bool operator>(point_t const other) const;
};

class Ray
{
private:
	//! All the points along the ray
	std::vector<point_t> m_points;

public:
	using value_type = decltype(m_points)::value_type;

    Ray() = default;

    /**
     * @brief point_count retrieves how many points are on the ray
     * @return the count of points on the ray
     */
    [[nodiscard]] size_t point_count() const;

    /**
     * @brief to_buffer retrieves a buffer friendly version of the ray
     * @param location the x and y location of the ray on the LDI image plane
     * @return a vector with location/depth and color interleaved
     *
     * This function is provided for quick OpenGL friendly buffer creation. As
     * all rays of an LDI contain a variable amount of points, this function in
     * combination with stl algorithms can transform the set of all rays into
     * the desired representation
     */
    [[nodiscard]] std::vector<float> to_buffer(
        std::pair<float, float> location) const;

    /**
     * @brief insert will place a colored point at the specified depth
     * @param depth at which the point lays on the ray
     * @param color of the point
     */
	void insert(const point_t point);
};

#endif // RAY_H
