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
    float depth;
    size_t splat_index;
};

class Ray
{
public:
    Ray(size_t const x, size_t const y);

    /**
     * @brief container grants access to all points on the ray as vector
     * @return a reference to the underlying vector
     */
    std::vector<point_t>& container();

    /**
     * @brief container grants access to all points on the ray as const vector
     * @return a const reference to the underlying vector
     */
    std::vector<point_t> const & container() const;
private:
    //! The location of the ray on the LDI plane
    std::pair<size_t, size_t> const m_location;

    //! All the points along the ray
    std::vector<point_t> m_points;
};

#endif // RAY_H
