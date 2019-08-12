#ifndef LAYEREDDEPTHIMAGE_H
#define LAYEREDDEPTHIMAGE_H

#include <tuple>
#include <utility>
#include <vector>

#include <cgv/render/render_types.h>

#include "pinholecameramodel.h"

using vec4 = cgv::render::render_types::vec4;
using mat4 = cgv::render::render_types::mat4;
using rgb = cgv::render::render_types::rgb;

/**
 * Borrowing from the LDI concept define depth pixel as a point with color
 * and depth.
 */
struct point_t {
  rgb color;
  float depth;
  size_t splat_index;
};

class LayeredDepthImage {
public:
  LayeredDepthImage() = default;

  LayeredDepthImage(PinholeCameraModel pcm);

  void warp_reference_into(PinholeCameraModel pcm, std::vector<rgb> color,
                           std::vector<float> depth);

  /**
   * @brief add_global_points inserts the given points into the LDI
   * @param points the 4 component vectors of position
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
   * @brief interleave_data retrieves the LDI information in an OpenGL
   * friendly way for uploading
   * @return the interleaved position & color data
   *
   * The returned buffer contains at first a 3 component position vector and
   * then a 3 component color vector. Furthermore the positions will be sorted
   * from front to back
   */
  std::vector<float> interleave_data() const;

private:
  /**
   * On every ray, there are a some colored points with different depths
   */
  using ray_t = std::vector<point_t>;

  /**
   * @brief m_layered_points stores all rays going from the center of
   * projection through every pixel
   */
  std::vector<ray_t> m_layered_points;

  /**
   * @brief m_camera holds configuration of projection details of the LDI
   */
  PinholeCameraModel m_camera;

  size_t to_index(size_t x, size_t y) const;
};

#endif // LAYEREDDEPTHIMAGE_H
