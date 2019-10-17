#ifndef CONVERSION_H
#define CONVERSION_H

#include <vector>

#include <cgv/render/render_types.h>
#include <cgv/math/mat.h>

using vec3 = cgv::render::render_types::vec3;
using mat4 = cgv::render::render_types::mat4;
using rgb = cgv::render::render_types::rgb;

std::vector<vec3> convert_to_vec3(std::vector<float> const &src);
std::vector<rgb> convert_to_rgb(std::vector<float> const &src);

std::vector<float> convert_to_floats(std::vector<vec3> const &src);
std::vector<float> convert_to_floats(std::vector<rgb> const &src);

template<typename T, size_t N, size_t M>
cgv::math::fmat<T, N, M> convert_mat(cgv::math::mat<T> const &src)
{
    assert(src.size() == N * M);

    cgv::math::fmat<T, N, M> ret;

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < M; ++j) {
            ret(i, j) = src(i, j);
        }
    }

    return ret;
}

#endif // CONVERSION_H
