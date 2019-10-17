#include "conversion.h"

#include <cgv/render/render_types.h>

std::vector<vec3> convert_to_vec3(const std::vector<float> &src)
{
    std::vector<vec3> ret;

    assert(src.size() % 3);
    for (size_t i = 0; i < src.size() / 3; ++i) {
        ret.emplace_back(src.at(3 * i), src.at(3 * i + 1), src.at(3 * i + 2));
    }

    return ret;
}

std::vector<rgb> convert_to_rgb(const std::vector<float> &src)
{
    std::vector<rgb> ret;

    assert(src.size() % 3);
    for (size_t i = 0; i < src.size() / 3; ++i) {
        ret.emplace_back(src.at(3 * i), src.at(3 * i + 1), src.at(3 * i + 2));
    }

    return ret;
}

std::vector<float> convert_to_floats(const std::vector<vec3> &src)
{
    std::vector<float> ret;

    for (auto const &v : src) {
        ret.emplace_back(v.x());
        ret.emplace_back(v.y());
        ret.emplace_back(v.z());
    }

    return ret;
}

std::vector<float> convert_to_floats(const std::vector<rgb> &src)
{
    std::vector<float> ret;

    for (auto const &v : src) {
        ret.emplace_back(v.R());
        ret.emplace_back(v.G());
        ret.emplace_back(v.B());
    }

    return ret;
}
