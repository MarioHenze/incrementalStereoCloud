#pragma once

#include <atomic>
#include <forward_list>
#include <mutex>
#include <vector>
#include <queue>

#include <cgv/render/render_types.h>

using vec3 = cgv::render::render_types::vec3;
using rgb = cgv::render::render_types::rgb;

class PointCloudQuery {
private:
    std::timed_mutex m_points_mutex;
    std::queue<vec3> m_points;
    std::timed_mutex m_colors_mutex;
    std::queue<rgb> m_colors;

    std::atomic_bool m_completed{false};

public:
    PointCloudQuery() = default;

    /**
     * @brief is_complete retrieves the status of the query
     * @return true if the supplying point cloud source has determined that all
     * possible points for a given request have been provided, false otherwise.
     */
    [[nodiscard]] bool is_complete() const;

    /**
     * @brief trigger_completion marks this query as completed.
     * A completed query will no longer be supplied by the point cloud source.
     * When set by the point source the query is fullfilled entirely. If set by
     * the query side, it marks an early abortion of the query.
     */
    void trigger_completion();

    /**
     * @brief consume_points moves all currently available points of the query
     * into the supplied containers
     * @param points the container storing all positions
     * @param colors the container storing all colors
     */
    void consume_points(
            std::vector<vec3> & points,
            std::vector<rgb> & colors,
            const std::chrono::microseconds time_budget
                = std::chrono::microseconds(1000));

    /**
     * @brief supply_points will copy all supplied points into the query
     * @param points the container storing all positions
     * @param colors the container storing all colors
     */
    void supply_points(const std::vector<vec3> &points,
            const std::vector<rgb> &colors);
};
