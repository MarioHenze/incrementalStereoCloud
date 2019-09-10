#include "pointcloudquery.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>

bool PointCloudQuery::is_complete() const
{
    return m_completed;
}

void PointCloudQuery::trigger_completion()
{
    if (m_completed)
        std::cerr << __FILE__
                  << __func__
                  << __LINE__
                  << ": Query is already marked as completed!";

    m_completed = true;
}

void PointCloudQuery::consume_points(
        std::vector<float> &points,
        std::vector<float> &colors,
        const std::chrono::microseconds time_budget)
{
    // Get initial timestamp
    auto const begin_time = std::chrono::high_resolution_clock::now();

    // There are two locks in this function. Ensure that the locking time is
    // bound to the upper limit of the time budget
    constexpr size_t lock_count = 2;
    std::chrono::microseconds time_slice(time_budget / lock_count);

    std::unique_lock points_lock(m_points_mutex, time_slice);
    if(!points_lock.owns_lock()) return;
    std::unique_lock colors_lock(m_colors_mutex, time_slice);
    if(!colors_lock.owns_lock()) return;

    // Return if time budget is already depleted
    if (std::chrono::high_resolution_clock::now() - begin_time >= time_budget)
        return;

    assert(m_points.size() == m_colors.size());
    auto const has_points = !m_points.empty() && !m_colors.empty()
                            && m_points.size() >= 3 && m_colors.size() >= 3;
    while (has_points) {
        // As positions and colors are 3 component vectors copy float data in
        // chunks
        for (size_t i = 0; i < 3; ++i) {
            points.push_back(m_points.front());
            m_points.pop();
            colors.push_back(m_colors.front());
            m_colors.pop();
        }

        // When time budget is depleted, abort copying
        if (std::chrono::high_resolution_clock::now() - begin_time
            >= time_budget)
            break;
    }

    assert(points.size() == colors.size());
}

void PointCloudQuery::supply_points(std::vector<float> const & points,
                                    std::vector<float> const & colors)
{
    assert(!m_completed);
    assert(points.size() % 3 == 0);
    assert(colors.size() % 3 == 0);
    assert(points.size() == colors.size());

    std::scoped_lock lock(m_points_mutex, m_colors_mutex);
    for (auto const &p : points)
        m_points.push(p);
    for (auto const &c : colors)
        m_colors.push(c);

    assert(m_points.size() == m_colors.size());
}
