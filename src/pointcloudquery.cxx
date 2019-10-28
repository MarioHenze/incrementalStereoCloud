#include "pointcloudquery.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>

PointCloudQuery::~PointCloudQuery()
{
    if (!m_completed || !m_consumed)
        std::cerr << "A point cloud query has been deleted, which has not been "
                     "completed or consumed entirely!";
}

bool PointCloudQuery::is_complete() const
{
    return m_completed;
}

bool PointCloudQuery::is_consumed() const
{
    return m_consumed;
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
        std::vector<vec3> &points,
        std::vector<rgb> &colors,
        const std::chrono::microseconds time_budget)
{
    // Get initial timestamp
    auto const begin_time = std::chrono::high_resolution_clock::now();

    // There are two locks in this function. Ensure that the locking time is
    // bound to the upper limit of the time budget
    constexpr size_t lock_count = 2;
    const std::chrono::microseconds time_slice(time_budget / lock_count);

    std::unique_lock points_lock(m_points_mutex, time_slice);
    if(!points_lock.owns_lock()) return;
    std::unique_lock colors_lock(m_colors_mutex, time_slice);
    if(!colors_lock.owns_lock()) return;

    // Return if time budget is already depleted
    if (std::chrono::high_resolution_clock::now() - begin_time >= time_budget)
        return;

    assert(m_points.size() == m_colors.size());
    std::function<bool()> const has_points = [this] {
        return !m_points.empty() && !m_colors.empty();
    };
    while (has_points()) {
        points.push_back(m_points.front());
        m_points.pop();
        colors.push_back(m_colors.front());
        m_colors.pop();

        // When time budget is depleted, abort copying
        if (std::chrono::high_resolution_clock::now() - begin_time
            >= time_budget)
            return;
    }

    /*
     * After the while loop, all points have been copied. If the supplying
     * thread has signaled, that this query is completed, this query can be
     * marked as consumed
     */
    if (m_completed)
        m_consumed = true;

    assert(points.size() == colors.size());
}

void PointCloudQuery::supply_points(const std::vector<vec3> &points,
                                    const std::vector<rgb> &colors)
{
    assert(!m_completed);
    assert(points.size() == colors.size());

    std::scoped_lock lock(m_points_mutex, m_colors_mutex);
    for (auto const &p : points)
        m_points.push(p);
    for (auto const &c : colors)
        m_colors.push(c);

    assert(m_points.size() == m_colors.size());
}
