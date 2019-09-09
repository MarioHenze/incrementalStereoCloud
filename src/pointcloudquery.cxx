#include "pointcloudquery.h"

#include <cassert>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>

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
    constexpr size_t lock_count = 2;
    std::chrono::microseconds time_slice(time_budget / lock_count);

    std::unique_lock points_lock(m_points_mutex, time_slice);
    if(!points_lock.owns_lock()) return;
    std::unique_lock colors_lock(m_colors_mutex, time_slice);
    if(!colors_lock.owns_lock()) return;

    points.insert(points.end(), m_points.begin(), m_points.end());
    colors.insert(colors.end(), m_colors.begin(), m_colors.end());

    assert(points.size() == colors.size());

    m_points.clear();
    m_colors.clear();
}

void PointCloudQuery::supply_points(std::vector<float> const & points,
                                    std::vector<float> const & colors)
{
    assert(!m_completed);
    assert(points.size() == colors.size());

    std::scoped_lock lock(m_points_mutex, m_colors_mutex);
    m_points.insert_after(m_points.end(), points.cbegin(), points.cend());
    m_colors.insert_after(m_colors.end(), colors.cbegin(), colors.cend());
}
