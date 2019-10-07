#include "pointcloudsource.h"

#include <limits>
#include <vector>

PointCloudSource::PointCloudSource(const std::string &filepath) :
    m_point_cloud(filepath)
{

}

void PointCloudSource::compute_queries()
{
    std::unique_lock queue_lock(m_pending_queries_mutex);

    // Wait for new queries and guard against spurious wakeups
    m_queries_present.wait(queue_lock,
                           [this] { return !m_pending_queries.empty(); });

    /*
     * At this point we have aquired the lock for all pending queries and know
     * that new ones are present for processing
     */

    assert(m_point_cloud.has_colors());

    auto const point_count = m_point_cloud.get_nr_points();
    std::vector<float> points(point_count * 3);
    std::vector<float> colors(point_count * 3);

    for (size_t i = 0; i < point_count; ++i) {
        auto const point = m_point_cloud.pnt(i);
        // TODO w divide??
        points.push_back(point.x());
        points.push_back(point.y());
        points.push_back(point.z());
    }

    for (size_t i = 0; i < point_count; ++i) {
        auto const color = m_point_cloud.clr(i);
        points.push_back(color.R());
        points.push_back(color.G());
        points.push_back(color.B());
    }

    m_pending_queries.front()->supply_points(points, colors);
}

std::shared_ptr<PointCloudQuery> PointCloudSource::queryPoints(
        const std::chrono::microseconds time_budget)
{
    constexpr size_t lock_count = 1;
    std::chrono::microseconds time_slice(time_budget / lock_count);

    std::unique_lock pending_queries_lock(m_pending_queries_mutex, time_slice);
    if(!pending_queries_lock.owns_lock())
        return nullptr;

    std::shared_ptr<PointCloudQuery> new_query =
            std::make_shared<PointCloudQuery>();
    m_pending_queries.push(new_query);
    return new_query;
}

void PointCloudSource::update_query(std::shared_ptr<PointCloudQuery> &pcq)
{
    assert(!pcq->is_complete());

    //TODO get a narrow pinhole camera for query and get points
    assert(std::numeric_limits<int>::max() >= m_point_cloud.get_nr_points());
    const int point_count = static_cast<int>(m_point_cloud.get_nr_points());
    std::vector<float> points;
    for (int i = 0; i < point_count; i++)
    {
        const auto p = m_point_cloud.pnt(i);
        points.emplace_back(p.x());
        points.emplace_back(p.y());
        points.emplace_back(p.z());
        points.emplace_back(p.w());
    }
    std::vector<float> colors;
    for (int i = 0; i < point_count; i++)
    {
        const auto c = m_point_cloud.clr(i);
        colors.emplace_back(c.R());
        colors.emplace_back(c.G());
        colors.emplace_back(c.B());
    }

    pcq->supply_points(points, colors);
}
