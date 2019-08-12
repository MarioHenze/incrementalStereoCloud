#include "pointcloudsource.h"

#include <limits>
#include <vector>

PointCloudSource::PointCloudSource(const std::string &filepath) :
    m_point_cloud(filepath)
{

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
