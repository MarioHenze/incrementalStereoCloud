#include "pointcloudsource.h"

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
        return std::make_shared<PointCloudQuery>(nullptr);

    std::shared_ptr<PointCloudQuery> new_query =
            std::make_shared<PointCloudQuery>();
    m_pending_queries.push(new_query);
    return new_query;
}

void PointCloudSource::update_query(std::shared_ptr<PointCloudQuery> &pcq)
{

}
