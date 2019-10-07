#include "pointcloudsource.h"

#include <limits>
#include <vector>

bool PointCloudSource::unprocessed_present() const
{
    if (m_pending_queries.empty())
        return false;

    auto const first_uncomplete
        = std::find_if(m_pending_queries.cbegin(),
                       m_pending_queries.cend(),
                       [](decltype(m_pending_queries)::value_type const query) {
                           return query->is_complete();
                       });
    return (first_uncomplete == m_pending_queries.cend());
}

PointCloudSource::PointCloudSource(const std::string &filepath) :
    m_point_cloud(filepath)
{

}

void PointCloudSource::compute_queries()
{
    std::unique_lock queue_lock(m_pending_queries_mutex);

    // Wait for new queries and guard against spurious wakeups
    m_queries_present.wait(queue_lock,
                           [this] { return unprocessed_present(); });

    /*
     * At this point we have aquired the lock for all pending queries and know
     * that new ones are present for processing
     */

    std::vector<float> points;
    // When no color is present in the data, use white points
    std::vector<float> colors
        = m_point_cloud.has_colors()
              ? std::vector<float>()
              : std::vector<float>(m_point_cloud.get_nr_points() * 3, 1.f);

    //TODO get a narrow pinhole camera for query and get points
    assert(std::numeric_limits<int>::max() >= m_point_cloud.get_nr_points());
    const int point_count = static_cast<int>(m_point_cloud.get_nr_points());

    for (int i = 0; i < point_count; ++i) {
        auto const point = m_point_cloud.transformed_pnt(i);
        // TODO w divide??
        points.push_back(point.x());
        points.push_back(point.y());
        points.push_back(point.z());
    }

    if (m_point_cloud.has_colors())
        for (int i = 0; i < point_count; ++i) {
            auto const color = m_point_cloud.clr(i);
            colors.push_back(color.R());
            colors.push_back(color.G());
            colors.push_back(color.B());
        }

    m_pending_queries.front()->supply_points(points, colors);
    m_pending_queries.front()->trigger_completion();


}

std::optional<std::shared_ptr<PointCloudQuery>>
PointCloudSource::get_finished_query()
{
    if (m_pending_queries.empty())
        return {};

    auto const first_complete
        = std::find_if(m_pending_queries.cbegin(),
                       m_pending_queries.cend(),
                       [](decltype(m_pending_queries)::value_type query) {
                           return query->is_complete();
                       });
    return first_complete == m_pending_queries.cend()
               ? decltype(get_finished_query())()
               : decltype(get_finished_query())(*first_complete);
}

void PointCloudSource::queryPoints(const std::chrono::microseconds time_budget)
{
    constexpr size_t lock_count = 1;
    std::chrono::microseconds time_slice(time_budget / lock_count);

    std::unique_lock pending_queries_lock(m_pending_queries_mutex, time_slice);
    if(!pending_queries_lock.owns_lock())
        return;

    std::shared_ptr<PointCloudQuery> new_query =
            std::make_shared<PointCloudQuery>();
    m_pending_queries.push_back(new_query);
    m_queries_present.notify_one();
}
