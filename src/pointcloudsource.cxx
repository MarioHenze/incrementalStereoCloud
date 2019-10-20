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
                       [](decltype(m_pending_queries)::value_type const &query) {
                           return query->is_complete();
                       });
    return (first_uncomplete == m_pending_queries.cend());
}

bool PointCloudSource::consumed_present() const
{
    if (m_pending_queries.empty())
        return false;

    auto const first_consumed
        = std::find_if(m_pending_queries.cbegin(),
                       m_pending_queries.cend(),
                       [](decltype(m_pending_queries)::value_type const &query) {
                           return query->is_consumed();
                       });
    return (first_consumed == m_pending_queries.cend());
}

PointCloudSource::PointCloudSource(const std::string &filepath)
    : m_point_cloud(filepath)
{

}

PointCloudSource::PointCloudSource(const point_cloud &pc) : m_point_cloud(pc) {}

void PointCloudSource::compute_queries()
{
    std::unique_lock queue_lock(m_pending_queries_mutex);

    // Wait for new queries and guard against spurious wakeups
    auto const was_in_time
        = m_queries_present.wait_for(queue_lock,
                                     std::chrono::seconds(1),
                                     [this] { return unprocessed_present(); });

    if (!was_in_time)
        // We have timed out while waiting
        return;

    /*
     * At this point we have aquired the lock for all pending queries and know
     * that new ones are present for processing
     */

    std::vector<vec3> points;
    // When no color is present in the data, use white points
    std::vector<rgb> colors
        = m_point_cloud.has_colors()
              ? std::vector<rgb>()
              : std::vector<rgb>(m_point_cloud.get_nr_points(), rgb(1.F));

    //TODO get a narrow pinhole camera for query and get points
    assert(std::numeric_limits<int>::max() >= m_point_cloud.get_nr_points());
    const int point_count = static_cast<int>(m_point_cloud.get_nr_points());

    for (int i = 0; i < point_count; ++i) {
        points.push_back(m_point_cloud.pnt(i));
    }

    if (m_point_cloud.has_colors())
        for (int i = 0; i < point_count; ++i) {
            colors.push_back(m_point_cloud.clr(i));
        }

    m_pending_queries.front()->supply_points(points, colors);
    m_pending_queries.front()->trigger_completion();
}

void PointCloudSource::remove_consumed_queries()
{
    std::unique_lock queue_lock(m_pending_queries_mutex);

    // Wait for new queries and guard against spurious wakeups
    auto const was_in_time
        = m_queries_present.wait_for(queue_lock,
                                     std::chrono::seconds(1),
                                     [this] { return unprocessed_present(); });

    if (!was_in_time)
        // Timeout
        return;

    m_pending_queries.remove_if(
        [](decltype(m_pending_queries)::value_type query) {
            return query->is_complete() && query->is_consumed();
        });
}

std::optional<std::shared_ptr<PointCloudQuery>>
PointCloudSource::get_finished_query()
{
    if (m_pending_queries.empty())
        return {};

    auto const first_complete
        = std::find_if(m_pending_queries.cbegin(),
                       m_pending_queries.cend(),
                       [](const decltype(m_pending_queries)::value_type &query) {
                           return query->is_complete() && !query->is_consumed();
                       });
    auto const ret = first_complete == m_pending_queries.cend()
                         ? decltype(get_finished_query())()
                         : decltype(get_finished_query())(*first_complete);

    return ret;
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

std::ostream &operator<<(std::ostream &os, const PointCloudSource &pcs)
{
    os << "\npending queries size: " << pcs.m_pending_queries.size()
       << "\nunprocessed queries present: " << pcs.unprocessed_present();
    return os;
}
