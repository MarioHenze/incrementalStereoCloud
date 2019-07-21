#pragma once

#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <libs/point_cloud/point_cloud.h>

#include "pointcloudquery.h"

class PointCloudSource
{
private:
    point_cloud m_point_cloud;

    /**
     * @brief m_pending_queries Holds all queries which were made
     */
    std::queue<std::shared_ptr<PointCloudQuery>> m_pending_queries;
    std::timed_mutex m_pending_queries_mutex;

    std::list<std::shared_ptr<PointCloudQuery>> m_active_queries;

    void update_query(std::shared_ptr<PointCloudQuery> & pcq);

public:
    PointCloudSource(std::string const & filepath);

    std::shared_ptr<PointCloudQuery> queryPoints(
            std::chrono::microseconds const time_budget =
                std::chrono::microseconds(1000));
};
