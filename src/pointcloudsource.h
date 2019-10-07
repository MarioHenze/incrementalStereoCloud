#pragma once

#include <chrono>
#include <condition_variable>
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

    /**
     * @brief m_pending_queries_mutex protects the query queue against
     * concurrent access
     */
    std::timed_mutex m_pending_queries_mutex;

    /**
     * @brief m_queries_present provides a signal for efficiently waiting for
     * new queries
     */
    std::condition_variable_any m_queries_present;

    void update_query(std::shared_ptr<PointCloudQuery> &pcq);

public:
    PointCloudSource(std::string const &filepath);

    /**
     * @brief compute_queries resolves all pending queries
     *
     * This function is intended to be called by another thread which resolves
     * all pending queries.
     */
    void compute_queries();

    std::shared_ptr<PointCloudQuery> queryPoints(
        std::chrono::microseconds const time_budget = std::chrono::microseconds(
            1000));
};
