#pragma once

#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <queue>

#include <libs/point_cloud/point_cloud.h>

#include "pointcloudquery.h"

class PointCloudSource
{
private:
    /**
     * @brief m_point_cloud holds an cgv point cloud
     *
     * This object is used to load points from an anbitrary point cloud file
     */
    point_cloud m_point_cloud;

    /**
     * @brief m_pending_queries Holds all queries which were made
     */
    std::list<std::shared_ptr<PointCloudQuery>> m_pending_queries;

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

    /**
     * @brief m_destructor_called signals that threads should abort operations
     */
    std::atomic<bool> m_destructor_called {false};

    /**
     * @brief unprocessed_present determines if unprocessed queries are in the
     * list of all queries.
     * @param query the query
     */
    [[nodiscard]] bool unprocessed_present() const;

public:
    PointCloudSource(std::string const &filepath);
    ~PointCloudSource();

    /**
     * @brief compute_queries resolves all pending queries
     *
     * This function is intended to be called by another thread which resolves
     * all pending queries.
     */
    void compute_queries();

    /**
     * @brief get_finished_query return a finised point cloud query if there is
     * one
     * @return a resolved point cloud query
     */
    std::optional<std::shared_ptr<PointCloudQuery>> get_finished_query();

    void queryPoints(
        std::chrono::microseconds const time_budget = std::chrono::microseconds(
            1000));
};
