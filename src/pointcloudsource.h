#pragma once

#include <chrono>
#include <condition_variable>
#include <iostream>
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
     * @brief unprocessed_present determines if unprocessed queries are in the
     * list of all queries.
     * @return true if unprocessed queries are present, false otherwise
     */
    [[nodiscard]] bool unprocessed_present() const;

    /**
     * @brief consumed_present determines if consumed queries are in the list of
     * all queries
     * @return true if consumed queries are present, false otherwise
     */
    [[nodiscard]] bool consumed_present() const;

public:
    PointCloudSource(std::string const &filepath);
    PointCloudSource(point_cloud const &pc);

    /**
     * @brief compute_queries resolves all pending queries
     *
     * This function is intended to be called by another thread which resolves
     * all pending queries.
     */
    void compute_queries();

    /**
     * @brief remove_consumed_queries deletes all consumed queries
     *
     * This function is intended to be called periodically to remove consumed
     * queries.
     */
    void remove_consumed_queries();

    /**
     * @brief get_finished_query return a finised point cloud query if there is
     * one
     * @return a resolved point cloud query
     */
    std::optional<std::shared_ptr<PointCloudQuery>> get_finished_query();

    void queryPoints(
        std::chrono::microseconds const time_budget = std::chrono::microseconds(
            1000));

    friend std::ostream &operator<<(std::ostream &os,
                                    PointCloudSource const &pcs);
};
