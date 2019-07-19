#pragma once

#include <string>

#include "pointcloudquery.h"

class PointCloudSource
{
private:
    void update_query(PointCloudQuery & pcq);

public:
    PointCloudSource(std::string const & filepath);

    PointCloudQuery queryPoints() const;
};
