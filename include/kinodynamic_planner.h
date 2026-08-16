#pragma once

#include "planner_config.h"

#include <vector>

namespace uav
{

class KinodynamicAStar3D
{
public:
    explicit KinodynamicAStar3D(PlannerConfig config);
    std::vector<TrajectoryPoint> plan() const;

private:
    PlannerConfig config_;
};

}  // namespace uav
