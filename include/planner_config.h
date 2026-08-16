#pragma once

#include "planner_helpers.h"

#include <cstddef>
#include <string>

namespace uav
{

enum class PlannerMode
{
    PointRobot,
    Kinodynamic
};

struct PlannerConfig
{
    PlannerMode planner_mode{PlannerMode::Kinodynamic};
    World world;
    Vec3 start;
    Vec3 goal;
    double maximum_velocity{3.0};
    double maximum_axis_acceleration{1.0};
    double planning_time_step{1.0};
    double trajectory_time_step{0.05};
    double velocity_resolution{1.0};
    double goal_position_tolerance{0.75};
    double goal_velocity_tolerance{0.01};
    double heuristic_weight{1.5};
    std::size_t maximum_expansions{500000};
    std::string output_directory{"output"};
};

PlannerConfig loadPlannerConfig(const std::string& filename);
std::string plannerModeName(PlannerMode mode);

}  // namespace uav
