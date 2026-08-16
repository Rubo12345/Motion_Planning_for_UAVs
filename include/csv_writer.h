#pragma once

#include "planner_helpers.h"

#include <string>
#include <vector>

namespace uav
{

void writePathCsv(const std::string& filename, const std::vector<Vec3>& path);

void writeTrajectoryCsv(const std::string& filename,
                        const std::vector<TrajectoryPoint>& trajectory);

void writeObstaclesCsv(const std::string& filename, const World& world);

}  // namespace uav
