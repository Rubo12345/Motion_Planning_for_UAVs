#pragma once

#include "planner_helpers.h"

#include <vector>

namespace uav
{

class AStar3D
{
public:
    explicit AStar3D(World world);
    std::vector<Vec3> plan(const Vec3& start, const Vec3& goal) const;
    std::vector<Vec3> simplify(const std::vector<Vec3>& path) const;
    bool collisionFree(const Vec3& from, const Vec3& to) const;

private:
    World world_;
    bool occupied(const Vec3& point) const;
};

class PointMassTrajectory
{
public:
    PointMassTrajectory(double max_velocity, double max_acceleration, double dt);
    std::vector<TrajectoryPoint> generate(const std::vector<Vec3>& path) const;

private:
    double max_velocity_;
    double max_acceleration_;
    double dt_;
};

}  // namespace uav
