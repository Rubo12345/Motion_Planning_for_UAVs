#include "uav_planner.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    const uav::World world{
        {0, 0, 0}, {10, 10, 6}, 0.5, 0.2, 0.125, {{{4, 0, 0}, {6, 8, 5}}}};
    const uav::Vec3 start{1, 1, 1};
    const uav::Vec3 goal{9, 9, 5.5};
    const uav::AStar3D planner(world);
    const auto raw = planner.plan(start, goal);
    const auto path = planner.simplify(raw);

    assert(!raw.empty() && path.size() >= 2);
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        assert(planner.collisionFree(path[i - 1], path[i]));
    }

    const auto trajectory = uav::PointMassTrajectory(2.0, 1.0, 0.05).generate(path);
    assert(!trajectory.empty());
    assert(std::abs(trajectory.back().position.x - goal.x) < 1e-9);
    assert(std::abs(trajectory.back().position.z - goal.z) < 1e-9);

    for (const auto& point : trajectory)
    {
        const double velocity =
            std::sqrt(point.velocity.x * point.velocity.x + point.velocity.y * point.velocity.y +
                      point.velocity.z * point.velocity.z);
        const double acceleration = std::sqrt(
            point.acceleration.x * point.acceleration.x +
            point.acceleration.y * point.acceleration.y +
            point.acceleration.z * point.acceleration.z);

        assert(velocity <= 2.0 + 1e-9);
        assert(acceleration <= 1.0 + 1e-9);
    }

    std::cout << "All planner tests passed\n";
}
