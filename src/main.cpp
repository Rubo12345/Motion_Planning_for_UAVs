#include "csv_writer.h"
#include "kinodynamic_planner.h"
#include "planner_config.h"
#include "uav_planner.h"

#include <chrono>
#include <iostream>

int main(int argc, char** argv)
{
    std::string config_file;
    if (argc > 1)
    {
        config_file = argv[1];
    }
    else
    {
        config_file = "config/planner.cfg";
    }

    try
    {
        const auto begin = std::chrono::steady_clock::now();
        const uav::PlannerConfig config = uav::loadPlannerConfig(config_file);
        std::vector<uav::Vec3> raw_path;
        std::vector<uav::Vec3> planned_path;
        std::vector<uav::TrajectoryPoint> trajectory;

        if (config.planner_mode == uav::PlannerMode::PointRobot)
        {
            const uav::AStar3D planner(config.world);
            raw_path = planner.plan(config.start, config.goal);
            planned_path = planner.simplify(raw_path);
            const uav::PointMassTrajectory model(config.maximum_velocity,
                                                 config.maximum_axis_acceleration,
                                                 config.trajectory_time_step);
            trajectory = model.generate(planned_path);
        }
        else
        {
            const uav::KinodynamicAStar3D planner(config);
            trajectory = planner.plan();
            for (const auto& state : trajectory)
            {
                raw_path.push_back(state.position);
            }
            planned_path = raw_path;
        }

        const std::string& output = config.output_directory;
        uav::writePathCsv(output + "/astar_path.csv", raw_path);
        uav::writePathCsv(output + "/smoothed_path.csv", planned_path);
        uav::writeTrajectoryCsv(output + "/trajectory.csv", trajectory);
        uav::writeObstaclesCsv(output + "/obstacles.csv", config.world);

        const auto elapsed = std::chrono::steady_clock::now() - begin;
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        std::cout << "Planner mode: " << uav::plannerModeName(config.planner_mode) << '\n'
                  << "Planned states: " << raw_path.size() << '\n'
                  << "Trajectory: " << trajectory.size() << " samples, "
                  << trajectory.back().time << " s\n"
                  << "Planning and export: " << milliseconds << " ms\n"
                  << "Outputs written to " << output << '\n';
    }
    catch (const std::exception& error)
    {
        std::cerr << "Planner error: " << error.what() << '\n';
        return 1;
    }
}
