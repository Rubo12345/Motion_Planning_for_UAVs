#include "csv_writer.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace uav
{
namespace
{
std::ofstream createOutputFile(const std::string& filename)
{
    const std::filesystem::path path(filename);
    if (!path.parent_path().empty())
    {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output(filename);
    if (!output)
    {
        throw std::runtime_error("cannot write " + filename);
    }
    return output;
}
}  // namespace

void writePathCsv(const std::string& filename, const std::vector<Vec3>& path)
{
    std::ofstream output = createOutputFile(filename);
    output << "x,y,z\n";

    for (const auto& point : path)
    {
        output << point.x << ',' << point.y << ',' << point.z << '\n';
    }
}

void writeTrajectoryCsv(const std::string& filename,
                        const std::vector<TrajectoryPoint>& trajectory)
{
    std::ofstream output = createOutputFile(filename);
    output << "time,x,y,z,vx,vy,vz,ax,ay,az,yaw,pitch\n";

    for (const auto& state : trajectory)
    {
        output << state.time << ',' << state.position.x << ',' << state.position.y << ','
               << state.position.z << ',' << state.velocity.x << ',' << state.velocity.y << ','
               << state.velocity.z << ',' << state.acceleration.x << ',' << state.acceleration.y
               << ',' << state.acceleration.z << ',' << state.yaw << ',' << state.pitch << '\n';
    }
}

void writeObstaclesCsv(const std::string& filename, const World& world)
{
    std::ofstream output = createOutputFile(filename);
    output << "min_x,min_y,min_z,max_x,max_y,max_z\n";

    for (const auto& obstacle : world.obstacles)
    {
        output << obstacle.min.x << ',' << obstacle.min.y << ',' << obstacle.min.z << ','
               << obstacle.max.x << ',' << obstacle.max.y << ',' << obstacle.max.z << '\n';
    }
}

}  // namespace uav
