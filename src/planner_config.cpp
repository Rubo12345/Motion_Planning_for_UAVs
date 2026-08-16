#include "planner_config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace uav
{
namespace
{
std::string trim(const std::string& text)
{
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::vector<double> numberList(const std::string& value, std::size_t expected,
                               const std::string& key)
{
    std::vector<double> numbers;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ','))
    {
        numbers.push_back(std::stod(trim(token)));
    }
    if (numbers.size() != expected)
    {
        throw std::runtime_error(key + " requires " + std::to_string(expected) + " values");
    }
    return numbers;
}

Vec3 vectorValue(const std::string& value, const std::string& key)
{
    const auto values = numberList(value, 3, key);
    return {values[0], values[1], values[2]};
}
}  // namespace

PlannerConfig loadPlannerConfig(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input)
    {
        throw std::runtime_error("cannot open planner config: " + filename);
    }

    PlannerConfig config;
    std::unordered_map<std::string, std::string> values;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty())
        {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos)
        {
            throw std::runtime_error("invalid config line " + std::to_string(line_number));
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key == "obstacle")
        {
            const auto box = numberList(value, 6, key);
            config.world.obstacles.push_back(
                {{box[0], box[1], box[2]}, {box[3], box[4], box[5]}});
        }
        else
        {
            values[key] = value;
        }
    }

    const auto required = [&](const std::string& key) -> const std::string&
    {
        const auto entry = values.find(key);
        if (entry == values.end())
        {
            throw std::runtime_error("missing config value: " + key);
        }
        return entry->second;
    };
    const auto scalar = [&](const std::string& key)
    {
        return std::stod(required(key));
    };

    std::string mode = required("planner_mode");
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char character) { return std::tolower(character); });
    if (mode == "point")
    {
        config.planner_mode = PlannerMode::PointRobot;
    }
    else if (mode == "kinodynamic")
    {
        config.planner_mode = PlannerMode::Kinodynamic;
    }
    else
    {
        throw std::runtime_error("planner_mode must be point or kinodynamic");
    }

    config.output_directory = required("output_directory");
    config.world.min = vectorValue(required("world_min"), "world_min");
    config.world.max = vectorValue(required("world_max"), "world_max");
    config.start = vectorValue(required("start"), "start");
    config.goal = vectorValue(required("goal"), "goal");
    config.world.resolution = scalar("grid_resolution");
    config.world.safety_margin = scalar("safety_margin");
    config.world.collision_check_step = scalar("collision_check_step");
    config.maximum_velocity = scalar("maximum_velocity");
    config.maximum_axis_acceleration = scalar("maximum_axis_acceleration");
    config.planning_time_step = scalar("planning_time_step");
    config.trajectory_time_step = scalar("trajectory_time_step");
    config.velocity_resolution = scalar("velocity_resolution");
    config.goal_position_tolerance = scalar("goal_position_tolerance");
    config.goal_velocity_tolerance = scalar("goal_velocity_tolerance");
    config.heuristic_weight = scalar("heuristic_weight");
    config.maximum_expansions = std::stoull(required("maximum_expansions"));

    if (config.world.resolution <= 0.0 || config.world.collision_check_step <= 0.0 ||
        config.maximum_velocity <= 0.0 || config.maximum_axis_acceleration <= 0.0 ||
        config.planning_time_step <= 0.0 || config.trajectory_time_step <= 0.0 ||
        config.velocity_resolution <= 0.0 || config.maximum_expansions == 0)
    {
        throw std::runtime_error("all resolutions, limits, time steps, and counts must be positive");
    }
    return config;
}

std::string plannerModeName(PlannerMode mode)
{
    if (mode == PlannerMode::PointRobot)
    {
        return "point";
    }
    else
    {
        return "kinodynamic";
    }
}

}  // namespace uav
