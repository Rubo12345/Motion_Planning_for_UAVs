#include "kinodynamic_planner.h"
#include "uav_planner.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace uav
{

KinodynamicAStar3D::KinodynamicAStar3D(PlannerConfig config) : config_(std::move(config))
{
}

std::vector<TrajectoryPoint> KinodynamicAStar3D::plan() const
{
    const World& world = config_.world;
    const double position_resolution = world.resolution;
    const double velocity_resolution = config_.velocity_resolution;
    const double time_step = config_.planning_time_step;
    const AStar3D collision_checker(world);

    const auto positionCell = [&](const Vec3& position)
    {
        const int x =
            static_cast<int>(std::lround((position.x - world.min.x) / position_resolution));
        const int y =
            static_cast<int>(std::lround((position.y - world.min.y) / position_resolution));
        const int z =
            static_cast<int>(std::lround((position.z - world.min.z) / position_resolution));

        return Cell{x, y, z};
    };

    const auto positionValue = [&](const Cell& cell)
    {
        const double x = world.min.x + cell.x * position_resolution;
        const double y = world.min.y + cell.y * position_resolution;
        const double z = world.min.z + cell.z * position_resolution;

        return Vec3{x, y, z};
    };

    const auto velocityCell = [&](const Vec3& velocity)
    {
        const int x = static_cast<int>(std::lround(velocity.x / velocity_resolution));
        const int y = static_cast<int>(std::lround(velocity.y / velocity_resolution));
        const int z = static_cast<int>(std::lround(velocity.z / velocity_resolution));

        return Cell{x, y, z};
    };

    const auto velocityValue = [&](const Cell& cell)
    {
        const double x = cell.x * velocity_resolution;
        const double y = cell.y * velocity_resolution;
        const double z = cell.z * velocity_resolution;

        return Vec3{x, y, z};
    };

    if (!collision_checker.collisionFree(config_.start, config_.start) ||
        !collision_checker.collisionFree(config_.goal, config_.goal))
    {
        throw std::runtime_error("start or goal is occupied/outside world");
    }

    const Cell start_position = positionCell(config_.start);
    const Cell start_velocity = velocityCell({0.0, 0.0, 0.0});
    const DynamicState start{start_position, start_velocity};

    std::priority_queue<OpenNode> open;
    std::unordered_map<DynamicState, double, DynamicStateHash> cost;
    std::unordered_map<DynamicState, ParentRecord, DynamicStateHash> parent;

    const auto heuristic = [&](const DynamicState& state)
    {
        const Vec3 position = positionValue(state.position);
        const double distance_to_goal = norm(config_.goal - position);

        return distance_to_goal / config_.maximum_velocity;
    };

    const auto isGoal = [&](const DynamicState& state)
    {
        const double position_error = norm(config_.goal - positionValue(state.position));
        const double velocity_error = norm(velocityValue(state.velocity));

        return position_error <= config_.goal_position_tolerance &&
               velocity_error <= config_.goal_velocity_tolerance;
    };

    cost[start] = 0.0;
    open.push({config_.heuristic_weight * heuristic(start), start});
    std::size_t expansions = 0;
    DynamicState final_state{};
    bool found = false;

    while (!open.empty() && expansions < config_.maximum_expansions)
    {
        const DynamicState current = open.top().state;
        open.pop();
        ++expansions;

        if (isGoal(current))
        {
            final_state = current;
            found = true;
            break;
        }

        const Vec3 current_position = positionValue(current.position);
        const Vec3 current_velocity = velocityValue(current.velocity);
        for (int ax = -1; ax <= 1; ++ax)
        {
            for (int ay = -1; ay <= 1; ++ay)
            {
                for (int az = -1; az <= 1; ++az)
                {
                    const Vec3 acceleration{ax * config_.maximum_axis_acceleration,
                                            ay * config_.maximum_axis_acceleration,
                                            az * config_.maximum_axis_acceleration};

                    const Vec3 integrated_velocity =
                        current_velocity + acceleration * time_step;
                    const Cell next_velocity_cell = velocityCell(integrated_velocity);
                    const Vec3 next_velocity = velocityValue(next_velocity_cell);

                    const double velocity_limit =
                        config_.maximum_velocity + velocity_resolution * 0.5;
                    if (norm(next_velocity) > velocity_limit)
                    {
                        continue;
                    }

                    const Vec3 integrated_position =
                        current_position + current_velocity * time_step +
                        acceleration * (0.5 * time_step * time_step);
                    const Cell next_position_cell = positionCell(integrated_position);
                    const Vec3 next_position = positionValue(next_position_cell);

                    const double motion_length =
                        norm(current_velocity) * time_step +
                        0.5 * norm(acceleration) * time_step * time_step;
                    const int collision_samples = std::max(
                        1, static_cast<int>(std::ceil(
                               motion_length / config_.world.collision_check_step)));

                    bool motion_is_safe = true;
                    Vec3 previous_position = current_position;

                    for (int sample = 1; sample <= collision_samples; ++sample)
                    {
                        const double sample_time =
                            time_step * static_cast<double>(sample) / collision_samples;
                        const Vec3 sample_position =
                            current_position + current_velocity * sample_time +
                            acceleration * (0.5 * sample_time * sample_time);
                        if (!collision_checker.collisionFree(previous_position, sample_position))
                        {
                            motion_is_safe = false;
                            break;
                        }

                        previous_position = sample_position;
                    }

                    if (!motion_is_safe ||
                        !collision_checker.collisionFree(previous_position, next_position))
                    {
                        continue;
                    }

                    const DynamicState next{next_position_cell, next_velocity_cell};
                    if (next == current)
                    {
                        continue;
                    }
                    const double candidate = cost.at(current) + time_step;
                    const auto known = cost.find(next);

                    if (known == cost.end() || candidate < known->second)
                    {
                        cost[next] = candidate;
                        parent[next] = {current, acceleration};
                        open.push({candidate + config_.heuristic_weight * heuristic(next), next});
                    }
                }
            }
        }
    }

    if (!found)
    {
        throw std::runtime_error("kinodynamic A* found no solution before maximum_expansions");
    }

    std::vector<DynamicState> states{final_state};
    std::vector<Vec3> controls;
    for (DynamicState state = final_state; !(state == start); state = parent.at(state).parent)
    {
        controls.push_back(parent.at(state).acceleration);
        states.push_back(parent.at(state).parent);
    }
    std::reverse(states.begin(), states.end());
    std::reverse(controls.begin(), controls.end());

    std::vector<TrajectoryPoint> trajectory;
    const auto appendState = [&](double time,
                                 const Vec3& position,
                                 const Vec3& velocity,
                                 const Vec3& acceleration)
    {
        const double horizontal_speed = std::hypot(velocity.x, velocity.y);
        double yaw;
        if (horizontal_speed > config_.goal_velocity_tolerance)
        {
            yaw = std::atan2(velocity.y, velocity.x);
        }
        else if (!trajectory.empty())
        {
            yaw = trajectory.back().yaw;
        }
        else
        {
            yaw = 0.0;
        }

        double pitch;
        if (norm(velocity) > config_.goal_velocity_tolerance)
        {
            pitch = std::atan2(velocity.z, horizontal_speed);
        }
        else
        {
            pitch = 0.0;
        }

        trajectory.push_back({time, position, velocity, acceleration, yaw, pitch});
    };

    for (std::size_t index = 0; index < controls.size(); ++index)
    {
        Vec3 segment_position;
        if (index == 0)
        {
            segment_position = config_.start;
        }
        else
        {
            segment_position = positionValue(states[index].position);
        }

        const Vec3 segment_velocity = velocityValue(states[index].velocity);
        const Vec3 acceleration = controls[index];
        double first_sample;
        if (trajectory.empty())
        {
            first_sample = 0.0;
        }
        else
        {
            first_sample = config_.trajectory_time_step;
        }

        for (double local_time = first_sample;
             local_time < time_step;
             local_time += config_.trajectory_time_step)
        {
            const Vec3 position = segment_position + segment_velocity * local_time +
                                  acceleration * (0.5 * local_time * local_time);
            const Vec3 velocity = segment_velocity + acceleration * local_time;
            appendState(index * time_step + local_time, position, velocity, acceleration);
        }
    }

    const double final_time = controls.size() * time_step;
    const Vec3 final_position = positionValue(states.back().position);
    const Vec3 final_velocity = velocityValue(states.back().velocity);
    appendState(final_time, final_position, final_velocity, {});

    return trajectory;
}

}  // namespace uav
