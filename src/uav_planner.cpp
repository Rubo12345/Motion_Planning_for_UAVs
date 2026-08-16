#include "uav_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace uav
{

AStar3D::AStar3D(World world) : world_(std::move(world))
{
    if (world_.resolution <= 0.0)
    {
        throw std::invalid_argument("resolution must be positive");
    }
}

bool AStar3D::occupied(const Vec3& p) const
{
    if (p.x < world_.min.x || p.y < world_.min.y || p.z < world_.min.z ||
        p.x > world_.max.x || p.y > world_.max.y || p.z > world_.max.z)
    {
        return true;
    }
    for (const auto& b : world_.obstacles)
    {
        const double m = world_.safety_margin;
        if (p.x >= b.min.x-m && p.x <= b.max.x+m && p.y >= b.min.y-m &&
            p.y <= b.max.y+m && p.z >= b.min.z-m && p.z <= b.max.z+m)
        {
            return true;
        }
    }
    return false;
}

std::vector<Vec3> AStar3D::plan(const Vec3& start, const Vec3& goal) const
{
    const auto toCell = [&](const Vec3& p)
    {
        return Cell{
            static_cast<int>(std::lround((p.x-world_.min.x)/world_.resolution)),
            static_cast<int>(std::lround((p.y-world_.min.y)/world_.resolution)),
            static_cast<int>(std::lround((p.z-world_.min.z)/world_.resolution))};
    };
    const auto toPoint = [&](const Cell& c)
    {
        return Vec3{world_.min.x+c.x*world_.resolution,
                    world_.min.y+c.y*world_.resolution,
                    world_.min.z+c.z*world_.resolution};
    };
    if (occupied(start) || occupied(goal))
    {
        throw std::runtime_error("start or goal is occupied/outside world");
    }
    const Cell s = toCell(start), g = toCell(goal);
    if (occupied(toPoint(s)) || occupied(toPoint(g)))
    {
        throw std::runtime_error("nearest start or goal voxel is occupied");
    }
    std::priority_queue<QueueNode> open;
    std::unordered_map<Cell,double,CellHash> cost;
    std::unordered_map<Cell,Cell,CellHash> parent;
    cost[s] = 0.0; open.push({norm(toPoint(g)-toPoint(s)), s});
    while (!open.empty())
    {
        const Cell current = open.top().cell; open.pop();
        if (current == g)
        {
            std::vector<Vec3> result{goal};
            for (Cell c = g; !(c == s); c = parent.at(c))
            {
                result.push_back(toPoint(parent.at(c)));
            }
            std::reverse(result.begin(), result.end()); result.front() = start; result.back() = goal;
            return result;
        }
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    Cell next{current.x + dx, current.y + dy, current.z + dz};
                    const Vec3 np = toPoint(next);

                    if (occupied(np) || !collisionFree(toPoint(current), np))
                    {
                        continue;
                    }

                    const double candidate = cost[current] +
                        world_.resolution * std::sqrt(double(dx * dx + dy * dy + dz * dz));
                    auto it = cost.find(next);

                    if (it == cost.end() || candidate < it->second)
                    {
                        cost[next] = candidate;
                        parent[next] = current;
                        open.push({candidate + norm(toPoint(g) - np), next});
                    }
                }
            }
        }
    }
    throw std::runtime_error("no collision-free path found");
}

bool AStar3D::collisionFree(const Vec3& a, const Vec3& b) const
{
    const double distance = norm(b-a);
    const int samples =
        std::max(1, static_cast<int>(std::ceil(distance / world_.collision_check_step)));
    for (int i = 0; i <= samples; ++i)
    {
        if (occupied(a + (b - a) * (double(i) / samples)))
        {
            return false;
        }
    }
    return true;
}

std::vector<Vec3> AStar3D::simplify(const std::vector<Vec3>& path) const
{
    if (path.size() < 3)
    {
        return path;
    }
    std::vector<Vec3> result{path.front()};
    std::size_t anchor=0;
    while (anchor+1 < path.size())
    {
        std::size_t next=path.size()-1;
        while (next > anchor + 1 && !collisionFree(path[anchor], path[next]))
        {
            --next;
        }
        result.push_back(path[next]); anchor=next;
    }
    return result;
}

PointMassTrajectory::PointMassTrajectory(double vmax, double amax, double dt)
    : max_velocity_(vmax), max_acceleration_(amax), dt_(dt)
{
    if (vmax <= 0 || amax <= 0 || dt <= 0)
    {
        throw std::invalid_argument("kinematic limits and dt must be positive");
    }
}

std::vector<TrajectoryPoint> PointMassTrajectory::generate(const std::vector<Vec3>& path) const
{
    if (path.size() < 2)
    {
        throw std::invalid_argument("path needs at least two points");
    }
    std::vector<TrajectoryPoint> out;
    double time_offset=0.0;
    for (std::size_t i=1;i<path.size();++i)
    {
        const Vec3 delta=path[i]-path[i-1];
        const double length=norm(delta);
        if (length < 1e-9)
        {
            continue;
        }
        const Vec3 direction=unit(delta);
        const double ramp=max_velocity_*max_velocity_/(2.0*max_acceleration_);
        const bool triangular=2.0*ramp>=length;
        double peak;
        if (triangular)
        {
            peak = std::sqrt(length * max_acceleration_);
        }
        else
        {
            peak = max_velocity_;
        }
        const double t_acc=peak/max_acceleration_;
        double cruise;
        if (triangular)
        {
            cruise = 0.0;
        }
        else
        {
            cruise = (length - 2.0 * ramp) / peak;
        }
        const double total=2.0*t_acc+cruise;
        auto append=[&](double t)
        {
            double s,v,a;
            if(t<t_acc)
            {
                a=max_acceleration_;v=a*t;s=0.5*a*t*t;
            }
            else if(t<t_acc+cruise)
            {
                a=0;v=peak;s=0.5*peak*t_acc+peak*(t-t_acc);
            }
            else
            {
                const double rem=total-t;a=-max_acceleration_;v=max_acceleration_*std::max(0.0,rem);s=length-0.5*max_acceleration_*rem*rem;
            }
            s=std::clamp(s,0.0,length);
            out.push_back({time_offset+t,path[i-1]+direction*s,direction*v,direction*a,
                std::atan2(direction.y,direction.x),std::atan2(direction.z,std::hypot(direction.x,direction.y))});
        };
        double first_sample_time;
        if (out.empty())
        {
            first_sample_time = 0.0;
        }
        else
        {
            first_sample_time = dt_;
        }

        for (double t = first_sample_time; t < total; t += dt_)
        {
            append(t);
        }
        append(total);
        out.back().position=path[i]; out.back().velocity={}; out.back().acceleration={};
        time_offset+=total;
    }
    if (out.empty())
    {
        throw std::invalid_argument("path has zero length");
    }
    return out;
}

} // namespace uav
