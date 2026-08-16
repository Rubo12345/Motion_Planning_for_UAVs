#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace uav
{

struct Vec3
{
    double x{};
    double y{};
    double z{};

    Vec3 operator+(const Vec3& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    Vec3 operator-(const Vec3& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    Vec3 operator*(double scale) const
    {
        return {x * scale, y * scale, z * scale};
    }
};

struct Box
{
    Vec3 min;
    Vec3 max;
};

struct World
{
    Vec3 min;
    Vec3 max;
    double resolution{0.5};
    double safety_margin{0.35};
    double collision_check_step{0.125};
    std::vector<Box> obstacles;
};

struct TrajectoryPoint
{
    double time{};
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    double yaw{};
    double pitch{};
};

inline double dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double norm(const Vec3& v)
{
    return std::sqrt(dot(v, v));
}

inline Vec3 unit(const Vec3& v)
{
    const double n = norm(v);
    if (n > 1e-12)
    {
        return v * (1.0 / n);
    }
    else
    {
        return Vec3{};
    }
}

struct Cell
{
    int x;
    int y;
    int z;

    bool operator==(const Cell& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellHash
{
    std::size_t operator()(const Cell& cell) const
    {
        return (static_cast<std::size_t>(cell.x) * 73856093u) ^
               (static_cast<std::size_t>(cell.y) * 19349663u) ^
               (static_cast<std::size_t>(cell.z) * 83492791u);
    }
};

struct QueueNode
{
    double f;
    Cell cell;

    bool operator<(const QueueNode& other) const
    {
        return f > other.f;
    }
};

struct DynamicState
{
    Cell position;
    Cell velocity;

    bool operator==(const DynamicState& other) const
    {
        return position == other.position && velocity == other.velocity;
    }
};

struct DynamicStateHash
{
    std::size_t operator()(const DynamicState& state) const
    {
        const CellHash hash;
        return hash(state.position) ^ (hash(state.velocity) << 1u);
    }
};

struct OpenNode
{
    double priority;
    DynamicState state;

    bool operator<(const OpenNode& other) const
    {
        return priority > other.priority;
    }
};

struct ParentRecord
{
    DynamicState parent;
    Vec3 acceleration;
};

}  // namespace uav
