# UAV Planner Algorithm Explanation

This project can plan UAV motion in two different ways. The selected method comes from `config/planner.cfg`:

```text
planner_mode = point
```

or:

```text
planner_mode = kinodynamic
```

## Program flow

When the program starts, it follows this sequence:

1. Read `config/planner.cfg`.
2. Create the 3D world and inflate every obstacle by the safety margin.
3. Read the UAV start, goal, speed limit, acceleration limit, and planner mode.
4. Run either the point-robot planner or the kinodynamic planner.
5. Check the planned motion for collisions.
6. Save the path, trajectory, and obstacles as CSV files.
7. The Python scripts read those files and draw or animate the result.

## Point-robot planning

Point mode uses ordinary 3D A*. It answers: “Which grid cells connect the start to the goal without entering an obstacle?”

Each A* node contains only a 3D position. The planner checks all 26 neighboring cells, including diagonal neighbors. Moving diagonally costs more than moving along one axis. A Euclidean-distance estimate guides the search toward the goal.

After A* finds a path, line-of-sight checks remove unnecessary intermediate cells. A rest-to-rest speed profile is then applied to every straight path segment. This respects the configured velocity and acceleration limits, but the vehicle stops at sharp corners.

Point mode is simple and usually fast. However, position-only A* does not consider the UAV velocity while searching.

## Kinodynamic UAV planning

Kinodynamic mode searches with a more realistic UAV state:

```text
state = 3D position + 3D velocity
```

The planner starts with zero velocity. From every state, it tries acceleration commands in the X, Y, and Z directions. Each command is limited by `maximum_axis_acceleration`.

For one planning time step, the next state is calculated with the constant-acceleration kinematic equations:

```text
next_position = position + velocity * dt + 0.5 * acceleration * dt²
next_velocity = velocity + acceleration * dt
```

A candidate is rejected when:

- Its velocity exceeds `maximum_velocity`.
- Its curved motion passes through an inflated obstacle.
- It leaves the configured world bounds.
- The same state has already been reached with a lower cost.

The search finishes only when the UAV is close enough to the goal and its velocity is close to zero. Requiring low final velocity means the planner must leave enough space to brake instead of reaching the goal at full speed.

The final motion primitives are sampled at `trajectory_time_step`. This produces smooth position, velocity, acceleration, yaw, and pitch values for animation.

## Collision safety

Obstacles are axis-aligned 3D boxes. Before checking a position, the planner expands every box by `safety_margin`. This approximates the physical size of the UAV and keeps its center away from walls.

Point mode samples every straight movement. Kinodynamic mode samples its parabolic constant-acceleration motion. The distance between collision samples is controlled by `collision_check_step`.

Smaller collision steps provide finer checking but require more computation.

## Important configuration values

- `planner_mode`: selects `point` or `kinodynamic`.
- `world_min`, `world_max`: define the planning volume.
- `start`, `goal`: define the requested 3D motion.
- `grid_resolution`: position-grid spacing.
- `velocity_resolution`: velocity-grid spacing for kinodynamic A*.
- `safety_margin`: obstacle inflation around the UAV.
- `collision_check_step`: spacing used during swept collision checks.
- `maximum_velocity`: maximum total UAV speed.
- `maximum_axis_acceleration`: acceleration limit for each control axis.
- `planning_time_step`: duration of one kinodynamic motion primitive.
- `trajectory_time_step`: sampling period used in the exported trajectory.
- `goal_position_tolerance`: acceptable final position error.
- `goal_velocity_tolerance`: acceptable final speed.
- `heuristic_weight`: how strongly kinodynamic A* prefers states near the goal.
- `maximum_expansions`: safety limit on kinodynamic search work.

## Output files

- `astar_path.csv` contains the discrete planned positions.
- `smoothed_path.csv` contains the point-mode shortcut path. In kinodynamic mode it contains the dynamic state positions.
- `trajectory.csv` contains time, position, velocity, acceleration, yaw, and pitch.
- `obstacles.csv` contains the obstacle boxes used by the visualizer.

The static visualizer draws all paths and obstacles. The animation displays a quadcopter model following the generated trajectory and rotating with the planned yaw and pitch.
