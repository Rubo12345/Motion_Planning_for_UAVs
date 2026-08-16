# 3D Motion Planning for UAVs

A compact C++17 UAV motion-planning demo with a Python 3D visualizer. It supports geometric 26-connected A* and velocity-state kinodynamic A* with bounded 3D acceleration. Axis-aligned obstacles are inflated by the configured UAV safety margin.

Planner settings are stored in `config/planner.cfg`. Set `planner_mode` to `point` or `kinodynamic`. See [algorithm_explanation.md](algorithm_explanation.md) for a simple walkthrough of the complete planning flow.

## Pipeline

1. Load the world, start/goal, obstacles, planner mode, and vehicle limits from configuration.
2. Plan with geometric 3D A* or position-velocity kinodynamic A*.
3. Inflate obstacles and perform swept collision checks.
4. Export path, position, velocity, acceleration, yaw, and pitch to CSV.

The example environment, start/goal, grid resolution, safety margin, planner mode, and kinematic limits can be changed without rebuilding by editing `config/planner.cfg`.

## Build and run

Requirements: CMake 3.16+, a C++17 compiler, Python 3, NumPy, and Matplotlib.

```bash
python -m pip install -r requirements.txt
cmake -S . -B build
cmake --build build --config Release
```

Run on Windows (Visual Studio generator):

```powershell
.\build\Release\uav_planner_demo.exe
python .\visualization\plot_trajectory.py
python .\visualization\animate_trajectory.py
```

For a single-command Windows demo, run `run_demo.bat`. With a single-config generator, the executable is `./build/uav_planner_demo`.

Run tests:

```bash
ctest --test-dir build -C Release --output-on-failure
```

## Outputs

- `output/astar_path.csv`: raw voxel path
- `output/smoothed_path.csv`: collision-free shortcut path
- `output/trajectory.csv`: time, position, velocity, acceleration, yaw, and pitch
- `output/obstacles.csv`: obstacle geometry used by the visualizer
