@echo off
setlocal
cmake -S . -B build || exit /b 1
cmake --build build --config Release || exit /b 1
if exist build\Release\uav_planner_demo.exe (build\Release\uav_planner_demo.exe) else (build\uav_planner_demo.exe)
if errorlevel 1 exit /b 1
python visualization\animate_trajectory.py
