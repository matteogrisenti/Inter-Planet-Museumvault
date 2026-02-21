# Part 2.5: PlanSys2 Integration

## Overview
This directory contains the implementation for **Problem 5**. The goal here is to transport the temporal durative planning model into a concrete robotics execution framework using **ROS2 PlanSys2**.

We map our PDDL actions into PlanSys2 action nodes (often utilizing "fake" nodes during simulation) to observe how the high-level planner orchestrates tasks in real-time, respecting action durations.

> [!WARNING]
> **Work in Progress**: This section utilizes the PlanSys2 architecture which operates differently from standard planners. The code and configuration are currently undergoing active modification by the user to properly integrate parallel continuous execution.

## 🚀 Execution Commands

### Prerequisites & Setup
Navigate to your PlanSys2 workspace (e.g., `plansys2_ws`).

If the `ros` command is **not recognized**, ensure your environment is sourced:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

If **dependencies are missing**, you can update and install them using `rosdep`:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /opt/ros/humble/setup.bash # Adjust to your ROS distro if necessary
source install/setup.bash
```

### Compiling the Museum Module
Ensure you are in the root of your `plansys2_ws` directory:
```bash
colcon build --symlink-install
source install/setup.bash
```

### Running the Project

**Terminal 1 (Launch ROS2 nodes):**
```bash
cd src/museum/launch
ros2 launch museum launch.py
```
> **Troubleshooting:** If you experience issues launching the nodes, try cleaning the workspace first:
> ```bash
> rm -rf build/ install/ log/ src/build/ src/install/ src/log/
> colcon build --symlink-install
> ```

**Terminal 2 (PlanSys2 Commands):**
Navigate back to your workspace root (e.g., `cd ~/plansys2_ws`).

Ensure ROS is sourced for this new terminal:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

Execute the planning sequence:
```bash
cd src/museum
bash commands.sh  # Or select another script or enter commands manually
```
