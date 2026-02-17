# Museum PlanSys2 Implementation

This package implements the PlanSys2 integration for the Inter-Planetary Museum temporal planning problem (Assignment 2.5).

## Overview

This implementation translates the temporal multi-robot PDDL domain and problem (from 2.4) into a working ROS2 PlanSys2 system. The system simulates the execution of actions using "fake" action nodes that demonstrate progress without actual robot hardware.

## Package Structure

```
2.5/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS2 package manifest
├── src/
│   └── action_node.cpp         # Generic action executor implementation
├── launch/
│   └── museum_plansys2_launch.py  # Launch file for system startup
├── pddl/
│   └── domain.pddl             # Temporal multi-robot domain (from 2.4)
├── terminal_commands.txt       # Pre-formatted PlanSys2 terminal commands
└── README.md                   # This file
```

## Actions Implemented

The package includes 14 durative actions from the domain:

### Movement
- `move-to-pressurized-room` (10s)
- `move-to-unpressurized-room` (10s)

### Sealing
- `activate-seal` (2s)

### Pod Management
- `pick-up-empty-pod-slot-1` (2s)
- `pick-up-full-pod-slot-1` (4s)
- `drop-pod-slot-1` (4s)

### Artifact Handling (Slot 1)
- `pick-up-slot-1` (1s)
- `put-in-pod-slot-1` (4s)
- `release-artifact-slot-1` (1s)
- `release-artifact-from-pod-slot-1` (4s)

### Cooling
- `cool-artifact-while-carrying-slot-1` (12s)
- `cool-artifact-while-carrying-in-pod-slot-1` (14s)

### Second Slot (Technician)
- `pick-up-slot-2` (2s)
- `release-artifact-slot-2` (2s)

## How to Use

### 1. Build the Package

Assuming you have a ROS2 workspace (e.g., `ros2_planning_ws`):

```bash
cd ~/ros2_planning_ws/src
# Copy or symlink the 2.5 directory here as museum_plansys2
ln -s /path/to/2.5 museum_plansys2

cd ~/ros2_planning_ws
colcon build --packages-select museum_plansys2
source install/setup.bash
```

### 2. Launch the System

```bash
ros2 launch museum_plansys2 museum_plansys2_launch.py
```

This will start:
- PlanSys2 infrastructure (domain expert, problem expert, planner, executor)
- All 14 action executor nodes

### 3. Define the Problem

In a **second terminal**, start the PlanSys2 terminal:

```bash
source ~/ros2_planning_ws/install/setup.bash
ros2 run plansys2_terminal plansys2_terminal
```

You will see the PlanSys2 prompt `>`. Now you can either:

**Option A**: Copy-paste commands from `terminal_commands.txt` line by line

**Option B**: Manually enter commands (see terminal_commands.txt for complete list)

Example minimal commands:
```
set instance curator robot
set instance entrance location
set predicate (robot-at curator entrance)
# ... (see terminal_commands.txt for full list)
set goal (and (artifact-at mart-nord-core-drill stasis-lab))
get plan
run
```

### 4. Watch Execution

Back in the first terminal (where you launched the system), you will see the action nodes executing:
```
[move_to_pressurized_room_node] ... [45%]
[pick_up_slot_1_node] ... [80%]
```

## Problem Description

The problem involves:
- **3 Robots**: curator, technician, scientist
- **7 Locations**: entrance, maintenance-tunnel, hall-a, hall-b, cryo-chamber, anti-vibration-pods-room, stasis-lab
- **6 Artifacts**: Various scientific, technological, and top-secret items
- **2 Pods**: For transporting fragile artifacts

### Goal
- Move `mart-nord-core-drill` to `stasis-lab` (cooled)
- Move `space-suit` to `stasis-lab`
- Move `quantum-chip` to `stasis-lab` (cooled)
- Move `mart-north-pole-ice-sample` to `cryo-chamber`
- Move `mart-mysterious-egg` to `cryo-chamber`
- Move `mart-laser-gun` to `hall-a`

### Constraints
- Tunnels require sealing mode activation
- Some artifacts need cooling in cryo-chamber
- Robots have different access permissions
- Hall-b has seismic activity (timed initial literals)

## Implementation Notes

### Action Node Design
The `action_node.cpp` implements a generic executor that:
- Takes action name and duration as parameters
- Simulates work by incrementing progress over time
- Sends feedback to PlanSys2 executor
- Displays progress percentage in terminal

### Launch File Strategy
The launch file creates one node instance per action, passing:
1. Action name (must match PDDL exactly)
2. Duration (in seconds, matches PDDL durations)

### Temporal Features
PlanSys2 handles:
- Timed initial literals (earthquake windows)
- Durative actions with conditions at start/end/overall
- Temporal constraints between actions

## Troubleshooting

### "Action not found" error
- Check that action names in launch file match domain.pddl exactly (including hyphens)

### Plan fails or no plan found
- Verify all instances are defined correctly
- Check that initial predicates match the problem specification
- Ensure goal is achievable given robot capabilities

### Nodes not executing
- Confirm all action nodes are running: `ros2 node list`
- Check node logs for errors: `ros2 node info <node_name>`

## References

- Assignment: `../Assignemnt.pdf` (Problem 2.5)
- PDDL Domain: `pddl/domain.pddl` (from 2.4)
- Original Problem: `../2.4/problem.pddl`
- PlanSys2 Documentation: https://plansys2.github.io/
