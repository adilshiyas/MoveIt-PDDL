# MoveIt-PDDL

A ROS 2 package demonstrating task and motion planning (TAMP) on a Franka Emika Panda manipulator. The robot solves the Tower of Hanoi puzzle by combining symbolic task planning with geometric motion planning.

The symbolic planning problem is modeled in PDDL and solved using Fast Downward. Each action is then executed using MoveIt 2 and the MoveIt Task Constructor (MTC), which computes inverse kinematics, collision-free motion plans, and gripper actions.

## Package Overview

This repository contains two primary ROS nodes:

- **hanoi_planner** – Generates a symbolic task plan by invoking the Fast Downward PDDL planner.
- **hanoi_mtc_executor** – Converts each symbolic action into a MoveIt Task Constructor task, computes IK and collision-free motion plans, and executes the resulting trajectory.

## Pipeline

```text
PDDL Problem
    ↓
Fast Downward
    ↓
Symbolic Plan
    ↓
MoveIt Task Constructor
    ↓
IK & Motion Planning
    ↓
Robot Execution
```

## Requirements
1. MoveIt 2

This package requires a working MoveIt 2 installation. Build and source a MoveIt workspace before building this package. See the MoveIt 2 installation guide for setup instructions.

2. Trimesh

Used to generate the STL meshes for the Tower of Hanoi disks.
```text
pip install trimesh
```

3. Fast Downward

Used to solve the PDDL task planning problem. Installing from source is recommended. The path to your Fast Downward installation must be provided as a ROS parameter when running the planner.

## Usage

### 1. Generating Disk Meshes
The package uses procedurally generated STL meshes for the Hanoi disks. Before building the package, generate meshes for the desired number of disks:

```text
python scripts/generate_disk_meshes.py --disks 4
```

The generated meshes will be written to the ```meshes/``` directory.

### 2. Build the Workspace

This package is intended to be built as an overlay on top of a MoveIt 2 workspace. Make sure your MoveIt 2 workspace has been sourced before building.
```text
colcon build
source install/local_setup.bash
```

### 3. Launch MoveIt 2

Start RViz, move_group, and the required controllers:
```text
ros2 launch moveit_hanoi demo.launch.py
```

### 4. Launching the MTC Executor

Start the MoveIt Task Constructor execution node:
```text
ros2 launch moveit_hanoi hanoi_demo.launch.py
```

### 5. Generate a Task Plan

Run the PDDL planner, providing the path to your Fast Downward installation via the `fast_downward_path` ROS parameter.

```text
ros2 run moveit_hanoi hanoi_planner.py --ros-args \
  -p fast_downward_path:=/path/to/fast-downward
```
Replace `/path/to/fast-downward` with the root directory of your Fast Downward installation.




