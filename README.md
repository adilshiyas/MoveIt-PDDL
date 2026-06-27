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
You will need to underlay a MoveIt workspace. Check the MoveIt 2 documentation here.

2. Trimesh
Needed for disk mesh generation
```text
pip install trimesh
```

3. Fast Downward
For solving PDDL problems. Check the documentation on how to install. Building from source is recommended. You will need to pass your Fast Downward directory as a ROS parameter later.

## Usage

### Generating Disk Meshes
The package currently relies on running scripts/generate_disk_meshes.py to populate /meshes/ with .stl files. Generate meshes for any number of disks

```text
python scripts/generate_disk_meshes.py --disks 4
```

### Build and Run Launch Files

Build and source workspace
```text
colcon build 
source install/local_setup.bash
```

Launching RViz and MoveIt controllers
```text
ros2 launch moveit_hanoi demo.launch.py
```

Launching hanoi_mtc_executor
```text
ros2 launch moveit_hanoi hanoi_demo.launch.py
```

Run hanoi_planner node
```text
ros2 run moveit_hanoi hanoi_planner.py
```




