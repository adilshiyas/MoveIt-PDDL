# MoveIt-PDDL

A ROS 2 package demonstrating task and motion planning (TAMP) on a Franka Emika Panda manipulator. The robot solves the Tower of Hanoi puzzle by combining symbolic task planning with geometric motion planning.

The symbolic planning problem is modeled in PDDL and solved using Fast Downward. Each action is then executed using MoveIt 2 and the MoveIt Task Constructor (MTC), which computes inverse kinematics, collision-free motion plans, and gripper actions.

## Package Overview

This repository contains two primary ROS nodes:

- **hanoi_planner** – Generates a symbolic task plan by invoking the Fast Downward PDDL planner.
- **hanoi_mtc_executor** – Converts each symbolic action into a MoveIt Task Constructor task, computes IK and collision-free motion plans, and executes the resulting trajectory.

<pre>
PDDL Problem → Fast Downward → Symbolic Plan → MoveIt Task Constructor → IK & Motion Planning → Robot Execution
</pre>
