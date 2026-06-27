#!/usr/bin/env python3

import rclpy
import subprocess

from pathlib import Path
from rclpy.node import Node
from moveit_hanoi.srv import ExecuteMove
from ament_index_python.packages import get_package_share_directory

FAST_DOWNWARD_PATH = "~/software/downward/fast-downward.py"
FAST_DOWNWARD_PARAM = "fast_downward_path"

class HanoiPlanner(Node):

    def __init__(self):
        super().__init__("hanoi_planner")

        self.declare_parameter(
            FAST_DOWNWARD_PARAM,
            str(Path(FAST_DOWNWARD_PATH).expanduser())
        )

        self.fast_downward_path = Path(
            self.get_parameter(FAST_DOWNWARD_PARAM).value
        ).expanduser()

        package_share = Path(get_package_share_directory("moveit_hanoi"))

        self.mesh_dir = package_share / "meshes"
        self.disk_meshes = sorted(self.mesh_dir.glob("disk*.stl"))
        self.num_disks = len(self.disk_meshes)

        if self.num_disks == 0:
            raise RuntimeError(f"No disk meshes found in {self.mesh_dir}")

        self.pddl_dir = package_share / "pddl"
        self.domain_path = self.pddl_dir / "domain.pddl"
        self.problem_path = self.pddl_dir / "problem.pddl"
        
        self.plan_dir = Path("~/.cache/moveit_hanoi").expanduser()
        self.plan_dir.mkdir(parents=True, exist_ok=True)
        self.plan_path = self.plan_dir / "sas_plan"

        self.client = self.create_client(ExecuteMove, "execute_move")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for execute_move service...")


    def move_disk(self, disk, target_peg, go_home_after):

        request = ExecuteMove.Request()
        request.disk = disk
        request.target_peg = target_peg
        request.go_home_after = go_home_after

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        return future.result()
    
    def generate_problem_file(self):

        lines = []

        lines.append(f"(define (problem hanoi-{self.num_disks})")
        lines.append("  (:domain hanoi)")
        lines.append("")

        #
        # Objects
        #
        lines.append("  (:objects")

        disk_names = [f"disk{i}" for i in range(1, self.num_disks + 1)]
        lines.append(f"    {' '.join(disk_names)} - disk")
        lines.append("    peg1 peg2 peg3 - peg")
        lines.append("  )")
        lines.append("")

        #
        # Initial state
        #
        lines.append("  (:init")

        # Largest disk sits on peg1.
        lines.append("    (on disk1 peg1)")

        # Every smaller disk sits on the next larger disk.
        for i in range(2, self.num_disks + 1):
            lines.append(f"    (on disk{i} disk{i-1})")

        # Only the smallest disk is clear.
        lines.append(f"    (clear disk{self.num_disks})")

        # Empty destination pegs.
        lines.append("    (clear peg2)")
        lines.append("    (clear peg3)")
        lines.append("")

        # Size relationships.
        # disk3 < disk2 < disk1
        for smaller in range(2, self.num_disks + 1):
            for larger in range(1, smaller):
                lines.append(f"    (smaller disk{smaller} disk{larger})")

        lines.append("  )")
        lines.append("")

        #
        # Goal
        #
        lines.append("  (:goal")
        lines.append("    (and")

        lines.append("      (on disk1 peg3)")

        for i in range(2, self.num_disks + 1):
            lines.append(f"      (on disk{i} disk{i-1})")

        lines.append("    )")
        lines.append("  )")
        lines.append(")")

        with open(self.problem_path, "w") as f:
            f.write("\n".join(lines))

        self.get_logger().info(
            f"Wrote {self.problem_path} for {self.num_disks} disks."
        )

    def execute_plan(self, plan):

        for i, (disk, target) in enumerate(plan):

            go_home_after = (i == len(plan) - 1)    
            response = self.move_disk(disk, target, go_home_after)

            if not response.success:
                self.get_logger().error(f"Execution failed for: move disk {disk} to peg {target}")
                return False
            
            self.get_logger().info(response.message)
        return True

    def parse_task_plan(self, plan_path: str):
        parsed_plan = []

        disk_to_peg = {
            "disk1": 1,
            "disk2": 1,
        }

        with open(plan_path, "r") as f:
            for raw_line in f:
                line = raw_line.strip()
                if not line:
                    continue
                if line.startswith(";"):
                    continue

                # Remove parenthesis:
                line = line.strip("()")
                tokens = line.split()

                action = tokens[0]

                if action == "move-to-peg":
                    # tokens = ["move-to-peg", disk, from, to_peg]
                    disk_name = tokens[1]
                    target_peg_name = tokens[3]

                    disk_id = disk_name_to_id(disk_name)
                    target_peg = peg_name_to_id(target_peg_name)

                    parsed_plan.append((disk_id, target_peg))
                    disk_to_peg[disk_name] = target_peg
                
                elif action == "move-to-disk":
                    # tokens = ["move-to-disk", disk, from, target_disk]
                    disk_name = tokens[1]
                    target_disk_name = tokens[3]

                    disk_id = disk_name_to_id(disk_name)
                    target_peg = disk_to_peg[target_disk_name]

                    parsed_plan.append((disk_id, target_peg))
                    disk_to_peg[disk_name] = target_peg

                else:
                    raise ValueError(f"Unknown action in plan: {action}")
        
        return parsed_plan

    def run_planner(self):
        if not self.fast_downward_path.exists():
            raise FileNotFoundError(f"Fast Downward not found at: {self.fast_downward_path}")
        if self.plan_path.exists():
            self.plan_path.unlink()
        
        cmd = [
            str(self.fast_downward_path),
            "--plan-file",
            str(self.plan_path),
            str(self.domain_path),
            str(self.problem_path),
            "--search",
            "astar(lmcut())",
        ]

        self.get_logger().info("Running Fast Downward...")
        self.get_logger().info(" ".join(cmd))

        result = subprocess.run(
            cmd,
            cwd=str(self.fast_downward_path.parent),
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            self.get_logger().error("Fast Downward failed.")
            self.get_logger().error(result.stdout)
            self.get_logger().error(result.stderr)
            return None
        
        if not self.plan_path.exists():
            self.get_logger().error(f"No plan file generated at {self.plan_path}")
            return None

        self.get_logger().info(f"Planner output written to: {self.plan_path}")
        return self.plan_path

def disk_name_to_id(name: str) -> int:
    return int(name.replace("disk", ""))

def peg_name_to_id(name: str) -> int:
    return int(name.replace("peg", ""))

    
def main():
    rclpy.init()
    node = HanoiPlanner()

    node.generate_problem_file()
    plan_path = node.run_planner()

    if plan_path is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    plan = node.parse_task_plan(str(plan_path))
    node.get_logger().info(f"Parsed plan: {plan}")
    
    plan_success = node.execute_plan(plan)
    print(f"Plan success: {plan_success}")

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()