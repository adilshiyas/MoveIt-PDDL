import trimesh
import argparse
import numpy as np
from shutil import rmtree

from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

MESH_DIR = PACKAGE_DIR / "meshes"

# Workspace root (ros2-pddl/)
WORKSPACE_DIR = PACKAGE_DIR.parent.parent

# install/moveit_hanoi/share/moveit_hanoi/meshes
INSTALL_MESH_DIR = (
    WORKSPACE_DIR
    / "install"
    / "moveit_hanoi"
    / "share"
    / "moveit_hanoi"
    / "meshes"
)


def create_ring(outer_radius, inner_radius, height, filename):
    # Outer cylinder
    outer = trimesh.creation.cylinder(
        radius=outer_radius,
        height=height,
        sections=64
    )

    # Inner cylinder
    inner = trimesh.creation.cylinder(
        radius=inner_radius,
        height=height * 1.1,
        sections=64
    )
    
    # Boolean difference: outer - inner
    ring = outer.difference(inner)

    ring.export(filename)
    print(f"Saved {filename}")

def main(args):
    num_disks = int(args.disks)
    
    # Clear existing meshes
    MESH_DIR.mkdir(exist_ok=True)
    for mesh_file in MESH_DIR.glob("*.stl"):
        mesh_file.unlink()

    # Remove installed meshes so colcon copies a clean directory
    if INSTALL_MESH_DIR.exists():
        rmtree(INSTALL_MESH_DIR)
        print(f"Removed {INSTALL_MESH_DIR}")

    min_rad = 0.02
    max_rad = 0.035

    inner_rad = 0.015
    disk_thickness = 0.025

    disk_rads = np.linspace(min_rad, max_rad, num_disks)[::-1]

    for i, disk_rad in enumerate(disk_rads):
        create_ring(
            disk_rad,
            inner_rad,
            disk_thickness,
            filename=str(MESH_DIR / f"disk{i+1}.stl")
        )

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--disks", type=int, default=2)
    args = parser.parse_args()
    return args

if __name__=="__main__":
    args = parse_arguments()
    main(args)
    

    