import trimesh
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
MESH_DIR = SCRIPT_DIR.parent / "meshes"


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

if __name__=="__main__":
    create_ring(
        outer_radius=0.035,
        inner_radius=0.02,
        height=0.025,
        filename=str(MESH_DIR / "disk1.stl")
    )
    create_ring(
        outer_radius=0.025,
        inner_radius=0.02,
        height=0.025,
        filename=str(MESH_DIR / "disk2.stl")
    )
    