#!/usr/bin/env python3
"""
Generate oriented bounding-box meshes for all STL files under meshes/collision.
Outputs to meshes/collision_boxes/<name>_box.stl without touching originals.
"""
from pathlib import Path

import trimesh


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    src_dir = repo_root / "meshes" / "collision"
    out_dir = repo_root / "meshes" / "collision_boxes"
    out_dir.mkdir(parents=True, exist_ok=True)

    for stl_path in sorted(src_dir.glob("*.stl")):
        mesh = trimesh.load_mesh(stl_path, force="mesh")
        if mesh.is_empty:
            print(f"[skip] {stl_path.name}: empty mesh")
            continue

        box_mesh = mesh.bounding_box_oriented.to_mesh()
        out_path = out_dir / f"{stl_path.stem}_box.stl"
        box_mesh.export(out_path)
        print(f"[ok] {stl_path.name} -> {out_path.relative_to(repo_root)}")


if __name__ == "__main__":
    main()
