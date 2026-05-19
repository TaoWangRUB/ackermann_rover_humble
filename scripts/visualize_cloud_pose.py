#!/usr/bin/env python3
"""Visualize point clouds (.ply) and pose tracks (.txt) with Open3D."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Visualize one or more .ply point clouds and pose .txt files. "
            "Supported pose formats: xyz, timestamp xyz, xyz+quat, "
            "timestamp xyz+quat, RTAB-Map timestamp xyz+quat+id, "
            "KITTI 3x4, and timestamp+KITTI 3x4."
        )
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="Input files (.ply and/or pose .txt).",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Rendered point size for point clouds.",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=20,
        help="Draw a coordinate frame every N poses (0 disables frames).",
    )
    parser.add_argument(
        "--frame-size",
        type=float,
        default=0.2,
        help="Coordinate frame size for sampled poses.",
    )
    parser.add_argument(
        "--z-up",
        action="store_true",
        help="Add a world frame assuming Z is up.",
    )
    return parser.parse_args()


def load_text_matrix(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(token) for token in line.split()])
    if not rows:
        raise ValueError(f"{path} is empty or contains no numeric data")
    return np.asarray(rows, dtype=np.float64)


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = np.linalg.norm([qx, qy, qz, qw])
    if norm == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = np.asarray([qx, qy, qz, qw], dtype=np.float64) / norm
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def parse_pose_file(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = load_text_matrix(path)
    cols = data.shape[1]

    if cols == 3:
        positions = data[:, :3]
        rotations = np.repeat(np.eye(3)[None, :, :], len(positions), axis=0)
        return positions, rotations

    if cols == 4:
        positions = data[:, 1:4]
        rotations = np.repeat(np.eye(3)[None, :, :], len(positions), axis=0)
        return positions, rotations

    if cols == 7:
        positions = data[:, :3]
        rotations = np.asarray(
            [quaternion_to_matrix(*row[3:7]) for row in data], dtype=np.float64
        )
        return positions, rotations

    if cols == 8:
        positions = data[:, 1:4]
        rotations = np.asarray(
            [quaternion_to_matrix(*row[4:8]) for row in data], dtype=np.float64
        )
        return positions, rotations

    if cols == 9:
        positions = data[:, 1:4]
        rotations = np.asarray(
            [quaternion_to_matrix(*row[4:8]) for row in data], dtype=np.float64
        )
        return positions, rotations

    if cols == 12:
        matrices = data.reshape((-1, 3, 4))
        return matrices[:, :, 3], matrices[:, :, :3]

    if cols == 13:
        matrices = data[:, 1:13].reshape((-1, 3, 4))
        return matrices[:, :, 3], matrices[:, :, :3]

    raise ValueError(
        f"Unsupported pose format in {path} with {cols} columns. "
        "Expected 3, 4, 7, 8, 9, 12, or 13 columns."
    )


def color_from_index(index: int) -> np.ndarray:
    palette = np.array(
        [
            [0.85, 0.20, 0.20],
            [0.20, 0.55, 0.90],
            [0.20, 0.70, 0.35],
            [0.95, 0.65, 0.15],
            [0.55, 0.35, 0.80],
            [0.15, 0.70, 0.70],
        ],
        dtype=np.float64,
    )
    return palette[index % len(palette)]


def make_track_geometries(
    path: Path,
    positions: np.ndarray,
    rotations: np.ndarray,
    color: np.ndarray,
    frame_step: int,
    frame_size: float,
) -> list[o3d.geometry.Geometry]:
    geometries: list[o3d.geometry.Geometry] = []

    if len(positions) == 0:
        return geometries

    if len(positions) == 1:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=frame_size * 0.25)
        sphere.paint_uniform_color(color.tolist())
        sphere.translate(positions[0])
        geometries.append(sphere)
    else:
        lines = np.column_stack(
            [np.arange(len(positions) - 1), np.arange(1, len(positions))]
        )
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(positions)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(
            np.repeat(color[None, :], len(lines), axis=0)
        )
        geometries.append(line_set)

    if frame_step > 0:
        for idx in range(0, len(positions), frame_step):
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
            transform = np.eye(4)
            transform[:3, :3] = rotations[idx]
            transform[:3, 3] = positions[idx]
            frame.transform(transform)
            geometries.append(frame)

    return geometries


def load_geometries(
    paths: Iterable[str],
    frame_step: int,
    frame_size: float,
    z_up: bool,
) -> list[o3d.geometry.Geometry]:
    geometries: list[o3d.geometry.Geometry] = []
    pose_index = 0

    for raw_path in paths:
        path = Path(raw_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(path)

        suffix = path.suffix.lower()
        if suffix == ".ply":
            cloud = o3d.io.read_point_cloud(str(path))
            if cloud.is_empty():
                raise ValueError(f"{path} did not contain any points")
            geometries.append(cloud)
            continue

        if suffix == ".txt":
            positions, rotations = parse_pose_file(path)
            color = color_from_index(pose_index)
            geometries.extend(
                make_track_geometries(path, positions, rotations, color, frame_step, frame_size)
            )
            pose_index += 1
            continue

        raise ValueError(f"Unsupported file type for {path}")

    if z_up:
        geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 1.5))

    return geometries


def main() -> int:
    args = parse_args()
    geometries = load_geometries(
        args.paths,
        frame_step=args.frame_step,
        frame_size=args.frame_size,
        z_up=args.z_up,
    )

    if not geometries:
        raise RuntimeError("No geometries were created from the provided inputs")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Cloud and Pose Viewer")
    for geometry in geometries:
        vis.add_geometry(geometry)
    render_option = vis.get_render_option()
    render_option.point_size = float(args.point_size)
    render_option.mesh_show_back_face = True
    vis.run()
    vis.destroy_window()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
