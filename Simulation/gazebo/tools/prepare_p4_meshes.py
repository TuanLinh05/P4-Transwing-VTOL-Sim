#!/usr/bin/env python3
"""Inspect and convert the public X-P4 glTF asset for the Gazebo model.

The source asset is an animation / presentation mesh, not engineering CAD.
This script intentionally keeps visual geometry separate from the simplified
collision and inertia definitions in model.sdf.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import struct
from pathlib import Path

import numpy as np


COMPONENT_FORMATS = {
    5120: ("b", 1),
    5121: ("B", 1),
    5122: ("h", 2),
    5123: ("H", 2),
    5125: ("I", 4),
    5126: ("f", 4),
}
COMPONENT_COUNTS = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
    "MAT2": 4,
    "MAT3": 9,
    "MAT4": 16,
}

SOURCE_TO_GAZEBO = np.array(
    [
        [-1.0, 0.0, 0.0],  # source -X (nose) -> Gazebo +X (forward)
        [0.0, 0.0, 1.0],   # source +Z -> Gazebo +Y (left)
        [0.0, 1.0, 0.0],   # source +Y -> Gazebo +Z (up)
    ]
)
# Rotor link local +Z must be the shaft / thrust axis because Gazebo Sim's
# MulticopterMotorModel applies thrust along the child link's local +Z.
# This orientation maps rotor (X,Y,Z) to aircraft (Y,Z,X).
ROTOR_TO_AIRCRAFT = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
)
OFFICIAL_WINGSPAN_M = 13.1 * 0.3048
CRUISE_TIME = 2.0
HOVER_TIME = 1.0


class GltfAsset:
    def __init__(self, path: Path) -> None:
        self.path = path
        with path.open("r", encoding="utf-8") as stream:
            self.data = json.load(stream)
        self.buffers = [
            (path.parent / entry["uri"]).read_bytes()
            for entry in self.data["buffers"]
        ]

    def accessor(self, index: int) -> np.ndarray:
        entry = self.data["accessors"][index]
        view = self.data["bufferViews"][entry["bufferView"]]
        count = entry["count"]
        columns = COMPONENT_COUNTS[entry["type"]]
        code, item_size = COMPONENT_FORMATS[entry["componentType"]]
        start = view.get("byteOffset", 0) + entry.get("byteOffset", 0)
        stride = view.get("byteStride", columns * item_size)
        raw = self.buffers[view["buffer"]]
        values = np.empty((count, columns), dtype=np.float64)
        unpack_format = "<" + code * columns
        for row in range(count):
            values[row] = struct.unpack_from(
                unpack_format, raw, start + row * stride
            )
        return values

    @staticmethod
    def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
        x, y, z, w = quaternion.astype(float)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                    0,
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                    0,
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                    0,
                ],
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )

    def node_matrix(
        self, node_index: int, override: dict[str, np.ndarray] | None = None
    ) -> np.ndarray:
        node = self.data["nodes"][node_index]
        if "matrix" in node:
            return np.asarray(node["matrix"], dtype=float).reshape(
                (4, 4), order="F"
            )
        override = override or {}
        translation = np.asarray(
            override.get("translation", node.get("translation", [0, 0, 0])),
            dtype=float,
        )
        rotation = np.asarray(
            override.get("rotation", node.get("rotation", [0, 0, 0, 1])),
            dtype=float,
        )
        scale = np.asarray(
            override.get("scale", node.get("scale", [1, 1, 1])),
            dtype=float,
        )
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = translation
        scale_matrix = np.diag([scale[0], scale[1], scale[2], 1])
        return (
            translation_matrix
            @ self.quaternion_matrix(rotation)
            @ scale_matrix
        )

    def animation_overrides(self, time_value: float) -> dict[int, dict]:
        overrides: dict[int, dict] = {}
        animation = self.data["animations"][0]
        for channel in animation["channels"]:
            sampler = animation["samplers"][channel["sampler"]]
            times = self.accessor(sampler["input"])[:, 0]
            values = self.accessor(sampler["output"])
            if time_value <= times[0]:
                value = values[0]
            elif time_value >= times[-1]:
                value = values[-1]
            else:
                right = int(np.searchsorted(times, time_value))
                left = right - 1
                fraction = (time_value - times[left]) / (
                    times[right] - times[left]
                )
                value = values[left] * (1 - fraction) + values[right] * fraction
                if channel["target"]["path"] == "rotation":
                    value = value / np.linalg.norm(value)
            target = channel["target"]
            overrides.setdefault(target["node"], {})[target["path"]] = value
        return overrides

    def walk(
        self,
        roots: list[int],
        time_value: float,
    ) -> tuple[list[dict], dict[int, np.ndarray]]:
        overrides = self.animation_overrides(time_value)
        geometry: list[dict] = []
        node_world: dict[int, np.ndarray] = {}

        def visit(node_index: int, parent_matrix: np.ndarray) -> None:
            node = self.data["nodes"][node_index]
            world = parent_matrix @ self.node_matrix(
                node_index, overrides.get(node_index)
            )
            node_world[node_index] = world
            if "mesh" in node:
                mesh_index = node["mesh"]
                for primitive in self.data["meshes"][mesh_index]["primitives"]:
                    attributes = primitive["attributes"]
                    positions = self.accessor(attributes["POSITION"])[:, :3]
                    homogeneous = np.column_stack(
                        [positions, np.ones(len(positions))]
                    )
                    points = (world @ homogeneous.T).T[:, :3]
                    normals = None
                    if "NORMAL" in attributes:
                        normal_matrix = np.linalg.inv(world[:3, :3]).T
                        normals = (
                            normal_matrix
                            @ self.accessor(attributes["NORMAL"])[:, :3].T
                        ).T
                        lengths = np.linalg.norm(normals, axis=1)
                        normals = normals / np.maximum(lengths[:, None], 1e-12)
                    texcoords = None
                    if "TEXCOORD_0" in attributes:
                        texcoords = self.accessor(attributes["TEXCOORD_0"])[
                            :, :2
                        ]
                    if "indices" in primitive:
                        triangles = (
                            self.accessor(primitive["indices"])[:, 0]
                            .astype(np.int64)
                            .reshape((-1, 3))
                        )
                    else:
                        triangles = np.arange(
                            len(points), dtype=np.int64
                        ).reshape((-1, 3))
                    geometry.append(
                        {
                            "node": node_index,
                            "mesh": mesh_index,
                            "name": self.data["meshes"][mesh_index]["name"],
                            "points": points,
                            "normals": normals,
                            "texcoords": texcoords,
                            "triangles": triangles,
                        }
                    )
            for child in node.get("children", []):
                visit(child, world)

        for root in roots:
            visit(root, np.eye(4))
        return geometry, node_world


def inspect(asset: GltfAsset) -> None:
    # RootNode (2) has children [body+wing assembly, left V-tail, right V-tail].
    roots = asset.data["nodes"][2]["children"]
    geometry, node_world = asset.walk(roots, time_value=2.0)
    _, hover_node_world = asset.walk(roots, time_value=1.0)
    print("Cruise geometry in the source RootNode frame")
    for item in geometry:
        points = item["points"]
        center = points.mean(axis=0)
        extent = np.ptp(points, axis=0)
        print(
            f"mesh={item['mesh']:2d} node={item['node']:2d} "
            f"center={np.round(center, 3)} extent={np.round(extent, 3)} "
            f"name={item['name']}"
        )
    all_points = np.vstack([item["points"] for item in geometry])
    print(f"total min:    {np.round(all_points.min(axis=0), 3)}")
    print(f"total max:    {np.round(all_points.max(axis=0), 3)}")
    print(f"total extent: {np.round(np.ptp(all_points, axis=0), 3)}")
    print("Animated-node origins at cruise")
    for node_index in (6, 30, 51, 57):
        matrix = node_world[node_index]
        print(
            f"node={node_index:2d} name={asset.data['nodes'][node_index]['name']}"
            f" origin={np.round(matrix[:3, 3], 3)}"
        )
    print("Cruise-to-animation-start rotations in Gazebo FLU axes")
    for node_index in (6, 30, 51, 57):
        cruise_rotation = node_world[node_index][:3, :3]
        hover_rotation = hover_node_world[node_index][:3, :3]
        delta_source = hover_rotation @ np.linalg.inv(cruise_rotation)
        delta = SOURCE_TO_GAZEBO @ delta_source @ SOURCE_TO_GAZEBO.T
        cosine = np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)
        angle = math.acos(cosine)
        if abs(math.sin(angle)) < 1e-9:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array(
                [
                    delta[2, 1] - delta[1, 2],
                    delta[0, 2] - delta[2, 0],
                    delta[1, 0] - delta[0, 1],
                ]
            ) / (2.0 * math.sin(angle))
        print(
            f"node={node_index:2d} axis={np.round(axis, 6)} "
            f"angle_deg={math.degrees(angle):.3f} "
            f"hover_origin={np.round(hover_node_world[node_index][:3, 3], 3)}"
        )


def rotation_axis_angle(rotation: np.ndarray) -> tuple[np.ndarray, float]:
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cosine)
    if abs(math.sin(angle)) < 1e-9:
        return np.array([1.0, 0.0, 0.0]), angle
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    ) / (2.0 * math.sin(angle))
    return axis / np.linalg.norm(axis), angle


def format_vector(values: np.ndarray) -> list[float]:
    return [round(float(value), 9) for value in values]


def write_obj(
    output_path: Path,
    geometry: list[dict],
    mesh_indices: set[int],
    origin_gazebo: np.ndarray,
    origin_source: np.ndarray,
    scale: float,
    local_basis: np.ndarray | None = None,
) -> dict:
    selected = [item for item in geometry if item["mesh"] in mesh_indices]
    if not selected:
        raise ValueError(f"No geometry selected for {output_path.name}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    local_basis = np.eye(3) if local_basis is None else local_basis
    lines = [
        "# Generated from the user-provided X-P4 visualization asset.",
        "# Visual mesh only; Gazebo collision and inertia are defined in SDF.",
        "mtllib p4_material.mtl",
        "usemtl p4_texture",
    ]
    vertex_offset = 1
    all_local_points: list[np.ndarray] = []

    for item in selected:
        points_gazebo = (
            (item["points"] - origin_source) @ SOURCE_TO_GAZEBO.T * scale
        )
        local_points = (points_gazebo - origin_gazebo) @ local_basis
        all_local_points.append(local_points)
        normals = item["normals"]
        if normals is None:
            normals_gazebo = np.zeros_like(local_points)
        else:
            normals_gazebo = normals @ SOURCE_TO_GAZEBO.T @ local_basis
            lengths = np.linalg.norm(normals_gazebo, axis=1)
            normals_gazebo /= np.maximum(lengths[:, None], 1e-12)
        texcoords = item["texcoords"]
        if texcoords is None:
            texcoords = np.zeros((len(local_points), 2), dtype=float)

        lines.append(f"o mesh_{item['mesh']}_{item['name'].replace(' ', '_')}")
        lines.extend(
            f"v {point[0]:.9f} {point[1]:.9f} {point[2]:.9f}"
            for point in local_points
        )
        # glTF and OBJ use opposite conventions for texture-coordinate V.
        lines.extend(
            f"vt {uv[0]:.9f} {1.0 - uv[1]:.9f}" for uv in texcoords
        )
        lines.extend(
            f"vn {normal[0]:.9f} {normal[1]:.9f} {normal[2]:.9f}"
            for normal in normals_gazebo
        )
        for triangle in item["triangles"]:
            face = [
                vertex_offset + int(triangle[0]),
                vertex_offset + int(triangle[1]),
                vertex_offset + int(triangle[2]),
            ]
            lines.append(
                "f " + " ".join(f"{index}/{index}/{index}" for index in face)
            )
        vertex_offset += len(local_points)

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    combined = np.vstack(all_local_points)
    return {
        "vertices": int(len(combined)),
        "minimum": format_vector(combined.min(axis=0)),
        "maximum": format_vector(combined.max(axis=0)),
        "extent": format_vector(np.ptp(combined, axis=0)),
    }


def export(asset: GltfAsset, output_dir: Path) -> None:
    roots = asset.data["nodes"][2]["children"]
    geometry, cruise_nodes = asset.walk(roots, CRUISE_TIME)
    _, hover_nodes = asset.walk(roots, HOVER_TIME)
    all_points = np.vstack([item["points"] for item in geometry])
    source_span = float(np.ptp(all_points[:, 2]))
    scale = OFFICIAL_WINGSPAN_M / source_span

    left_hinge_source = cruise_nodes[6][:3, 3]
    right_hinge_source = cruise_nodes[30][:3, 3]
    origin_source = (left_hinge_source + right_hinge_source) / 2.0

    def to_gazebo(point: np.ndarray) -> np.ndarray:
        return (point - origin_source) @ SOURCE_TO_GAZEBO.T * scale

    left_hinge = to_gazebo(left_hinge_source)
    right_hinge = to_gazebo(right_hinge_source)

    groups = {
        "p4_fuselage.obj": ({0, 16, 17, 18, 19}, np.zeros(3)),
        "p4_left_wing.obj": ({1, 2, 5, 8}, left_hinge),
        "p4_right_wing.obj": ({9, 10, 13}, right_hinge),
    }
    rotor_groups = {
        "p4_prop_left_inner.obj": {3, 4},
        "p4_prop_left_outer.obj": {6, 7},
        "p4_prop_right_inner.obj": {11, 12},
        "p4_prop_right_outer.obj": {14, 15},
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    texture_source = asset.path.parent / "images" / "Ptero.jpg"
    shutil.copyfile(texture_source, output_dir / "p4_texture.jpg")
    (output_dir / "p4_material.mtl").write_text(
        "\n".join(
            [
                "# Material for the X-P4 visualization mesh.",
                "newmtl p4_texture",
                "Ka 1.000000 1.000000 1.000000",
                "Kd 1.000000 1.000000 1.000000",
                "Ks 0.100000 0.100000 0.100000",
                "Ns 24.000000",
                "d 1.000000",
                "illum 2",
                "map_Kd p4_texture.jpg",
                "",
            ]
        ),
        encoding="utf-8",
    )

    repository_root = Path(__file__).resolve().parents[3]
    try:
        source_label = asset.path.relative_to(repository_root).as_posix()
    except ValueError:
        source_label = str(asset.path)

    metadata: dict = {
        "source": source_label,
        "source_reference_state_seconds": CRUISE_TIME,
        "official_wingspan_m": OFFICIAL_WINGSPAN_M,
        "source_span_units": source_span,
        "uniform_scale_m_per_source_unit": scale,
        "source_origin": format_vector(origin_source),
        "coordinate_mapping": "Gazebo (X,Y,Z) = source (-X,+Z,+Y)",
        "meshes": {},
        "link_origins": {
            "base_link": [0.0, 0.0, 0.0],
            "left_wing_link": format_vector(left_hinge),
            "right_wing_link": format_vector(right_hinge),
        },
    }

    for name, (indices, local_origin) in groups.items():
        metadata["meshes"][name] = write_obj(
            output_dir / name,
            geometry,
            indices,
            local_origin,
            origin_source,
            scale,
        )

    rotor_parent = {
        "p4_prop_left_inner.obj": left_hinge,
        "p4_prop_left_outer.obj": left_hinge,
        "p4_prop_right_inner.obj": right_hinge,
        "p4_prop_right_outer.obj": right_hinge,
    }
    rotor_origins: dict[str, list[float]] = {}
    for name, indices in rotor_groups.items():
        parts = [item for item in geometry if item["mesh"] in indices]
        points = np.vstack([item["points"] for item in parts])
        # The center of the bounding box is a robust estimate of the shaft
        # center because each propeller is exported as two blade halves.
        center_source = (points.min(axis=0) + points.max(axis=0)) / 2.0
        center_gazebo = to_gazebo(center_source)
        parent_origin = rotor_parent[name]
        rotor_origins[name] = format_vector(center_gazebo - parent_origin)
        metadata["meshes"][name] = write_obj(
            output_dir / name,
            geometry,
            indices,
            center_gazebo,
            origin_source,
            scale,
            local_basis=ROTOR_TO_AIRCRAFT,
        )
    metadata["rotor_origins_relative_to_wing"] = rotor_origins
    metadata["rotor_link_rpy_relative_to_wing"] = [
        round(math.pi / 2.0, 9),
        0.0,
        round(math.pi / 2.0, 9),
    ]

    fold_joints: dict[str, dict] = {}
    for side, node_index in (("left", 6), ("right", 30)):
        cruise_rotation = cruise_nodes[node_index][:3, :3]
        hover_rotation = hover_nodes[node_index][:3, :3]
        delta_source = hover_rotation @ np.linalg.inv(cruise_rotation)
        delta = SOURCE_TO_GAZEBO @ delta_source @ SOURCE_TO_GAZEBO.T
        axis, angle = rotation_axis_angle(delta)
        fold_joints[side] = {
            "axis_in_base_link": format_vector(axis),
            "hover_angle_rad": round(float(angle), 9),
            "hover_angle_deg": round(math.degrees(angle), 6),
        }
    metadata["fold_joints"] = fold_joints

    (output_dir / "p4_geometry_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Exported Gazebo visual meshes to {output_dir}")
    print(json.dumps(metadata, indent=2))


def parse_args() -> argparse.Namespace:
    script_path = Path(__file__).resolve()
    default_root = script_path.parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=default_root
        / "Ref"
        / "P4_clone"
        / "Ptero_Dynamics_gltf"
        / "Ptero_Dynamics.gltf",
    )
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=default_root
        / "Simulation"
        / "gazebo"
        / "models"
        / "p4_transwing_estimated"
        / "meshes",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    asset = GltfAsset(args.input.resolve())
    if args.inspect:
        inspect(asset)
        return
    export(asset, args.output.resolve())


if __name__ == "__main__":
    main()
