#!/usr/bin/env python3
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

try:
    import numpy as np
except Exception:
    np = None


def floats(text, count=None):
    values = [float(v) for v in (text or "").split()]
    if count is not None and len(values) != count:
        raise ValueError(f"expected {count} values, got {len(values)}: {text!r}")
    return values


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: validate_urdf_offline.py ROBOT.URDF OUTPUT.JSON")
    urdf_path, output_path = map(os.path.abspath, sys.argv[1:])
    root = ET.parse(urdf_path).getroot()
    base_dir = os.path.dirname(urdf_path)
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [x.get("name") for x in links]
    joint_names = [x.get("name") for x in joints]

    parent_by_child = {}
    duplicate_children = []
    edges = []
    for joint in joints:
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        edges.append((parent, child))
        if child in parent_by_child:
            duplicate_children.append(child)
        parent_by_child[child] = parent
    roots = sorted(set(link_names) - set(parent_by_child))

    adjacency = {name: [] for name in link_names}
    for parent, child in edges:
        adjacency.setdefault(parent, []).append(child)
    visited = set()
    active = set()
    cycles = []

    def walk(node):
        if node in active:
            cycles.append(node)
            return
        if node in visited:
            return
        visited.add(node)
        active.add(node)
        for child in adjacency.get(node, []):
            walk(child)
        active.remove(node)

    for tree_root in roots:
        walk(tree_root)

    mesh_refs = []
    missing_meshes = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        mesh_refs.append(filename)
        if filename.startswith("package://"):
            resolved = None
        else:
            resolved = os.path.normpath(os.path.join(base_dir, filename))
        if not resolved or not os.path.isfile(resolved) or os.path.getsize(resolved) == 0:
            missing_meshes.append({"reference": filename, "resolved": resolved})

    inertial_rows = []
    invalid_inertials = []
    for link in links:
        inertial = link.find("inertial")
        if inertial is None:
            invalid_inertials.append({"link": link.get("name"), "reason": "missing inertial"})
            continue
        mass_node = inertial.find("mass")
        inertia_node = inertial.find("inertia")
        if mass_node is None or inertia_node is None:
            invalid_inertials.append({"link": link.get("name"), "reason": "missing mass or inertia"})
            continue
        mass = float(mass_node.get("value"))
        ixx = float(inertia_node.get("ixx")); ixy = float(inertia_node.get("ixy")); ixz = float(inertia_node.get("ixz"))
        iyy = float(inertia_node.get("iyy")); iyz = float(inertia_node.get("iyz")); izz = float(inertia_node.get("izz"))
        eig = None
        physical = mass > 0
        if np is not None:
            eig = np.linalg.eigvalsh(np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)).tolist()
            physical = physical and min(eig) > 0 and eig[0] + eig[1] + 1e-12 >= eig[2]
        else:
            physical = physical and min(ixx, iyy, izz) > 0
        inertial_rows.append({"link": link.get("name"), "mass": mass, "principal_moments": eig})
        if not physical:
            invalid_inertials.append({"link": link.get("name"), "reason": "non-physical mass/inertia", "mass": mass, "principal_moments": eig})

    movable = []
    bad_axes = []
    for joint in joints:
        if joint.get("type") == "fixed":
            continue
        axis_node = joint.find("axis")
        axis = floats(axis_node.get("xyz") if axis_node is not None else "1 0 0", 3)
        norm = math.sqrt(sum(v * v for v in axis))
        limit = joint.find("limit")
        row = {
            "name": joint.get("name"),
            "type": joint.get("type"),
            "axis": axis,
            "axis_norm": norm,
            "lower_rad": float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            "upper_rad": float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
            "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") is not None else None,
            "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") is not None else None,
        }
        movable.append(row)
        if abs(norm - 1.0) > 1e-6:
            bad_axes.append(row)

    limit_pairs = {(j["lower_rad"], j["upper_rad"]) for j in movable}
    mass_values = [row["mass"] for row in inertial_rows]
    common_mass = max(set(mass_values), key=mass_values.count) if mass_values else None
    common_mass_count = mass_values.count(common_mass) if common_mass is not None else 0

    checks = {
        "xml_parses": True,
        "single_root": len(roots) == 1,
        "tree_has_no_cycles": not cycles,
        "all_links_connected": len(visited) == len(link_names),
        "unique_link_names": len(link_names) == len(set(link_names)),
        "unique_joint_names": len(joint_names) == len(set(joint_names)),
        "each_child_has_one_parent": not duplicate_children,
        "exactly_14_movable_joints": len(movable) == 14,
        "all_meshes_resolve": not missing_meshes,
        "all_links_have_physical_inertial": not invalid_inertials,
        "all_movable_axes_unit_length": not bad_axes,
        "joint_limits_not_all_identical": len(limit_pairs) > 1,
        "mass_values_not_dominated_by_single_placeholder": common_mass_count < max(1, int(0.5 * len(mass_values))),
    }

    report = {
        "urdf": urdf_path,
        "robot_name": root.get("name"),
        "base_type": "mobile/source" if roots == ["base_link"] and "world" not in link_names else "inspect",
        "counts": {
            "links": len(links), "joints": len(joints), "movable_joints": len(movable),
            "visuals": len(root.findall(".//visual")), "collisions": len(root.findall(".//collision")),
            "unique_mesh_refs": len(set(mesh_refs)), "inertials": len(inertial_rows),
        },
        "roots": roots,
        "checks": checks,
        "movable_joints": movable,
        "missing_meshes": missing_meshes,
        "invalid_inertials": invalid_inertials,
        "bad_axes": bad_axes,
        "cycles": cycles,
        "unvisited_links": sorted(set(link_names) - visited),
        "mass_summary": {
            "total_kg": sum(mass_values),
            "most_common_mass_kg": common_mass,
            "most_common_mass_count": common_mass_count,
            "most_common_mass_ratio": (common_mass_count / len(mass_values)) if mass_values else None,
            "links_with_mass": len(mass_values),
        },
        "joint_limit_pairs": sorted([list(x) for x in limit_pairs]),
    }
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps({"counts": report["counts"], "mass_summary": report["mass_summary"], "checks": checks, "output": output_path}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
