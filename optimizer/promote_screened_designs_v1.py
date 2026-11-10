#!/usr/bin/env python3
"""Select a diverse subset of pointwise-screen-passing designs and create a
promotion manifest for whole-task evaluation.

Selection is greedy max-min diversity in placement radius, placement
orientation, and attachment orientation. It does not claim optimality.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, obj):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(obj, indent=2, allow_nan=False))
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proposals", required=True)
    parser.add_argument("--screen-dir", required=True)
    parser.add_argument("--task-envelope", required=True)
    parser.add_argument("--promote-count", type=int, default=8)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    if args.promote_count < 1:
        raise ValueError("promote-count must be positive")

    proposal_path = Path(args.proposals).resolve()
    screen_dir = Path(args.screen_dir).resolve()
    envelope = Path(args.task_envelope).resolve()

    pool = json.loads(proposal_path.read_text())
    summary = json.loads((screen_dir / "summary.json").read_text())

    if pool.get("schema") != "joint_proposal_pool_v2":
        raise ValueError("Expected joint_proposal_pool_v2")
    if summary.get("schema") != "joint_pointwise_screen_summary_v2":
        raise ValueError("Expected joint_pointwise_screen_summary_v2")

    proposal_by_id = {
        design["id"]: design for design in pool["designs"]
    }
    screen_by_id = {
        result["design_id"]: result for result in summary["results"]
    }

    passing = []
    for design_id, screen in screen_by_id.items():
        if screen["status"] != "pointwise_screen_pass":
            continue
        if design_id not in proposal_by_id:
            raise ValueError(f"Screened design missing from proposals: {design_id}")
        passing.append(proposal_by_id[design_id])

    if not passing:
        raise RuntimeError("No pointwise-screen-passing designs to promote")

    if args.promote_count > len(passing):
        print(
            f"Warning: requested {args.promote_count} promotions, "
            f"but only {len(passing)} designs passed screening."
        )
        promote_count = len(passing)
    else:
        promote_count = args.promote_count

    # Feature vector: placement radius, placement rotation, attachment rotation.
    features = []
    for design in passing:
        X_BF = np.asarray(design["placement"]["X_BF"], dtype=float)
        A = np.asarray(design["A"], dtype=float)

        placement_rotation = Rotation.from_matrix(X_BF[:3, :3])
        attachment_rotation = Rotation.from_matrix(A[:3, :3])

        features.append({
            "design": design,
            "radius": float(np.linalg.norm(X_BF[:3, 3])),
            "placement_rotation": placement_rotation,
            "attachment_rotation": attachment_rotation,
        })

    def rotation_distance(a, b):
        relative = a.inv() * b
        return relative.magnitude() / np.pi

    def distance(a, b):
        radius_distance = abs(a["radius"] - b["radius"]) / (
            args.outer_radius if hasattr(args, "outer_radius") else 1.0
        )
        placement_distance = rotation_distance(
            a["placement_rotation"],
            b["placement_rotation"],
        )
        attachment_distance = rotation_distance(
            a["attachment_rotation"],
            b["attachment_rotation"],
        )
        return radius_distance + placement_distance + attachment_distance
        # Start from the design with median placement radius to avoid biasing
    # toward only the closest or farthest candidates.
    radii = np.array([feature["radius"] for feature in features])
    median_radius = float(np.median(radii))
    first_index = int(np.argmin(np.abs(radii - median_radius)))

    selected_indices = [first_index]
    remaining = [
        index for index in range(len(features))
        if index != first_index
    ]

    while len(selected_indices) < promote_count and remaining:
        best_index = None
        best_minimum_distance = -1.0

        for candidate_index in remaining:
            minimum_distance = min(
                distance(features[candidate_index], features[selected])
                for selected in selected_indices
            )
            if minimum_distance > best_minimum_distance:
                best_minimum_distance = minimum_distance
                best_index = candidate_index

        selected_indices.append(best_index)
        remaining.remove(best_index)

    selected_designs = [features[index]["design"] for index in selected_indices]

    manifest = {
        "schema": "joint_promotion_manifest_v1",
        "task_envelope": str(envelope),
        "task_json_sha256": sha(envelope / "task.json"),
        "task_arrays_sha256": sha(envelope / "task_arrays.npz"),
        "urdf_sha256": pool["urdf_sha256"],
        "proposal_pool": str(proposal_path),
        "proposal_pool_sha256": sha(proposal_path),
        "screen_dir": str(screen_dir),
        "selection": {
            "method": "greedy_max_min_radius_placement_attachment_diversity",
            "requested_count": args.promote_count,
            "selected_count": len(selected_designs),
            "available_passing_count": len(passing),
        },
        "designs": selected_designs,
        "scope": (
            "Promoted pointwise-screen-passing designs only. Pointwise passes "
            "do not establish connected-path, capability, collision, dynamic, "
            "clinical or hardware feasibility."
        ),
    }

    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError("Use a NEW promotion manifest path")
    atomic_json(output, manifest)

    print(json.dumps({
        "status": "promotion_manifest_PASS",
        "available_passing": len(passing),
        "promoted": len(selected_designs),
        "manifest": str(output),
        "selected_ids": [design["id"] for design in selected_designs],
    }, indent=2))


if __name__ == "__main__":
    main()
