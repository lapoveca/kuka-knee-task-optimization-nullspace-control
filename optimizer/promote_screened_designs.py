#!/usr/bin/env python3
"""Select a diverse subset of pointwise-screen-passing nine-mount designs.

Selection is greedy max-min diversity in placement radius, placement
orientation, attachment family, and peripheral azimuth. It does not claim
optimality.
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


def rotation_distance(a, b):
    relative = a.inv() * b
    return relative.magnitude() / np.pi


def validate_promotion_inputs(pool, summary, proposal_path, screen_dir, envelope):
    manifest_path = screen_dir / "manifest.json"
    screening = json.loads(manifest_path.read_text())
    if screening.get("schema") != "joint_pointwise_screen_v3":
        raise ValueError("Unexpected screening manifest schema")
    if screening.get("proposal_hash") != sha(proposal_path):
        raise ValueError("Screening belongs to another proposal pool")
    if Path(pool["task_envelope"]).resolve() != envelope:
        raise ValueError("Proposal task-envelope path differs")
    if pool["task_json_sha256"] != sha(envelope / "task.json"):
        raise ValueError("Proposal task.json identity differs")
    task_meta = json.loads((envelope / "task.json").read_text())
    arrays_path = (envelope / task_meta["arrays_file"]).resolve()
    if arrays_path.parent != envelope:
        raise ValueError("Task arrays must remain inside the envelope folder")
    actual_arrays_hash = sha(arrays_path)
    if task_meta["task_arrays_sha256"] != actual_arrays_hash:
        raise ValueError("Task arrays differ from the hash in task.json")
    declared_arrays_hash = pool.get("task_arrays_sha256")
    if (
        declared_arrays_hash is not None
        and declared_arrays_hash != actual_arrays_hash
    ):
        raise ValueError("Proposal task arrays identity differs")
    if pool["urdf_sha256"] != sha(Path("models/med14_runtime.urdf")):
        raise ValueError("Proposal robot model differs")
    dependencies = screening.get("dependencies", {})
    required = {"screen_joint_proposals.py", "task_envelope.py",
                "robot_model.py", "attachment.py", "transforms.py"}
    if {Path(name).name for name in dependencies} != required:
        raise ValueError("Screen dependency coverage differs")
    for name, expected in dependencies.items():
        if sha(name) != expected:
            raise ValueError("Screening dependency changed: " + name)
    designs = pool["designs"]
    ids = [d["id"] for d in designs]
    results = summary["results"]
    recorded = [r["design_id"] for r in results]
    if not ids or len(ids) != len(set(ids)) or len(recorded) != len(set(recorded)):
        raise ValueError("Empty pool or duplicate proposal/screen IDs")
    if set(ids) != set(recorded):
        raise ValueError("Screening incomplete or contains foreign design IDs")
    if summary.get("scheduled_designs") != len(ids) or summary.get("recorded_designs") != len(recorded):
        raise ValueError("Screen summary counts differ")
    proposal_by_id = {d["id"]:d for d in designs}
    allowed = {"pointwise_screen_pass", "screen_budget_exhausted",
               "initial_pose_no_solution", "pointwise_no_solution_with_budget"}
    for result in results:
        name = result["design_id"]
        if result["status"] not in allowed:
            raise ValueError("Unresolved screening error/status: " + name)
        if result != json.loads((screen_dir / (name + ".json")).read_text()):
            raise ValueError("Screen checkpoint differs from summary: " + name)
        if result["attachment"] != proposal_by_id[name]["attachment"]:
            raise ValueError("Screen attachment differs: " + name)
        if result["status"] == "pointwise_screen_pass":
            if result["screened_poses"] != result["required_screen_poses"] or result["required_screen_poses"] != summary["screen_pose_count"]:
                raise ValueError("Screen pass has incomplete pose coverage: " + name)
    return {"screen_manifest_sha256":sha(manifest_path),
            "screen_summary_sha256":sha(screen_dir / "summary.json"),
            "screen_records_sha256":{r["design_id"]:sha(screen_dir / (r["design_id"]+".json")) for r in results}}


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

    if pool.get("schema") != "joint_proposal_pool_v3":
        raise ValueError("Expected joint_proposal_pool_v3")
    if summary.get("schema") != "joint_pointwise_screen_summary_v3":
        raise ValueError("Expected joint_pointwise_screen_summary_v3")

    verified_screening = validate_promotion_inputs(
        pool, summary, proposal_path, screen_dir, envelope
    )

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
            raise ValueError(
                f"Screened design missing from proposals: {design_id}"
            )
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

    features = []
    for design in passing:
        X_BF = np.asarray(design["placement"]["X_BF"], dtype=float)
        A = np.asarray(design["A"], dtype=float)
        attachment = design["attachment"]

        features.append({
            "design": design,
            "radius": float(np.linalg.norm(X_BF[:3, 3])),
            "placement_rotation": Rotation.from_matrix(X_BF[:3, :3]),
            "attachment_rotation": Rotation.from_matrix(A[:3, :3]),
            "attachment_type": attachment["attachment_type"],
            "azimuth_deg": attachment["azimuth_deg"],
        })

    def attachment_distance(a, b):
        if a["attachment_type"] != b["attachment_type"]:
            return 1.0

        if (
            a["attachment_type"] == "PERIPHERAL_RING"
            and b["attachment_type"] == "PERIPHERAL_RING"
        ):
            azimuth_a = float(a["azimuth_deg"])
            azimuth_b = float(b["azimuth_deg"])
            difference = abs(azimuth_a - azimuth_b) % 360.0
            if difference > 180.0:
                difference = 360.0 - difference
            return difference / 180.0

        return rotation_distance(
            a["attachment_rotation"],
            b["attachment_rotation"],
        )

    def distance(a, b):
        radius_distance = abs(a["radius"] - b["radius"])
        placement_distance = rotation_distance(
            a["placement_rotation"],
            b["placement_rotation"],
        )
        return (
            radius_distance
            + placement_distance
            + attachment_distance(a, b)
        )

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

    selected_designs = [
        features[index]["design"] for index in selected_indices
    ]

    manifest = {
        "schema": "joint_promotion_manifest_v1",
        "task_envelope": str(envelope),
        "task_json_sha256": sha(envelope / "task.json"),
        "task_arrays_sha256": sha(envelope / "task_arrays.npz"),
        "urdf_sha256": pool["urdf_sha256"],
        "proposal_pool": str(proposal_path),
        "proposal_pool_sha256": sha(proposal_path),
        "screen_dir": str(screen_dir),
        "verified_screening": verified_screening,
        "selection": {
            "method": (
                "greedy_max_min_radius_placement_attachment_family_azimuth_"
                "diversity"
            ),
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
        "selected_ids": [
            design["id"] for design in selected_designs
        ],
    }, indent=2))


if __name__ == "__main__":
    main()
