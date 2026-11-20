#!/usr/bin/env python3
"""Export a selected optimizer knee-task path into a self-contained plan bundle.

The exporter reads a completed campaign and its controller-test manifest,
verifies the selected trajectory identity, copies the required evidence, and
creates a dense end-effector reference for the ROS 2 CIC workflow.

It does not modify the campaign directory.
"""

import argparse
import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
OPTIMIZER_ROOT = REPOSITORY_ROOT / "optimizer"

for import_path in (REPOSITORY_ROOT, OPTIMIZER_ROOT):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from task_envelope import TaskEnvelope, transform
from robot_model import RobotModel


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, content):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(content, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def copy_verified(source, destination, expected_hash=None):
    source = Path(source).resolve()
    destination = Path(destination).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)

    actual = sha256(destination)
    if expected_hash is not None and actual != expected_hash:
        raise ValueError(f"Hash mismatch after copy: {source}")

    return actual


def find_inputs_json(start):
    current = Path(start).resolve()
    for candidate in [current, *current.parents]:
        possible = candidate / "inputs.json"
        if possible.is_file():
            return possible
    raise FileNotFoundError("Could not locate evaluation inputs.json")


def finite_transform(matrix, name):
    matrix = np.asarray(matrix, dtype=float)
    transform(matrix)
    return matrix


def main():
    parser = argparse.ArgumentParser(
        description="Export a selected optimizer knee-task path."
    )
    parser.add_argument(
        "--campaign-dir",
        required=True,
        help="Completed campaign directory.",
    )
    parser.add_argument(
        "--manifest",
        required=True,
        help="controller_test_manifest.json produced by the campaign reviewer.",
    )
    parser.add_argument(
        "--rank",
        type=int,
        default=1,
        help="Manifest rank to export.",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=50,
        help="Dense end-effector reference rate in Hz.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="New export directory. Defaults to output/selected_plans/<timestamp>.",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("Reference rate must be positive.")

    campaign_dir = Path(args.campaign_dir).resolve()
    manifest_path = Path(args.manifest).resolve()

    if not campaign_dir.is_dir():
        raise FileNotFoundError(f"Campaign directory not found: {campaign_dir}")
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")

    manifest_entries = load_json(manifest_path)
    selected = next(
        (entry for entry in manifest_entries if int(entry["rank"]) == args.rank),
        None,
    )
    if selected is None:
        raise ValueError(f"Rank {args.rank} not found in manifest.")

    selected_path = Path(selected["selected_path"]).resolve()
    reference_file = selected_path / "whole_trajectory_path.npz"

    if not selected_path.is_dir():
        raise FileNotFoundError(f"Selected path not found: {selected_path}")
    if not reference_file.is_file():
        raise FileNotFoundError(f"Reference not found: {reference_file}")

    reference_hash = sha256(reference_file)
    if reference_hash != selected["selected_path_sha256"]:
        raise ValueError("Selected trajectory hash does not match the manifest.")

    campaign_manifest = load_json(campaign_dir / "campaign_manifest.json")
    campaign_summary = load_json(campaign_dir / "campaign_summary.json")

    if campaign_summary.get("status") != "complete":
        raise ValueError("Campaign is not complete.")

    task_envelope_dir = Path(campaign_manifest["task_envelope"]).resolve()
    budget_path = Path(campaign_manifest["budget"]).resolve()
    weights_path = Path(campaign_manifest["weights"]).resolve()

    urdf_path = next(
        (
            path
            for path in campaign_manifest["files"]
            if path.endswith("med14_runtime.urdf")
        ),
        None,
    )
    if urdf_path is None:
        raise ValueError("Runtime URDF is absent from the campaign manifest.")
    urdf_path = Path(urdf_path).resolve()

    for path, expected in [
        (task_envelope_dir / "task.json", campaign_manifest["files"][str(task_envelope_dir / "task.json")]),
        (task_envelope_dir / "task_arrays.npz", campaign_manifest["files"][str(task_envelope_dir / "task_arrays.npz")]),
        (budget_path, campaign_manifest["files"][str(budget_path)]),
        (weights_path, campaign_manifest["files"][str(weights_path)]),
        (urdf_path, campaign_manifest["files"][str(urdf_path)]),
    ]:
        if sha256(path) != expected:
            raise ValueError(f"Campaign input hash mismatch: {path}")

    inputs_path = find_inputs_json(selected_path)
    inputs = load_json(inputs_path)

    placement = inputs["placement"]
    if placement.get("frame") != "X_BF":
        raise ValueError("Evaluation placement does not declare frame X_BF.")

    candidate = next(
        (
            item
            for item in inputs["candidates"]
            if item["id"] == selected["design_id"]
        ),
        None,
    )
    if candidate is None:
        raise ValueError("Selected design is absent from evaluation inputs.")

    with np.load(reference_file, allow_pickle=False) as archive:
        time_q = np.array(archive["time_s"], dtype=float)
        q_reference = np.array(archive["q"], dtype=float)
        attachment = np.array(archive["A"], dtype=float)
        femur_placement = np.array(archive["X_BF"], dtype=float)
        global_reference = np.array(archive["X_BG"], dtype=float)

    attachment = finite_transform(attachment, "A")
    femur_placement = finite_transform(femur_placement, "X_BF")
    global_reference = finite_transform(global_reference, "X_BG")

    if not np.allclose(attachment, candidate["A"], atol=1e-10, rtol=0):
        raise ValueError("Attachment transform differs from evaluation inputs.")

    if not np.allclose(femur_placement, placement["X_BF"], atol=1e-10, rtol=0):
        raise ValueError("Femur placement differs from evaluation inputs.")

    envelope = TaskEnvelope(task_envelope_dir)
    robot = RobotModel()

    if time_q[0] != envelope.time_s[0] or time_q[-1] != envelope.time_s[-1]:
        raise ValueError("Posture path does not cover the task duration.")

    q_start = q_reference[0].copy()
    attachment_inverse = np.linalg.inv(attachment)

    required_start_ee = envelope.required_ee(
        float(time_q[0]),
        femur_placement,
        attachment,
    )

    required_start_tibia = femur_placement @ envelope.pose_at(float(time_q[0]))

    fk_start_ee = robot.fk_ee(q_start)
    fk_start_tibia = fk_start_ee @ attachment

    position_error = float(
        np.linalg.norm(
            fk_start_ee[:3, 3] - required_start_ee[:3, 3]
        )
    )
    rotation_error = float(
        Rotation.from_matrix(
            fk_start_ee[:3, :3] @ required_start_ee[:3, :3].T
        ).magnitude()
    )
    
    start_position_tolerance, start_rotation_tolerance = envelope.tolerances(
        float(time_q[0])
    )

    if (
        position_error > start_position_tolerance
        or rotation_error > start_rotation_tolerance
    ):
        raise ValueError(
            "FK start pose exceeds the task start tolerance."
        )

    sample_period = 1.0 / args.rate
    dense_time = np.arange(
        envelope.time_s[0],
        envelope.time_s[-1] + sample_period,
        sample_period,
    )
    if dense_time[-1] > envelope.time_s[-1]:
        dense_time[-1] = envelope.time_s[-1]

    dense_ee = np.empty((len(dense_time), 4, 4))
    dense_tibia = np.empty((len(dense_time), 4, 4))

    for index, time in enumerate(dense_time):
        dense_ee[index] = envelope.required_ee(
            float(time),
            femur_placement,
            attachment,
        )

        dense_tibia[index] = femur_placement @ envelope.pose_at(float(time))


    joint_velocity = (
        np.diff(q_reference, axis=0)
        / np.diff(time_q)[:, None]
    )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    design_id = selected["design_id"]
    path_id = Path(selected_path).name

    if args.output_dir is None:
        output_dir = (
            campaign_dir
            / "selected_plans"
            / f"{timestamp}_{design_id}_{path_id}"
        )
    else:
        output_dir = Path(args.output_dir).resolve()

    campaign_resolved = campaign_dir.resolve()
    output_resolved = output_dir.resolve()

    if output_resolved == campaign_resolved or campaign_resolved in output_resolved.parents:
        raise ValueError(
            "Export directory must not be inside the campaign directory."
        )

    output_dir.mkdir(parents=True)

    provenance = output_dir / "provenance"
    provenance.mkdir()

    copied = {}

    copied["task_envelope"] = copy_verified(
        task_envelope_dir / "task.json",
        output_dir / "task" / "task.json",
    )
    copied["task_arrays"] = copy_verified(
        task_envelope_dir / "task_arrays.npz",
        output_dir / "task" / "task_arrays.npz",
    )
    copied["budget"] = copy_verified(
        budget_path,
        output_dir / "budget.json",
    )
    copied["score_policy"] = copy_verified(
        weights_path,
        output_dir / "score_policy.json",
    )
    copied["urdf"] = copy_verified(
        urdf_path,
        output_dir / "models" / "med14_runtime.urdf",
    )
    copied["campaign_manifest"] = copy_verified(
        campaign_dir / "campaign_manifest.json",
        provenance / "campaign_manifest.json",
    )
    copied["campaign_summary"] = copy_verified(
        campaign_dir / "campaign_summary.json",
        provenance / "campaign_summary.json",
    )
    copied["controller_manifest"] = copy_verified(
        manifest_path,
        provenance / "controller_test_manifest.json",
    )
    copied["evaluation_inputs"] = copy_verified(
        inputs_path,
        provenance / "evaluation_inputs.json",
    )
    copied["audit_report"] = copy_verified(
        selected["audit_report"],
        provenance / "audit_report.json",
    )
    copied["path_report"] = copy_verified(
        selected_path / "report.json",
        provenance / "path_report.json",
    )
    copied["path_capability"] = copy_verified(
        selected_path / "capability.json",
        provenance / "capability.json",
    )
    copied["path_capability_cases"] = copy_verified(
        selected_path / "capability_cases.csv",
        provenance / "capability_cases.csv",
    )
    copied["whole_trajectory_path"] = copy_verified(
        reference_file,
        provenance / "whole_trajectory_path.npz",
        expected_hash=reference_hash,
    )

    np.savez_compressed(
        output_dir / "reference.npz",
        time_task_s=envelope.time_s,
        X_FT_task=envelope.X_FT,
        time_ee_s=dense_time,
        X_BE_task=dense_ee,
        X_BT_task=dense_tibia,
        time_q_s=time_q,
        q_ref_rad=q_reference,
        qdot_segment_rad_s=joint_velocity,
        A_X_ET=attachment,
        X_TE=attachment_inverse,
        X_BF=femur_placement,
        X_GF_reference=envelope.X_GF_reference,
        X_BG=global_reference,
        q_start_rad=q_start,
        X_BE_start_required=required_start_ee,
        X_BT_start_required=required_start_tibia,
        X_BE_start_fk=fk_start_ee,
        X_BT_start_fk=fk_start_tibia,
    )

    generated_reference_hash = sha256(output_dir / "reference.npz")

    plan = {
        "schema": "knee_selected_plan_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_dir": str(campaign_dir),
        "manifest": str(manifest_path),
        "rank": args.rank,
        "design_id": design_id,
        "arm": selected["arm"],
        "attachment_id": selected["attachment_id"],
        "selected_path": str(selected_path),
        "selected_path_sha256": reference_hash,
        "task_id": envelope.meta["task_id"],
        "task_duration_s": float(envelope.time_s[-1]),
        "reference_rate_hz": args.rate,
        "joint_order": [
            f"lbr_A{index}" for index in range(1, 8)
        ],
        "q_start_rad": q_start.tolist(),
        "reference_frame": "lbr_link_0",
        "end_effector_frame": "lbr_link_ee",
        "tibia_frame": "tibia",
        "reference_semantics": {
            "X_BE": "X_BF * X_FT * inverse(A)",
            "X_BT": "X_BF * X_FT",
            "X_BT_actual": "FK(q_measured) * A",
        },
        "execution": {
            "short_baseline_s": 70.0,
            "full_baseline_s": float(envelope.time_s[-1]),
            "secondary_control": "off",
            "primary_gains": {
                "translational_stiffness": 3000.0,
                "rotational_stiffness": 150.0,
                "translational_damping_ratio": 1.0,
                "rotational_damping_ratio": 1.0,
            },
        },
        "files": copied,
        "generated": {
            "reference_npz": generated_reference_hash,
        },
    }

    write_json(output_dir / "plan.json", plan)

    validation = {
        "schema": "knee_selected_plan_validation_v1",
        "status": "pass",
        "checks": {
            "campaign_complete": True,
            "selected_path_hash": True,
            "campaign_input_hashes": True,
            "attachment_matches_inputs": True,
            "femur_placement_matches_inputs": True,
            "task_duration_matches_posture_path": True,
            "transforms_valid": True,
            "attachment_inverse_consistent": bool(
                np.allclose(
                    attachment @ attachment_inverse,
                    np.eye(4),
                    atol=1e-10,
                    rtol=0,
                )
            ),
            "dense_reference_consistent": bool(
                np.allclose(
                    dense_ee[0],
                    required_start_ee,
                    atol=1e-12,
                    rtol=0,
                )
            ),
            "fk_start_position_error_m": position_error,
            "fk_start_rotation_error_rad": rotation_error,
        },
        "limitations": [
            "Offline optimizer metrics are not controller tracking results.",
            "Capability is a nominal actuator-budget model, not measured load capacity.",
            "Collision checking is sampled, not continuous swept-volume certification.",
        ],
    }

    write_json(output_dir / "validation_report.json", validation)

    readme = f"""# Selected knee plan

Design: `{design_id}`
Rank: `{args.rank}`
Path: `{path_id}`
Task duration: `{envelope.time_s[-1]:.1f}` s
Reference rate: `{args.rate}` Hz

This bundle contains a copied, hash-verified selected optimizer result and a
dense end-effector reference for the ROS 2 CIC workflow.

The campaign directory remains unchanged.

## Reference

The end-effector reference is

    X_BE(t) = X_BF * X_FT(t) * inverse(A)

The requested tibia pose is

    X_BT(t) = X_FT(t)

The realized tibia pose is

    X_BT_actual(t) = FK(q_measured(t)) * A

## Baseline

Run the short baseline first:

    python3 scripts/run_selected_knee_baseline.py \\
      --plan-dir {output_dir} \\
      --duration 70

Run the full task only after the short baseline passes:

    python3 scripts/run_selected_knee_baseline.py \\
      --plan-dir {output_dir} \\
      --duration {envelope.time_s[-1]:.0f}

The baseline uses secondary control off.
"""

    (output_dir / "README.md").write_text(readme, encoding="utf-8")

    print(f"Exported selected plan: {output_dir}")
    print(f"Design: {design_id}")
    print(f"Path: {path_id}")
    print(f"Task duration: {envelope.time_s[-1]:.1f} s")
    print(f"Reference samples: {len(dense_time)}")
    print(f"FK start position error: {position_error:.3e} m")
    print(f"FK start rotation error: {rotation_error:.3e} rad")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise
