#!/usr/bin/env python3
"""Robot-informed global placement x multiple-attachment proposal generator.

Constructs globally distributed relative placements from sampled reserved-range
robot configurations:

    X_BF = FK(q0) @ A @ inverse(X_FT(t0))

The constructing robot configuration q0 is saved for each design and can be
used as an IK seed by the matching screener.

Applies:
- robot-base-centered placement shell,
- attachment forbidden-direction cone,
- necessary URDF outer-reach rejection,
- rotation-geodesic attachment diversity.

This is NOT an IK, connected-path, collision, clinical, hardware-mounting or
global-optimality certificate.
"""
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from attachment import make_attachment_candidate
from robot_model import RobotModel
from task_envelope import TaskEnvelope, transform


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation_to_angles(R):
    if isinstance(R, Rotation):
        rotation = R
    else:
        rotation = Rotation.from_matrix(np.asarray(R, dtype=float))
    angles = rotation.as_euler("xyz", degrees=True)
    return {
        "roll_deg": float(angles[0]),
        "pitch_deg": float(angles[1]),
        "yaw_deg": float(angles[2]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-envelope", required=True)
    parser.add_argument("--placement-count", type=int, default=12)
    parser.add_argument(
        "--extra-attachments-per-placement", type=int, default=3
    )
    parser.add_argument("--inner-radius-m", type=float, default=0.30)
    parser.add_argument("--outer-radius-m", type=float, default=1.20)
    parser.add_argument(
        "--min-attachment-separation-deg", type=float, default=25.0
    )
    parser.add_argument("--rng-seed", type=int, default=5001)
    parser.add_argument("--max-attempts", type=int, default=100000)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    out = Path(args.output_dir)
    if out.exists():
        raise FileExistsError("Use a NEW output directory")

    if args.placement_count < 1:
        raise ValueError("placement-count must be positive")
    if args.extra_attachments_per_placement < 0:
        raise ValueError("extra-attachments-per-placement must be nonnegative")
    if not (0.0 < args.inner_radius_m < args.outer_radius_m):
        raise ValueError("Require 0 < inner radius < outer radius")
    if args.min_attachment_separation_deg < 0.0:
        raise ValueError("Invalid attachment separation")
    if args.max_attempts < 1:
        raise ValueError("Invalid attempt budget")

    task = TaskEnvelope(args.task_envelope)
    robot = RobotModel()
    rng = np.random.default_rng(args.rng_seed)

    # Necessary outer-reach bound from fixed URDF origin offsets.
    urdf = ET.fromstring(robot.urdf_path.read_bytes())
    children = {
        joint.find("child").attrib["link"]: joint
        for joint in urdf.findall("joint")
    }

    link = "lbr_link_ee"
    chain_lengths = []
    seen_links = set()

    while link != "lbr_link_0":
        if link in seen_links or link not in children:
            raise ValueError("Cannot trace EE to audited base")
        seen_links.add(link)

        joint = children[link]
        if joint.attrib["type"] not in ("fixed", "revolute", "continuous"):
            raise ValueError("Unsupported joint type in reach-bound trace")
        if joint.find("mimic") is not None:
            raise ValueError("Mimic joints unsupported")

        origin = joint.find("origin")
        xyz = np.fromstring(
            origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            sep=" ",
        )
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("Invalid URDF origin")

        chain_lengths.append(float(np.linalg.norm(xyz)))
        link = joint.find("parent").attrib["link"]

    outer_bound_m = sum(chain_lengths) + 1e-10
    if outer_bound_m <= 0.0:
        raise ValueError("Invalid outer reach bound")

    reach_radii_m = outer_bound_m + np.array(
        [task.tolerances(time_s)[0] for time_s in task.time_s]
    )

    reserved_lower = robot.position_lower + np.deg2rad(5.0)
    reserved_upper = robot.position_upper - np.deg2rad(5.0)
    if np.any(reserved_lower >= reserved_upper):
        raise ValueError("Reserved joint range is empty")

    initial_time_s = float(task.time_s[0])
    initial_X_FT = task.pose_at(initial_time_s)

    def attachment_offsets(A):
        X_TE = np.linalg.inv(A)
        return task.X_FT[:, :3, 3] + np.einsum(
            "nij,j->ni",
            task.X_FT[:, :3, :3],
            X_TE[:3, 3],
        )

    def outer_reach_valid(X_BF, A):
        target_origins = X_BF[:3, 3] + attachment_offsets(A) @ X_BF[:3, :3].T
        distances = np.linalg.norm(target_origins, axis=1)
        return bool(np.all(distances <= reach_radii_m + 1e-10))

    def shell_valid(X_BF):
        distance = np.linalg.norm(X_BF[:3, 3])
        return (
            args.inner_radius_m
            <= distance
            <= args.outer_radius_m
        )

    def sample_attachment():
        while True:
            angles = rotation_to_angles(Rotation.random(random_state=rng))
            try:
                A = make_attachment_candidate(
                    roll_angle_rad=np.deg2rad(angles["roll_deg"]),
                    pitch_angle_rad=np.deg2rad(angles["pitch_deg"]),
                    yaw_angle_rad=np.deg2rad(angles["yaw_deg"]),
                    tibia_offset_m=0.35,
                )
                return angles, A
            except ValueError:
                continue

    def attachment_separated(candidate_A, accepted_attachments):
        R_new = Rotation.from_matrix(candidate_A[:3, :3])
        for _, A_old in accepted_attachments:
            R_old = Rotation.from_matrix(A_old[:3, :3])
            relative = R_new.inv() * R_old
            if relative.magnitude() < np.deg2rad(
                args.min_attachment_separation_deg
            ):
                return False
        return True

    designs = []
    rejection_counts = {
        "shell_rejections": 0,
        "outer_bound_rejections": 0,
        "attachment_separation_rejections": 0,
        "placements_skipped": 0,
    }

    for placement_index in range(args.placement_count):
        accepted_attachments = None
        placement = None
        constructing_q0 = None

        for _ in range(args.max_attempts):
            q0 = reserved_lower + (
                reserved_upper - reserved_lower
            ) * rng.uniform(0.05, 0.95, 7)

            angles, A = sample_attachment()

            X_BF = (
                robot.fk_ee(q0)
                @ A
                @ np.linalg.inv(initial_X_FT)
            )

            if not shell_valid(X_BF):
                rejection_counts["shell_rejections"] += 1
                continue

            if not outer_reach_valid(X_BF, A):
                rejection_counts["outer_bound_rejections"] += 1
                continue

            accepted_attachments = [(angles, A)]
            placement = X_BF
            constructing_q0 = q0
            break

        if accepted_attachments is None:
            rejection_counts["placements_skipped"] += 1
            print(
                f"SKIP placement {placement_index}: "
                "no robot-informed proposal passed shell/reach tests",
                flush=True,
            )
            continue

        target_attachment_count = 1 + args.extra_attachments_per_placement

        for _ in range(args.max_attempts):
            if len(accepted_attachments) >= target_attachment_count:
                break

            angles, A = sample_attachment()

            if not attachment_separated(A, accepted_attachments):
                rejection_counts["attachment_separation_rejections"] += 1
                continue

            if not outer_reach_valid(placement, A):
                rejection_counts["outer_bound_rejections"] += 1
                continue

            accepted_attachments.append((angles, A))

        for attachment_index, (angles, A) in enumerate(accepted_attachments):
            design_id = (
                f"global_robotinformed_p{placement_index:03d}"
                f"_a{attachment_index:02d}"
            )

            target_origins = (
                placement[:3, 3]
                + attachment_offsets(A) @ placement[:3, :3].T
            )

            designs.append({
                "id": design_id,
                "proposal_kind": "global_robot_informed_multi_attachment",
                "placement": {
                    "frame": "X_BF",
                    "X_BF": placement.tolist(),
                },
                "attachment": {
                    "id": f"attachment_{attachment_index:02d}",
                    **angles,
                },
                "A": A.tolist(),
                "constructing_q0_rad": constructing_q0.tolist(),
                "status": "structural_outer_reach_pass_NOT_IK",
                "maximum_target_origin_radius_m": float(
                    np.linalg.norm(target_origins, axis=1).max()
                ),
            })

    if not designs:
        raise RuntimeError(
            "No designs generated; inspect shell, attachment cone, "
            "and outer-reach restrictions."
        )

    out.mkdir(parents=True)

    payload = {
        "schema": "joint_proposal_pool_v2",
        "task_envelope": str(Path(args.task_envelope).resolve()),
        "task_json_sha256": sha(
            Path(args.task_envelope) / "task.json"
        ),
        "urdf_sha256": robot.urdf_sha256,
        "settings": vars(args),
        "placement_domain": {
            "frame": "robot_base_B",
            "metric": (
                "distance from robot-base origin to task/femur reference origin"
            ),
            "inner_radius_m": args.inner_radius_m,
            "outer_radius_m": args.outer_radius_m,
            "construction": (
                "X_BF = FK(q0) @ A @ inverse(X_FT(t0)); "
                "initial EE pose exactly reachable by construction"
            ),
            "interpretation": (
                "Proposal-sampling shell only; not collision-free, clinically "
                "admissible, or physically mountable."
            ),
        },
        "outer_bound": {
            "reference": "lbr_link_0 origin",
            "radius_m": outer_bound_m,
            "chain_offset_sum_m": sum(chain_lengths),
            "chain_lengths_m": chain_lengths,
        },
        "rejection_counts": rejection_counts,
        "designs": designs,
        "note": (
            "Robot-informed global relative placements with multiple "
            "attachments. No full-path, collision, dynamics or hardware claims."
        ),
    }

    proposal_path = out / "proposals.json"
    proposal_path.write_text(
        json.dumps(payload, indent=2, allow_nan=False)
    )

    print(json.dumps({
        "status": "global_robot_informed_proposal_generation_PASS",
        "placements": args.placement_count,
        "designs": len(designs),
        "rejections": rejection_counts,
        "outer_radius_m": outer_bound_m,
        "proposals": str(proposal_path),
    }, indent=2))


if __name__ == "__main__":
    main()
