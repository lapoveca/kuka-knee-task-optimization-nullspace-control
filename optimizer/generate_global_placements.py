#!/usr/bin/env python3
"""Robot-informed placement and nine-attachment proposal generator.

A placement is constructed using a fixed reference attachment parameterization:

    X_BF = FK(q0) @ A_reference @ inverse(X_FT(t0))

The reference attachment is used only to parameterize placement sampling. It
is not a preferred or primary attachment candidate.

After a placement is accepted, all nine discrete attachment candidates are
generated with equal importance:

* one axial in-line attachment;
* eight peripheral attachments at 45-degree azimuth increments.

Each attachment must pass the necessary URDF-derived outer-reach bound. This
is not an IK, connected-path, collision, hardware, or optimality certificate.
"""
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from attachment import make_attachment_catalog, make_axial_inline_attachment
from robot_model import RobotModel
from task_envelope import TaskEnvelope, transform
from placement_domain import PlacementDomain


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-envelope", required=True)
    parser.add_argument(
        "--placement-domain",
        default="configs/placement_domain.json",
        help="Placement-domain JSON configuration.",
    )
    parser.add_argument("--rng-seed", type=int, default=5001)
    parser.add_argument("--placements-per-arm", type=int, default=10)
    parser.add_argument(
        "--allow-placement-count-override",
        action="store_true",
        help=(
            "Allow a pilot placement count that differs from the placement "
            "domain configuration. Use only for short validation runs."
        ),
    )
    parser.add_argument(
        "--max-proposal-attempts", type=int, default=100000
    )
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    out = Path(args.output_dir)
    if out.exists():
        raise FileExistsError("Use a NEW output directory")

    if args.placements_per_arm < 1:
        raise ValueError("placements-per-arm must be positive")
    if args.max_proposal_attempts < 1:
        raise ValueError("Invalid global proposal attempt budget")

    task = TaskEnvelope(args.task_envelope)
    robot = RobotModel()
    placement_domain = PlacementDomain.from_json(args.placement_domain)
    required_arms = list(placement_domain.enabled_arms)

    if not required_arms:
        raise ValueError("No enabled arms configured in placement domain")

    if (
        args.placements_per_arm != placement_domain.placements_per_arm
        and not args.allow_placement_count_override
    ):
        raise ValueError(
            "placements-per-arm must match placement-domain "
            f"placements_per_arm={placement_domain.placements_per_arm}; "
            "use --allow-placement-count-override for an explicit pilot run"
        )

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
        if joint.attrib["type"] not in (
            "fixed", "revolute", "continuous"
        ):
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

    catalog = make_attachment_catalog()
    if len(catalog) != 9:
        raise RuntimeError("Attachment catalog must contain exactly nine mounts")

    reference_entry = catalog[0]
    if reference_entry["attachment_id"] != "axial_inline":
        raise RuntimeError(
            "The first catalog entry must be axial_inline and is used only "
            "to parameterize placement construction"
        )

    reference_A = np.asarray(reference_entry["A"], dtype=float)
    reference_X_TE = np.asarray(reference_entry["X_TE"], dtype=float)

    def attachment_target_origins(X_BF, X_TE):
        """EE origins for every task time, expressed in the robot base frame."""
        X_FT_translations = task.X_FT[:, :3, 3]
        X_FT_rotations = task.X_FT[:, :3, :3]

        attachment_translations_t = X_FT_translations + np.einsum(
            "nij,j->ni",
            X_FT_rotations,
            X_TE[:3, 3],
        )

        return (
            X_BF[:3, 3]
            + np.einsum(
                "ij,nj->ni",
                X_BF[:3, :3],
                attachment_translations_t,
            )
        )

    def outer_reach_valid(X_BF, X_TE):
        target_origins = attachment_target_origins(X_BF, X_TE)
        distances = np.linalg.norm(target_origins, axis=1)
        return bool(np.all(distances <= reach_radii_m + 1e-10))

    def placement_valid(X_BF):
        return placement_domain.contains(X_BF[:3, 3])

    designs = []
    rejection_counts = {
        "placement_domain_rejections": 0,
        "outer_bound_rejections": 0,
        "disabled_arm_rejections": 0,
        "arm_quota_rejections": 0,
        "placements_skipped": 0,
    }

    arm_counts = {arm: 0 for arm in required_arms}
    proposal_attempts = 0

    while True:
        pending_arms = [
            arm
            for arm in required_arms
            if arm_counts[arm] < args.placements_per_arm
        ]
        if not pending_arms:
            break

        if proposal_attempts >= args.max_proposal_attempts:
            break

        proposal_attempts += 1

        q0 = reserved_lower + (
            reserved_upper - reserved_lower
        ) * rng.uniform(0.05, 0.95, 7)

        X_BF = (
            robot.fk_ee(q0)
            @ reference_A
            @ np.linalg.inv(initial_X_FT)
        )

        if not placement_valid(X_BF):
            rejection_counts["placement_domain_rejections"] += 1
            continue

        arm = placement_domain.arm_of(X_BF[:3, 3])

        if arm is None or arm not in arm_counts:
            rejection_counts["disabled_arm_rejections"] += 1
            continue

        if arm_counts[arm] >= args.placements_per_arm:
            rejection_counts["arm_quota_rejections"] += 1
            continue

        if not outer_reach_valid(X_BF, reference_X_TE):
            rejection_counts["outer_bound_rejections"] += 1
            continue

        attachment_outer_reach = {}
        all_attachments_outer_reach_valid = True

        for entry in catalog:
            X_TE = np.asarray(entry["X_TE"], dtype=float)
            valid = outer_reach_valid(X_BF, X_TE)
            attachment_outer_reach[entry["attachment_id"]] = valid

            if not valid:
                all_attachments_outer_reach_valid = False

        if not all_attachments_outer_reach_valid:
            rejection_counts["outer_bound_rejections"] += 1
            rejection_counts["placements_skipped"] += 1
            continue

        arm_counts[arm] += 1
        placement_sequence = arm_counts[arm]
        placement = X_BF

        for entry in catalog:
            attachment_id = entry["attachment_id"]
            A = np.asarray(entry["A"], dtype=float)
            X_TE = np.asarray(entry["X_TE"], dtype=float)

            target_origins = attachment_target_origins(X_BF, X_TE)
            maximum_target_origin_radius_m = float(
                np.linalg.norm(target_origins, axis=1).max()
            )

            design_id = (
                f"placement_{arm}_{placement_sequence:03d}"
                f"__{attachment_id}"
            )

            designs.append({
                "id": design_id,
                "proposal_kind": "robot_informed_placement_nine_mount",
                "placement": {
                    "frame": "X_BF",
                    "X_BF": placement.tolist(),
                    "point": "femur_reference_origin",
                    "arm": arm,
                },
                "attachment": {
                    "attachment_id": attachment_id,
                    "attachment_type": entry["attachment_type"],
                    "axial_offset_m": entry["axial_offset_m"],
                    "radial_offset_m": entry["radial_offset_m"],
                    "azimuth_deg": entry["azimuth_deg"],
                },
                "A": A.tolist(),
                "X_TE": X_TE.tolist(),
                "constructing_q0_rad": q0.tolist(),
                "status": "structural_outer_reach_pass_NOT_IK",
                "maximum_target_origin_radius_m": (
                    maximum_target_origin_radius_m
                ),
            })

    missing_quotas = {
        arm: args.placements_per_arm - arm_counts[arm]
        for arm in required_arms
        if arm_counts[arm] < args.placements_per_arm
    }

    if missing_quotas:
        raise RuntimeError(
            "Arm placement quotas were not met. "
            f"required_arms={required_arms}, "
            f"placements_per_arm={args.placements_per_arm}, "
            f"arm_counts={arm_counts}, "
            f"missing_quotas={missing_quotas}, "
            f"proposal_attempts={proposal_attempts}, "
            f"rejections={rejection_counts}"
        )

    if not designs:
        raise RuntimeError(
            "No designs generated despite satisfied arm quotas; "
            "inspect placement domain and outer-reach restrictions."
        )

    out.mkdir(parents=True)

    payload = {
        "schema": "joint_proposal_pool_v3",
        "task_envelope": str(Path(args.task_envelope).resolve()),
        "task_json_sha256": sha(
            Path(args.task_envelope) / "task.json"
        ),
        "urdf_sha256": robot.urdf_sha256,
        "settings": vars(args),
        "placement_domain": placement_domain.metadata(),
        "placement_construction": {
            "formula": (
                "X_BF = FK(q0) @ A_reference @ inverse(X_FT(t0)); "
                "the reference attachment parameterizes placement sampling "
                "only and is not a preferred candidate"
            ),
            "placement_point": "femur_reference_origin",
            "placement_point_definition": (
                "Origin of the fixed femur reference frame F, expressed in "
                "the robot base frame lbr_link_0."
            ),
            "reference_attachment_id": (
                reference_entry["attachment_id"]
            ),
        },
        "attachment_catalog": {
            "attachment_count": len(catalog),
            "axial_inline_offset_m": 0.31,
            "peripheral_axial_offset_m": 0.28,
            "peripheral_radial_offset_m": 0.03,
            "peripheral_mount_count": 8,
            "peripheral_azimuth_step_deg": 45.0,
            "equal_candidate_importance": True,
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
            "Robot-informed relative placements with nine equally important "
            "discrete attachments. No full-path, collision, dynamics, "
            "hardware, or global-optimality claims."
        ),
    }

    proposal_path = out / "proposals.json"
    proposal_path.write_text(
        json.dumps(payload, indent=2, allow_nan=False)
    )

    print(json.dumps({
        "status": "robot_informed_placement_proposal_generation_PASS",
        "placements": sum(arm_counts.values()),
        "designs": len(designs),
        "attachments_per_placement": len(catalog),
        "rejections": rejection_counts,
        "end_effector_reach_bound_m": outer_bound_m,
        "proposals": str(proposal_path),
    }, indent=2))


if __name__ == "__main__":
    main()
