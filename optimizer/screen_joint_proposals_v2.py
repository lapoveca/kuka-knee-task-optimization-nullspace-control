#!/usr/bin/env python3
"""Pointwise IK screening for robot-informed joint proposal pools.

Uses the constructing robot configuration saved by
generate_global_placements_v3.py as the first IK seed.

This is limited pointwise screening, not connected-path validation.
A pass does not imply wrench-capability, collision-free motion, dynamic
feasibility, or a globally optimal mounting.
"""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from attachment import make_attachment_candidate
from robot_model import RobotModel
from task_envelope import TaskEnvelope, transform


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
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--max-designs", type=int, default=0)
    parser.add_argument("--seeds-per-pose", type=int, default=4)
    parser.add_argument("--max-nfev", type=int, default=120)
    parser.add_argument("--rng-seed", type=int, default=42)
    parser.add_argument("--design-seconds", type=float, default=120.0)
    args = parser.parse_args()

    if args.max_designs < 0:
        raise ValueError("max-designs must be nonnegative")
    if args.seeds_per_pose < 2:
        raise ValueError("seeds-per-pose must be at least 2")
    if args.max_nfev < 1:
        raise ValueError("max-nfev must be positive")
    if args.design_seconds <= 0.0:
        raise ValueError("design-seconds must be positive")

    proposal_path = Path(args.proposals).resolve()
    pool = json.loads(proposal_path.read_text())

    if pool.get("schema") != "joint_proposal_pool_v2":
        raise ValueError("Expected joint_proposal_pool_v2")

    task = TaskEnvelope(pool["task_envelope"])
    robot = RobotModel()

    if pool["task_json_sha256"] != sha(
        task.folder / "task.json"
    ):
        raise ValueError("Proposal task identity differs from current task")
    if pool["urdf_sha256"] != robot.urdf_sha256:
        raise ValueError("Proposal URDF identity differs from current model")

    reserved_lower = robot.position_lower + np.deg2rad(5.0)
    reserved_upper = robot.position_upper - np.deg2rad(5.0)
    if np.any(reserved_lower >= reserved_upper):
        raise ValueError("Reserved joint range is empty")

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    dependencies = [
        Path(__file__).resolve(),
        Path("task_envelope.py").resolve(),
        Path("robot_model.py").resolve(),
        Path("attachment.py").resolve(),
        Path("transforms.py").resolve(),
    ]

    identity = {
        "schema": "joint_pointwise_screen_v2",
        "proposal_hash": sha(proposal_path),
        "dependencies": {
            str(path): sha(path) for path in dependencies
        },
        "seeds_per_pose": args.seeds_per_pose,
        "max_nfev": args.max_nfev,
        "rng_seed": args.rng_seed,
        "design_seconds": args.design_seconds,
        "reserve_deg": 5.0,
    }

    manifest_path = out / "manifest.json"
    if manifest_path.exists():
        if json.loads(manifest_path.read_text()) != identity:
            raise ValueError(
                "Screen configuration changed; use a NEW output directory"
            )
    else:
        atomic_json(manifest_path, identity)

    # Select boundary, phase-endpoint and transfer-midpoint poses.
    selected_times = sorted(set(
        [float(task.time_s[0])]
        + [
            float(phase[key])
            for phase in task.phases
            for key in ("start_time_s", "end_time_s")
        ]
        + [
            0.5 * (
                phase["start_time_s"] + phase["end_time_s"]
            )
            for phase in task.phases
            if phase["kind"] == "transfer"
        ]
    ))

    selected_poses = []
    for time_s in selected_times:
        X_FT = task.pose_at(time_s)
        position_tolerance, orientation_tolerance = task.tolerances(time_s)

        matching = None
        for index, record in enumerate(selected_poses):
            if np.allclose(
                record["X_FT"], X_FT, atol=1e-9, rtol=0.0
            ):
                matching = index
                break

        if matching is None:
            selected_poses.append({
                "time_s": time_s,
                "X_FT": X_FT,
                "position_tolerance": position_tolerance,
                "orientation_tolerance": orientation_tolerance,
            })
        else:
            selected_poses[matching]["position_tolerance"] = min(
                selected_poses[matching]["position_tolerance"],
                position_tolerance,
            )
            selected_poses[matching]["orientation_tolerance"] = min(
                selected_poses[matching]["orientation_tolerance"],
                orientation_tolerance,
            )

    selected_poses.sort(key=lambda record: record["time_s"])

    def collect():
        results = []
        for design in pool["designs"]:
            checkpoint = out / f"{design['id']}.json"
            if checkpoint.exists():
                results.append(json.loads(checkpoint.read_text()))

        atomic_json(out / "summary.json", {
            "schema": "joint_pointwise_screen_summary_v2",
            "scheduled_designs": len(pool["designs"]),
            "recorded_designs": len(results),
            "screen_pose_count": len(selected_poses),
            "results": results,
            "note": (
                "Pointwise multi-start screen only; passes require complete "
                "connected path evaluation before ranking. Rejections are "
                "budgeted search outcomes, not impossibility proofs."
            ),
        })

    lock_path = out / ".screen.lock"
    try:
        lock_fd = os.open(
            lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY
        )
    except FileExistsError:
        raise RuntimeError(
            f"Screen lock exists: {lock_path}. "
            "Do not run concurrent screeners in the same output directory."
        )

    os.write(lock_fd, str(os.getpid()).encode())
    os.close(lock_fd)

    completed_designs = 0

    try:
        for design in pool["designs"]:
            checkpoint = out / f"{design['id']}.json"
            if checkpoint.exists():
                continue

            if args.max_designs and completed_designs >= args.max_designs:
                break

            completed_designs += 1
            started = time.time()
            rng = np.random.default_rng(args.rng_seed)

            rows = []
            status = "pointwise_screen_pass"
            failure = None
            previous_solution = None

            try:
                X_BF = transform(design["placement"]["X_BF"])
                attachment = design["attachment"]

                A = make_attachment_candidate(
                    roll_angle_rad=np.deg2rad(attachment["roll_deg"]),
                    pitch_angle_rad=np.deg2rad(attachment["pitch_deg"]),
                    yaw_angle_rad=np.deg2rad(attachment["yaw_deg"]),
                    tibia_offset_m=0.35,
                )

                if not np.allclose(
                    A, np.asarray(design["A"], dtype=float),
                    atol=1e-9, rtol=0.0,
                ):
                    raise ValueError(
                        "Proposal attachment reconstruction differs"
                    )
                
                constructing_seed = np.asarray(
                    design.get("constructing_q0_rad", []), dtype=float
                )
                use_constructing_seed = bool(
                    constructing_seed.shape == (7,)
                    and np.all(np.isfinite(constructing_seed))
                )
                

                for pose in selected_poses:
                    elapsed = time.time() - started
                    if elapsed > args.design_seconds:
                        status = "screen_budget_exhausted"
                        failure = {"time_s": pose["time_s"]}
                        break

                    target = (
                        X_BF
                        @ pose["X_FT"]
                        @ np.linalg.inv(A)
                    )

                    def residual(q):
                        end_effector = robot.fk_ee(q)
                        position_error = (
                            end_effector[:3, 3] - target[:3, 3]
                        )
                        orientation_error = Rotation.from_matrix(
                            target[:3, :3].T @ end_effector[:3, :3]
                        ).as_rotvec()
                        return np.concatenate([
                            position_error / 0.1,
                            orientation_error,
                        ])

                    seeds = []
                    if previous_solution is not None:
                        seeds.append(previous_solution.copy())
                    if use_constructing_seed:
                        seeds.append(constructing_seed.copy())
                    if not seeds:
                        seeds.append(
                            0.5 * (reserved_lower + reserved_upper)
                        )

                    while len(seeds) < args.seeds_per_pose:
                        seeds.append(
                            reserved_lower
                            + (
                                reserved_upper - reserved_lower
                            )
                            * rng.uniform(0.05, 0.95, 7)
                        )

                    solution = None
                    best_normalized_error = None

                    for seed in seeds:
                        fit = least_squares(
                            residual,
                            np.clip(
                                seed,
                                reserved_lower + 1e-10,
                                reserved_upper - 1e-10,
                            ),
                            bounds=(
                                reserved_lower,
                                reserved_upper,
                            ),
                            max_nfev=args.max_nfev,
                            ftol=1e-9,
                            xtol=1e-9,
                            gtol=1e-9,
                        )

                        end_effector = robot.fk_ee(fit.x)
                        position_error = float(
                            np.linalg.norm(
                                end_effector[:3, 3] - target[:3, 3]
                            )
                        )
                        orientation_error = float(
                            Rotation.from_matrix(
                                target[:3, :3].T @ end_effector[:3, :3]
                            ).magnitude()
                        )

                        normalized_error = max(
                            position_error / pose["position_tolerance"],
                            orientation_error
                            / pose["orientation_tolerance"],
                        )

                        if (
                            best_normalized_error is None
                            or normalized_error < best_normalized_error
                        ):
                            best_normalized_error = normalized_error

                        if (
                            position_error
                            <= pose["position_tolerance"]
                            and orientation_error
                            <= pose["orientation_tolerance"]
                        ):
                            solution = fit.x.copy()
                            break

                        if time.time() - started > args.design_seconds:
                            break

                    if solution is None:
                        if time.time() - started > args.design_seconds:
                            status = "screen_budget_exhausted"
                        else:
                            status = "pointwise_no_solution_with_budget"

                        failure = {
                            "time_s": pose["time_s"],
                            "best_normalized_pose_error":
                                best_normalized_error,
                        }
                        break

                    previous_solution = solution
                    rows.append({
                        "time_s": float(pose["time_s"]),
                        "q_rad": solution.tolist(),
                        "position_error_m": position_error,
                        "orientation_error_rad": orientation_error,
                    })

            except Exception as exc:
                status = "error"
                failure = {
                    "error": f"{type(exc).__name__}: {exc}",
                }

            result = {
                "schema": "joint_pointwise_screen_result_v2",
                "design_id": design["id"],
                "proposal_kind": design["proposal_kind"],
                "status": status,
                "used_constructing_seed": use_constructing_seed,
                "screened_poses": len(rows),
                "required_screen_poses": len(selected_poses),
                "failure": failure,
                "solutions": rows,
                "runtime_s": time.time() - started,
                "scope": (
                    "No connecting edges checked; no wrench capacity or "
                    "full-task feasibility claim."
                ),
            }

            atomic_json(checkpoint, result)
            collect()

            print(
                design["id"],
                status,
                f"{len(rows)}/{len(selected_poses)}",
                flush=True,
            )

        collect()
        print(
            "Invocation complete; re-run the same command to continue "
            "remaining designs. Failed screens are retained as budgeted "
            "search outcomes."
        )

    finally:
        lock_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
