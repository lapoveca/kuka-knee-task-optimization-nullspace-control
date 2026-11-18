"""Run identity, manifest, events, and outcome recording."""

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def write_json(path, content):
    path.write_text(
        json.dumps(content, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def append_event(path, event):
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, allow_nan=False) + "\n")


def create_run(root, args, env, q_start):
    run_root = root / "runs"
    run_root.mkdir(exist_ok=True)

    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    directory = run_root / f"{name}_{args.ns_config}"
    directory.mkdir()

    try:
        revision = subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        dirty = bool(subprocess.check_output(
            ["git", "-C", str(root), "status", "--porcelain"],
            text=True,
        ).strip())
    except (OSError, subprocess.CalledProcessError):
        revision = None
        dirty = None

    path_file = (
        Path(args.path_dir).expanduser().resolve()
        / "whole_trajectory_path.npz"
    )
    envelope_dir = Path(args.envelope_dir).expanduser().resolve()
    arrays_file = envelope_dir / env.meta["arrays_file"]

    manifest = {
        "schema": "knee_run_manifest_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_revision": revision,
        "git_dirty": dirty,
        "executor_sha256": file_hash(
            root / "scripts/execute_knee_task_from_optimizer.py"
        ),
        "initialization_helper_sha256": file_hash(
            root / "integration/initialization.py"
        ),
        "task_id": env.meta["task_id"],
        "task_json_sha256": file_hash(envelope_dir / "task.json"),
        "task_arrays_sha256": file_hash(arrays_file),
        "path_sha256": file_hash(path_file),
        "joint_order": [
            f"lbr_A{i}" for i in range(1, 8)
        ],
        "q_start_rad": [float(value) for value in q_start],
        "ns_profile": args.ns_config,
        "initialize_only": args.initialize_only,
        "initialization_policy": {
            "position_tolerance_rad": 0.05,
            "velocity_tolerance_rad_s": 0.05,
            "settle_duration_sim_s": 0.5,
            "maximum_sample_gap_sim_s": 0.25,
            "settling_timeout_wall_s": 10.0,
            "action_timeout_margin": 2.5,
            "action_timeout_floor_s": 5.0,
        },
        "status": "created",
    }

    write_json(directory / "manifest.json", manifest)
    append_event(directory / "events.jsonl", {
        "event": "run_created",
        "task_id": env.meta["task_id"],
        "ns_profile": args.ns_config,
    })

    return directory


def finalize_run(directory, status, outcome):
    manifest_path = directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["status"] = status
    write_json(manifest_path, manifest)

    append_event(directory / "events.jsonl", {
        "event": "run_finalized",
        "status": status,
        "outcome": outcome,
    })