#!/usr/bin/env python3
"""Resumable whole-task evaluation of paired placement-attachment designs.

Repairs v1 orchestration:
- current evaluator status names
- actual audit report path
- mean_group_min_lambda field
- configured RNG-seed collection
- promotion identity checks
- audit coverage checks
- exclusive batch lock
- timeout/config provenance

No collision, dynamics, hardware-capacity or global-optimality claims.
"""
import argparse, csv, fcntl, hashlib, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic(path, obj):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, allow_nan=False))
    tmp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", required=True)
    p.add_argument("--batch-dir", required=True)
    p.add_argument("--budget", default="configs/screening_budget_historical_v1.json")
    p.add_argument("--rng-seed", type=int, default=42)
    p.add_argument("--reserve-deg", type=float, default=5.0)
    p.add_argument("--stride-s", type=float, default=2.0)
    p.add_argument("--edge-dt", type=float, default=0.1)
    p.add_argument("--audit-dt", type=float, default=0.01)
    p.add_argument("--seeds", type=int, default=12)
    p.add_argument("--max-tracks", type=int, default=5)
    p.add_argument("--max-nfev", type=int, default=200)
    p.add_argument("--max-jobs", type=int, default=0)
    p.add_argument("--timeout-s", type=int, default=1200)
    p.add_argument("--retry-failed", action="store_true")
    a = p.parse_args()

    if a.max_jobs < 0 or a.timeout_s < 1:
        raise ValueError("Invalid job limits")
    if min(a.stride_s, a.edge_dt, a.audit_dt, a.seeds, a.max_tracks, a.max_nfev) <= 0:
        raise ValueError("Invalid numerical configuration")
    if not np.isfinite(a.reserve_deg) or a.reserve_deg < 5:
        raise ValueError("This workflow requires reserve_deg >= 5")

    manifest_path = Path(a.manifest).resolve()
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != "joint_promotion_manifest_v1":
        raise ValueError("Expected joint_promotion_manifest_v1 manifest")

    designs = manifest["designs"]
    ids = [d["id"] for d in designs]
    if not designs or len(ids) != len(set(ids)):
        raise ValueError("Empty or duplicate design IDs")
    if any(Path(i).name != i or i in (".", "..") for i in ids):
        raise ValueError("Design IDs must be simple directory names")

    base = Path.cwd()
    evaluator = base / "evaluate_task_mounts_v1.py"
    auditor = base / "audit_task_mount_run_v1.py"
    budget = Path(a.budget).resolve()
    envelope = Path(manifest["task_envelope"]).resolve()

    required = [
        evaluator, auditor, budget,
        envelope / "task.json", envelope / "task_arrays.npz",
        base / "attachment.py", base / "robot_model.py",
        base / "task_envelope.py", base / "transforms.py",
        base / "tibia_task_map.py", base / "workspace.py",
        base / "wrench_margin.py",
        base / "models" / "med14_runtime.urdf",
    ]
    required = sorted(set(x.resolve() for x in required))
    for x in required:
        if not x.exists():
            raise FileNotFoundError(x)

    task_json = envelope / "task.json"
    task_arrays = envelope / "task_arrays.npz"
    urdf = base / "models" / "med14_runtime.urdf"

    declared_task_json = manifest.get("task_json_sha256")
    declared_task_arrays = manifest.get("task_arrays_sha256")
    declared_urdf = manifest.get("urdf_sha256")
    actual_task_json = sha(task_json)
    actual_task_arrays = sha(task_arrays)
    actual_urdf = sha(urdf)

    if declared_task_json and declared_task_json != actual_task_json:
        raise ValueError("Promotion task.json identity differs from current task")
    if declared_task_arrays and declared_task_arrays != actual_task_arrays:
        raise ValueError("Promotion task_arrays.npz identity differs from current task")
    if declared_urdf and declared_urdf != actual_urdf:
        raise ValueError("Promotion URDF identity differs from current model")

    signature = {
        "schema": "joint_design_eval_batch_v2",
        "manifest_sha256": sha(manifest_path),
        "files": {str(x): sha(x) for x in required},
        "design_ids": ids,
        "rng_seed": a.rng_seed,
        "timeout_s": a.timeout_s,
        "evaluator_options": {
            "reserve_deg": a.reserve_deg,
            "stride_s": a.stride_s,
            "edge_dt": a.edge_dt,
            "audit_dt": a.audit_dt,
            "seeds": a.seeds,
            "max_tracks": a.max_tracks,
            "max_nfev": a.max_nfev,
        },
        "budget": json.loads(budget.read_text()),
        "task_json_sha256": actual_task_json,
        "task_arrays_sha256": actual_task_arrays,
        "urdf_sha256": actual_urdf,
    }

    out = Path(a.batch_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    lock_path = out / ".batch.lock"
    lock = lock_path.open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError(
            f"Another process owns {lock_path}. "
            "Use a different batch directory or stop the other process."
        )
    lock.write(
        f"pid={os.getpid()} host={os.uname().nodename} "
        f"started={time.strftime('%Y-%m-%dT%H:%M:%S')}\n"
    )
    lock.flush()

    manifest_out = out / "manifest.json"
    if manifest_out.exists():
        if json.loads(manifest_out.read_text()) != signature:
            raise ValueError(
                "Source/config/manifest/timeout changed. "
                "Use a NEW batch directory; do not mix results."
            )
    else:
        atomic(manifest_out, signature)

    def audit_is_complete(audit_dir, expected_paths, selected_path):
        report_path = audit_dir / "audit_report.json"
        if not report_path.is_file():
            return False

        audit = json.loads(report_path.read_text())
        if audit.get("status") != "PASS":
            return False
        if audit.get("expected_paths") != expected_paths:
            return False
        if audit.get("checked_paths") != expected_paths:
            return False

        results = audit.get("results", [])
        if len(results) != expected_paths:
            return False
        if not all(result.get("pass_all") is True for result in results):
            return False

        selected_path = Path(selected_path).resolve()
        selected_candidate = selected_path.parts[-2]
        selected_relative = f"{selected_candidate}/{selected_path.name}"

        audited_relative_paths = {
            result.get("path", "").replace("\\", "/")
            for result in results
        }

        return selected_relative in audited_relative_paths

    def collect():
        jobs, rows = [], []
        for design in designs:
            job_dir = out / design["id"] / f"seed_{a.rng_seed}"
            checkpoint = job_dir / "checkpoint.json"
            if not checkpoint.exists():
                continue
            job = json.loads(checkpoint.read_text())
            jobs.append(job)
            best = job.get("best", {}) if job["status"] == "audited_path_found" else {}
            rows.append({
                "design_id": design["id"],
                "proposal_kind": design["proposal_kind"],
                "status": job["status"],
                "rng_seed": job["rng_seed"],
                "best_min_lambda": best.get("min_lambda"),
                "best_mean_group_min_lambda": best.get("mean_group_min_lambda"),
                "best_extra_margin_deg": best.get("worst_track_margin_deg"),
                "best_path": best.get("path"),
                "independent_audit": job.get("independent_audit"),
                "runtime_s": job.get("runtime_s"),
            })
        atomic(out / "summary.json", {
            "scheduled_designs": len(designs),
            "recorded_designs": len(jobs),
            "jobs": jobs,
            "rows": rows,
            "scope": (
                "One whole-task evaluator run and independent audit per promoted "
                "joint design; bounded search; no global optimum claim."
            ),
        })
        if rows:
            with (out / "leaderboard.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

    count = 0
    for design in designs:
        job = out / design["id"] / f"seed_{a.rng_seed}"
        job.mkdir(parents=True, exist_ok=True)
        checkpoint = job / "checkpoint.json"

        if checkpoint.exists():
            previous = json.loads(checkpoint.read_text())
            if previous["status"] in (
                "audited_path_found", "no_path_found_with_budget"
            ):
                continue
            if not a.retry_failed:
                continue

        if a.max_jobs and count >= a.max_jobs:
            collect()
            print("Reached invocation job limit. Re-run the SAME command to continue.")
            return
        count += 1

        attempt = 1
        while (job / f"attempt_{attempt:03d}").exists():
            attempt += 1
        work = job / f"attempt_{attempt:03d}"
        work.mkdir()

        placement = dict(design["placement"])
        if placement.get("frame") != "X_BF":
            raise ValueError(f"Design {design['id']} has unexpected placement frame")

        candidate = [{
            "id": design["id"],
            "roll_deg": design["attachment"]["roll_deg"],
            "pitch_deg": design["attachment"]["pitch_deg"],
            "yaw_deg": design["attachment"]["yaw_deg"],
        }]
        atomic(work / "placement.json", placement)
        atomic(work / "candidate.json", candidate)

        result_dir = work / "result"
        audit_dir = work / "audit"
        started = time.time()
        status = {
            "design_id": design["id"],
            "proposal_kind": design["proposal_kind"],
            "rng_seed": a.rng_seed,
            "status": "error",
            "attempt": attempt,
        }

        env = os.environ.copy()
        for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
            env.setdefault(name, "1")

        def call(command, log):
            with log.open("w") as stream:
                completed = subprocess.run(
                    command, stdout=stream, stderr=subprocess.STDOUT,
                    timeout=a.timeout_s, cwd=base, env=env,
                )
            if completed.returncode:
                raise RuntimeError(
                    f"Subprocess exit {completed.returncode}; see {log}"
                )

        print("START", design["id"], flush=True)
        try:
            call([
                sys.executable, str(evaluator),
                "--task-envelope", str(envelope),
                "--placement", str(work / "placement.json"),
                "--candidates", str(work / "candidate.json"),
                "--budget", str(budget),
                "--reserve-deg", str(a.reserve_deg),
                "--stride-s", str(a.stride_s),
                "--edge-dt", str(a.edge_dt),
                "--audit-dt", str(a.audit_dt),
                "--seeds", str(a.seeds),
                "--max-tracks", str(a.max_tracks),
                "--max-nfev", str(a.max_nfev),
                "--rng-seed", str(a.rng_seed),
                "--output-dir", str(result_dir),
            ], work / "evaluator.log")

            ranking = json.loads((result_dir / "ranking.json").read_text())
            result = ranking["all_results"][0]

            if result["status"] == "sampled_path_found":
                call([
                    sys.executable, str(auditor),
                    "--run-dir", str(result_dir),
                    "--task-envelope", str(envelope),
                    "--check-dt", str(audit_dt := a.audit_dt),
                    "--output-dir", str(audit_dir),
                ], work / "auditor.log")

                expected = int(result["complete_paths"])
                selected = result["best"]["path"]
                if not audit_is_complete(audit_dir, expected, selected):
                    raise ValueError("Independent audit incomplete or does not cover selected path")

                status.update(
                    status="audited_path_found",
                    best=result["best"],
                    independent_audit=str(
                        (audit_dir / "audit_report.json").resolve()
                    ),
                )
            elif result["status"] == "no_path_found_with_budget":
                status.update(
                    status="no_path_found_with_budget",
                    failure=result.get("failure"),
                )
            else:
                raise ValueError(
                    f"Unexpected evaluator status: {result['status']}"
                )
        except subprocess.TimeoutExpired as exc:
            status.update(status="timeout", error=repr(exc))
        except Exception as exc:
            status.update(
                status="error",
                error=f"{type(exc).__name__}: {exc}",
            )

        status["runtime_s"] = time.time() - started
        atomic(checkpoint, status)
        collect()
        print("DONE", design["id"], status["status"], flush=True)

    collect()
    print("No remaining scheduled jobs "
          "(errors/timeouts require --retry-failed).")


if __name__ == "__main__":
    main()

