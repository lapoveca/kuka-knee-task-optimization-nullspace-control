# Running a new optimizer design or trajectory

This document explains how to take a completed optimizer campaign, select a design/path, export a controller-ready plan, execute it through the ROS 2 Gazebo CIC baseline runner, and extract quick tracking diagnostics.

## Required sequence

```text
campaign → reviewer atlas → controller manifest → export selected plan → verify export → 320 s baseline → quick diagnostics
```

Never run a raw campaign path directly. Always export the selected plan first.

For a new design, a direct 320 s run is usually appropriate if the export validation passes. A 70 s check remains useful for a completely new task type, a new attachment family, or after an executor change.

## Before you start

Ensure that:

- The optimizer campaign has completed.
- You know which campaign you want to use.
- Gazebo is available through the combined launch file.
- You intend to run the CIC baseline with secondary/null-space control off.
- You do not intend to change CIC gains or modify the C++ controller during baseline testing.
- You use the same reference rate, currently 50 Hz, for comparable runs.

## 1. Open the optimizer directory

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control/optimizer
```

## 2. Identify the current campaign

```bash
FULL_CAMPAIGN_DIR=$(cat output/LAST_VV_612_CAMPAIGN.txt)
echo "$FULL_CAMPAIGN_DIR"
ls "$FULL_CAMPAIGN_DIR"
```

The campaign directory should contain:

```text
campaign_manifest.json
campaign_summary.json
evaluation/
generation/
screening/
ranking_guarded/
```

## 3. Generate a reviewer atlas

The atlas must be outside the campaign directory.

Do **not** create the atlas directory yourself. The reviewer creates it.

For the top five candidates:

```bash
ATLAS_DIR="output/reviews/$(basename "$FULL_CAMPAIGN_DIR")_$(date +%Y%m%d_%H%M%S)"

python3 review_vv_campaign.py \
  "$FULL_CAMPAIGN_DIR" \
  "$ATLAS_DIR" \
  --top 5 \
  --no-gif
```

For ranks up to 50:

```bash
ATLAS_DIR="output/reviews/$(basename "$FULL_CAMPAIGN_DIR")_top50_$(date +%Y%m%d_%H%M%S)"

python3 review_vv_campaign.py \
  "$FULL_CAMPAIGN_DIR" \
  "$ATLAS_DIR" \
  --top 50 \
  --no-gif
```

Do not run this before the reviewer:

```bash
mkdir -p "$ATLAS_DIR"
```

The reviewer requires that its output directory does not already exist.

## 4. Inspect the controller manifest

```bash
less "$ATLAS_DIR/controller_test_manifest.json"
```

To print selected ranks:

```bash
python3 - <<'PY'
import json
from pathlib import Path

atlas_dir = sorted(Path("output/reviews").glob("*_top50_*"))[-1]
manifest = json.loads((atlas_dir / "controller_test_manifest.json").read_text())

for entry in manifest:
    if entry["rank"] in (4, 20, 50):
        print(
            entry["rank"],
            entry["design_id"],
            entry["arm"],
            entry["attachment_id"],
            entry["selected_path"].split("/")[-1],
        )
PY
```

Run this command from the `optimizer/` directory. If run from the repository root, use:

```bash
sorted(Path("optimizer/output/reviews").glob("*_top50_*"))
```

The manifest lists controller-ready candidates with:

- Rank
- Design ID
- Arm
- Attachment ID
- Selected path
- Path hash
- Audit report
- Collision status
- Capability metrics
- Joint-limit room
- Weighted score
- Controller-ready EE reference

## 5. Return to the repository root

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control
```

## 6. Export the selected plan

Use:

```bash
scripts/export_selected_knee_plan.py
```

Always pass an explicit `--output-dir` outside the campaign directory.

Recommended output location:

```text
optimizer/output/selected_plans/rank<NN>_<design_id>_<timestamp>
```

Example for rank 20:

```bash
ATLAS_DIR="optimizer/output/reviews/vv_612_20261119_131527_top50_20261120_155046"

EXPORT_DIR="optimizer/output/selected_plans/rank20_placement_+X_015__peripheral_05_$(date +%Y%m%d_%H%M%S)"

python3 scripts/export_selected_knee_plan.py \
  --campaign-dir "optimizer/output/vv_612_20261119_131527" \
  --manifest "$ATLAS_DIR/controller_test_manifest.json" \
  --rank 20 \
  --rate 50 \
  --output-dir "$EXPORT_DIR"
```

Do not rerun an export command after it has succeeded. Each invocation creates a new timestamped directory. If you accidentally rerun it and interrupt it, use the earlier completed export directory, not the interrupted one.

Do not rely on the exporter default. It may resolve inside the campaign directory and trigger:

```text
Export directory must not be inside the campaign directory.
```

## 7. Verify the export

```bash
EXPORT_DIR="optimizer/output/selected_plans/<your_export_directory>"

cat "$EXPORT_DIR/validation_report.json"
ls -lh "$EXPORT_DIR"
cat "$EXPORT_DIR/README.md"
```

The export is ready only if:

- `reference.npz` exists.
- `plan.json` exists.
- `validation_report.json` reports `"status": "pass"`.
- The design, path, rank, and campaign match what you intended.
- The FK start errors are near zero.

Example of a valid report:

```json
{
  "status": "pass",
  "checks": {
    "campaign_complete": true,
    "selected_path_hash": true,
    "campaign_input_hashes": true,
    "attachment_matches_inputs": true,
    "femur_placement_matches_inputs": true,
    "task_duration_matches_posture_path": true,
    "transforms_valid": true,
    "attachment_inverse_consistent": true,
    "dense_reference_consistent": true
  }
}
```

## 8. Launch Gazebo

In one terminal:

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control
source ~/ros2_ws/install/setup.bash

ros2 launch lbr_ros2_control combined_launch_file
```

Wait until Gazebo, the controller manager, JIC, and CIC are fully running.

## 9. Run the baseline

In a second terminal:

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control
source ~/ros2_ws/install/setup.bash

python3 scripts/run_selected_knee_baseline.py \
  --plan-dir "$EXPORT_DIR" \
  --duration 320
```

The runner prints:

- Selected design
- Selected path
- Task duration
- Requested duration
- Target `q_start`
- Current joint state
- JIC profile

Review these carefully before confirming JIC motion.

Press **ENTER** only if the target posture and JIC profile are intended.

After JIC completes and the robot settles, authorize CIC activation and task replay.

## 10. Quick diagnostics

After the 320 s run completes, run:

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control

python3 - <<'PY'
from pathlib import Path
import csv
import numpy as np

run_dir = sorted(
    Path("runs/selected_plan").glob("*_320s_baseline"),
    key=lambda p: p.stat().st_mtime,
)[-1]

csv_path = run_dir / "knee_task_baseline_log.csv"
print("run:", run_dir)
print("csv:", csv_path)

with csv_path.open() as f:
    rows = list(csv.DictReader(f))

print("samples:", len(rows))

def vec(rows, prefix):
    return np.array([
        [float(r[f"{prefix}_x"]), float(r[f"{prefix}_y"]), float(r[f"{prefix}_z"])]
        for r in rows
    ])

def quat(rows, prefix):
    return np.array([
        [float(r[f"{prefix}_x"]), float(r[f"{prefix}_y"]),
         float(r[f"{prefix}_z"]), float(r[f"{prefix}_w"])]
        for r in rows
    ])

ref_pos = vec(rows, "reference_ee_position")
act_pos = vec(rows, "actual_ee_position")
ref_quat = quat(rows, "reference_ee_quaternion")
act_quat = quat(rows, "actual_ee_quaternion")

ref_quat /= np.linalg.norm(ref_quat, axis=1, keepdims=True)
act_quat /= np.linalg.norm(act_quat, axis=1, keepdims=True)

dots = np.clip(np.abs(np.sum(ref_quat * act_quat, axis=1)), -1.0, 1.0)
ori_rad = 2.0 * np.arccos(dots)

pos_mm = np.linalg.norm(ref_pos - act_pos, axis=1) * 1000.0
ori_deg = np.degrees(ori_rad)

time = np.array([float(r["task_time_s"]) for r in rows])
duration = time[-1] - time
effective_rate = (len(rows) - 1) / duration

print(f"duration: {duration:.3f} s")
print(f"effective rate: {effective_rate:.2f} Hz")
print(
    "position error mm: "
    f"max={pos_mm.max():.3f}, mean={pos_mm.mean():.3f}, final={pos_mm[-1]:.3f}"
)
print(
    "orientation error deg: "
    f"max={ori_deg.max():.3f}, mean={ori_deg.mean():.3f}, final={ori_deg[-1]:.3f}"
)

pos_i = int(np.argmax(pos_mm))
ori_i = int(np.argmax(ori_deg))
print(
    "max position error at:",
    f"task_time={time[pos_i]:.3f} s, phase={rows[pos_i]['phase_name']}"
)
print(
    "max orientation error at:",
    f"task_time={time[ori_i]:.3f} s, phase={rows[ori_i]['phase_name']}"
)

fault_count = sum(
    r["fault_flags"].strip() not in ("", "[]", "none", "None")
    for r in rows
)
print(f"faults: fault_flags={fault_count}")
print("phases:", sorted(set(r["phase_name"] for r in rows)))
PY
```

This diagnostic uses `knee_task_baseline_log.csv`, not `initialization.csv`.

## 11. Record the result

For every run, save:

- Exported plan directory
- Campaign ID
- Design ID
- Arm
- Attachment ID
- Path ID
- Rank
- Run directory
- Duration
- Sample count
- Effective sample rate
- Position error: max, mean, final
- Orientation error: max, mean, final
- Faults
- JIC initialization result
- Visualization behavior
- Any stall, abort, or unexpected motion
- Phase and time of maximum errors

## Baseline rules

For these initial tests:

- Use CIC with secondary/null-space control off.
- Do not modify the C++ controller.
- Do not change gains.
- Do not enable null-space objectives.
- Use the same reference rate for comparable runs.
- Run 320 s directly after export validation passes.
- Use 70 s only for a new task type, new attachment family, or after an executor change.

## Current tested results

All runs use the same CIC baseline: secondary/null-space control off, absolute exported EE reference, 50 Hz requested reference rate, and no controller modification.

| Rank | Design | Arm | Attachment | Path | Samples | Position max/mean/final mm | Orientation max/mean/final deg | Faults | Assessment |
|---:|---|---|---|---|---:|---|---|---:|---|
| 1 | `placement_+Y_003__peripheral_02` | +Y | `peripheral_02` | `path_00` | 11,856 | 5.829 / 1.076 / 0.966 | 0.899 / 0.505 / 0.609 | 0 | Excellent |
| 2 | `placement_+Y_008__peripheral_06` | +Y | `peripheral_06` | `path_00` | 11,863 | 7.219 / 1.131 / 1.536 | 3.300 / 0.541 / 0.886 | 0 | Excellent |
| 4 | `placement_-X_016__peripheral_06` | -X | `peripheral_06` | `path_00` | 11,939 | 15.754 / 9.317 / 12.000 | 8.190 / 4.733 / 5.919 | 0 | Poor relative to 1–2 |
| 20 | `placement_+X_015__peripheral_05` | +X | `peripheral_05` | `path_02` | 11,963 | 3.210 / 1.004 / 1.141 | 0.990 / 0.445 / 0.565 | 0 | Excellent |
| 50 | `placement_+Y_007__peripheral_02` | +Y | `peripheral_02` | `path_02` | TBD | TBD | TBD | TBD | Pending |

## Current interpretation

The CIC baseline can complete multiple optimizer-selected references without faults.

However, completion without faults does **not** imply acceptable tracking. Rank 4 completed but had substantially larger position and orientation errors than ranks 1, 2, and 20.

Therefore, the appropriate current claim is:

> The current ROS 2 Gazebo CIC baseline can execute multiple optimizer-selected knee-task references, but measured tracking quality varies substantially across selected designs and paths.

Do not claim uniform acceptable performance until an explicit tracking threshold is defined and a larger, repeatable set of plans has been evaluated.

## What a completed run proves

A completed 320 s run proves that the runner can execute the exported EE reference through JIC initialization and CIC replay.

It does not prove:

- Correct optimizer joint-path execution
- Null-space benefit
- Hardware safety
- Clinical validity
- Repeatability
- Collision safety
- Torque safety

Those require separate measured evidence.

## Known exporter rule

Always use:

```bash
--output-dir optimizer/output/selected_plans/<explicit_name>
```

Do not use the exporter default without checking where it resolves.

The campaign directory must remain unchanged. The selected-plan export is a separate controller-ready package containing `plan.json`, `reference.npz`, validation data, and provenance.

## Known atlas rule

The reviewer atlas must be outside the campaign directory.

Use:

```text
optimizer/output/reviews/<campaign_name>_<timestamp>
```

Do not use:

```text
optimizer/output/<campaign_name>/atlas_<timestamp>
```

The reviewer rejects an output directory inside the campaign directory.

## Known diagnostic rule

Always analyze:

```text
runs/selected_plan/<run>/knee_task_baseline_log.csv
```

Do not analyze:

```text
runs/selected_plan/<run>/initialization.csv
```

The initialization CSV contains JIC initialization data, not EE tracking data.
