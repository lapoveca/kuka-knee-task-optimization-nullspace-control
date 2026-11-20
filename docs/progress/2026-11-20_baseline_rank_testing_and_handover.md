# CIC baseline rank testing and handover — 2026-11-20

## Purpose

This document records the current state of the ROS 2 Gazebo CIC baseline workflow for optimizer-selected knee-task trajectories.

It is intended as a handover document for the next work phase: finalizing executor scripts, expanding visualization, improving documentation, and only then beginning controlled tuning and null-space evaluation.

## What was tested

Five optimizer-selected VV designs and paths were exported from the current 612-design campaign and executed for the full 320 s task through the CIC baseline runner.

All runs used:

- JIC initialization to the selected optimizer `q_start`
- CIC activation
- Secondary/null-space control disabled
- Absolute exported end-effector reference
- 50 Hz requested dense-reference rate
- No modification to the C++ controller
- No gain changes
- No null-space objectives

## Results

| Rank | Design | Arm | Attachment | Path | Samples | Position max/mean/final mm | Orientation max/mean/final deg | Faults | Assessment |
|---:|---|---|---|---|---:|---|---|---:|---|
| 1 | `placement_+Y_003__peripheral_02` | +Y | `peripheral_02` | `path_00` | 11,856 | 5.829 / 1.076 / 0.966 | 0.899 / 0.505 / 0.609 | 0 | Excellent |
| 2 | `placement_+Y_008__peripheral_06` | +Y | `peripheral_06` | `path_00` | 11,863 | 7.219 / 1.131 / 1.536 | 3.300 / 0.541 / 0.886 | 0 | Excellent |
| 4 | `placement_-X_016__peripheral_06` | -X | `peripheral_06` | `path_00` | 11,939 | 15.754 / 9.317 / 12.000 | 8.190 / 4.733 / 5.919 | 0 | Poor relative to 1–2 |
| 20 | `placement_+X_015__peripheral_05` | +X | `peripheral_05` | `path_02` | 11,963 | 3.210 / 1.004 / 1.141 | 0.990 / 0.445 / 0.565 | 0 | Excellent |
| 50 | `placement_+Y_007__peripheral_02` | +Y | `peripheral_02` | `path_02` | 11,971 | 21.226 / 11.595 / 17.499 | 6.337 / 4.183 / 5.386 | 0 | Poor relative to 1, 2, 20 |

## Run directories

```text
Rank 1: runs/selected_plan/20261120T125932_423183Z_320s_baseline
Rank 2: runs/selected_plan/20261120T142906_771200Z_320s_baseline
Rank 4: runs/selected_plan/20261120T144041_788292Z_320s_baseline
Rank 20: runs/selected_plan/20261120T150042_477916Z_320s_baseline
Rank 50: runs/selected_plan/20261120T150852_147170Z_320s_baseline
```

## Main finding

The current CIC baseline can complete diverse optimizer-selected references without controller faults.

However, completion does not imply acceptable tracking. Three of the five tested plans achieved mean EE position error below 1.2 mm and mean orientation error below 0.55 deg. Two plans, ranks 4 and 50, completed but had substantially larger tracking errors.

Therefore, the appropriate current claim is:

> The current ROS 2 Gazebo CIC baseline can execute multiple optimizer-selected knee-task references, but measured tracking quality varies substantially across selected designs and paths.

It is not yet justified to claim that “most optimizer trajectories” execute to an acceptable standard. Only five ranks have been tested, no explicit acceptance threshold has been defined, and no repeated-run repeatability study has been performed.

## Important observations

### Rank 4

Rank 4 completed with no faults but had poor tracking:

- Mean position error: 9.317 mm
- Mean orientation error: 4.733 deg
- Final position error: 12.000 mm
- Final orientation error: 5.919 deg
- Maximum position error occurred during a transfer at 197.552 s
- Maximum orientation error occurred during a station at 72.291 s

### Rank 50

Rank 50 completed with no faults but also had poor tracking:

- Mean position error: 11.595 mm
- Mean orientation error: 4.183 deg
- Final position error: 17.499 mm
- Final orientation error: 5.386 deg
- Maximum position error occurred during a station at 304.195 s
- Maximum orientation error occurred during a station at 312.518 s

### Rank 20

Rank 20 was excellent:

- Mean position error: 1.004 mm
- Mean orientation error: 0.445 deg
- Maximum position error: 3.210 mm
- Maximum orientation error: 0.990 deg
- No faults

This demonstrates that rank 4’s poor result is not simply a general executor failure. The executor can achieve strong tracking on other optimizer-selected plans.

## Reference-age fix

The earlier false `stale_reference` issue was resolved by recording the actual pose-message arrival time using the executor node clock:

```python
self.actual_pose_received_time = (
    self.get_clock().now().nanoseconds * 1e-9
)
```

`measured_pose()` now returns `self.actual_pose_received_time`.

After this fix, all five full-duration runs reported:

```text
faults: fault_flags=0
```

## Visualization state

The visualization now includes:

- Homing target text: `TARGET / START / POSTURE`
- Reference path
- Actual EE trail
- EE reference and actual markers
- Tibia-origin sphere and TF
- Tibia cylinder: 0.31 m length, 0.06 m diameter
- Two lateral strips indicating internal/external rotation
- TF frames for `ref_ee`, `actual_ee`, and `tibia`

The tibia visualization is schematic. It is not a collision model, clinical model, or load-bearing component.

## Effective logging rate

All full runs logged approximately 37.1–37.4 Hz rather than the requested 50 Hz.

| Rank | Samples | Effective rate |
|---:|---:|---:|
| 1 | 11,856 | 37.05 Hz |
| 2 | 11,863 | 37.07 Hz |
| 4 | 11,939 | 37.31 Hz |
| 20 | 11,963 | 37.38 Hz |
| 50 | 11,971 | 37.41 Hz |

This is not necessarily a control failure, but it should be investigated before relying on high-frequency derivative-based diagnostics. Possible causes include loop overhead, visualization publishing, ROS callbacks, sleep timing, or logging overhead.

## Export workflow

The validated workflow is:

```text
campaign → reviewer atlas → controller manifest → export selected plan → verify export → CIC baseline → quick diagnostics
```

The exporter is:

```text
scripts/export_selected_knee_plan.py
```

The runner is:

```text
scripts/run_selected_knee_baseline.py
```

The exporter requires an explicit output directory outside the campaign directory:

```bash
--output-dir optimizer/output/selected_plans/<explicit_name>
```

The reviewer atlas must also be outside the campaign directory:

```bash
optimizer/output/reviews/<campaign_name>_<timestamp>
```

## Diagnostic workflow

Always analyze:

```text
runs/selected_plan/<run>/knee_task_baseline_log.csv
```

Do not analyze:

```text
runs/selected_plan/<run>/initialization.csv
```

The initialization CSV contains JIC initialization data, not EE tracking data.

The task log contains raw reference and actual EE poses, so the quick diagnostic computes:

- EE position error from reference and actual positions
- EE orientation error from reference and actual quaternions
- Maximum, mean, and final errors
- Task time and phase of maximum errors
- Fault count

## Next work

The next phase should proceed in this order:

1. Finalize and clean up the executor scripts.
2. Add the broader visualization suite.
3. Improve documentation and operator workflow.
4. Define explicit tracking and completion criteria.
5. Investigate why ranks 4 and 50 track poorly.
6. Investigate the approximately 37 Hz effective logging rate.
7. Repeat selected plans to establish repeatability.
8. Only then evaluate null-space control under identical baseline conditions.

## Claims that are supported

The following claim is supported by the five completed runs:

> The current ROS 2 Gazebo CIC baseline workflow can initialize, replay, and complete diverse optimizer-selected 320 s knee-task references without reported controller faults.

## Claims that are not yet supported

The following claims are not yet supported:

- Most optimizer trajectories achieve acceptable tracking.
- The optimizer joint path is being followed.
- Null-space control improves performance.
- The system is repeatable.
- The system is safe for hardware or clinical use.
- Collision, torque, or load capacity is validated.
