# Selected-plan CIC baseline: 2026-11-20

## Status

The first full-duration selected optimizer plan has completed through the current ROS 2 Jazzy / Gazebo CIC baseline workflow.

This is an important integration milestone: the runner can load an optimizer-selected plan, initialize the robot with JIC, activate CIC with secondary control disabled, replay the absolute exported EE reference, visualize the task, and record telemetry for the complete 320 s trajectory.

This does **not** yet establish clinical validity, hardware safety, null-space benefit, repeatability, or full anatomical-task accuracy. It establishes a working simulation baseline.

## Completed run

- Plan: `vv_612_20261119_131527_rank01_placement_+Y_003__peripheral_02_path_00_rev2`
- Design: `placement_+Y_003__peripheral_02`
- Exported task duration: 320.0 s
- Requested duration: 320.0 s
- Run directory: `runs/selected_plan/20261120T125932_423183Z_320s_baseline`
- Samples: 11,856
- Effective logging rate: approximately 37.05 Hz
- Secondary / null-space control: off
- Controller modification: none; the C++ controller remains unchanged

## Tracking result

| Metric | Value |
|---|---:|
| Position error, mean | 1.076 mm |
| Position error, maximum | 5.829 mm |
| Position error, final | 0.966 mm |
| Orientation error, mean | 0.505 deg |
| Orientation error, maximum | 0.899 deg |
| Orientation error, final | 0.609 deg |
| Faults | none |

### Interpretation

The mean Cartesian tracking error of 1.076 mm and mean orientation error of 0.505 deg are strong for this baseline.

The maximum position error of 5.829 mm and maximum orientation error of 0.899 deg are also small in absolute terms, but they must not be dismissed. The next analysis should identify:

- The task time at which each maximum occurs.
- Whether the maximum occurs during a station, transfer, direction reversal, or high-curvature segment.
- Whether the error is localized or recurring.
- Whether the same phase shows elevated torque, joint-limit proximity, velocity, or reference-age issues.
- Whether the optimizer’s offline predicted error and the controller’s measured error differ.

A single aggregate maximum is insufficient for a complete baseline assessment.

## Reference-age fix

The earlier 70 s run reported:

```text
faults: {'stale_reference': 2836}
```

This was a false positive caused by comparing the CIC pose message header stamp against the executor node clock.

The original implementation used:

```python
self.actual_pose.header.stamp.sec
+ 1e-9 * self.actual_pose.header.stamp.nanosec
```

The corrected implementation stores the message arrival time:

```python
self.actual_pose_received_time = (
    self.get_clock().now().nanoseconds * 1e-9
)
```

Then `measured_pose()` returns `self.actual_pose_received_time`.

After this change, the full 320 s run reports:

```text
faults: {}
```

This confirms that the previous `stale_reference` count was a timestamp-domain issue, not a real reference-freshness failure.

## Visualization changes

The visualization is now functional and has been extended.

### Homing label

The previous label was too small and visually awkward.

Current design:

```text
TARGET
START
POSTURE
```

- Text height: 0.075 m
- Position offset: 0.14 m above the final homing chain point
- Marker type: `TEXT_VIEW_FACING`
- Alpha: 1.0

### Tibia visualization

The tibia is now represented by:

- A cylinder of length 0.31 m
- A cylinder diameter of 0.06 m
- Two lateral strips to indicate internal/external rotation
- Strip length: 0.24 m
- Strip diameter: 0.008 m
- Strip offset from cylinder axis: 0.045 m

The cylinder axis is derived from the attachment transform between the EE and tibia frames.

The strips rotate with the tibia frame, so their orientation gives a visual indication of internal/external rotation.

### Important limitation

The tibia visualization is schematic.

It is not:

- A collision model
- A clinical anatomical model
- A load-bearing component
- A substitute for proper collision checking
- A substitute for measured force or torque validation

## Current execution pipeline

The runner follows this sequence:

1. Load the selected optimizer plan.
2. Validate the plan and extract the absolute EE reference.
3. Initialize the robot to the selected `q_start` using JIC.
4. Require JIC action success and measured settling.
5. Ask the operator to authorize CIC activation.
6. Activate CIC with secondary/null-space control off.
7. Stream the absolute EE reference.
8. Publish visualization markers and TF.
9. Log reference pose, actual pose, tracking error, reference age, and fault flags.
10. Complete the requested duration.
11. Write run artifacts.

The runner uses the exported absolute reference. It does not re-anchor the task to the current robot pose.

## Current telemetry

The CSV contains:

- Task time
- Reference EE pose
- Actual EE pose
- Reference and actual quaternions
- Position error
- Orientation error
- Reference age
- Fault flags
- Visualization-related state

The current report script computes:

- Position error in mm
- Orientation error in degrees
- Maximum, mean, and final error
- Fault counts

## What remains unproven

The full run is encouraging, but it does not yet prove:

- Phase-wise tracking quality.
- Repeatability across repeated runs.
- Robustness across different optimizer plans and designs.
- Correct optimizer joint-path execution.
- Null-space benefit.
- Torque safety or actuator-headroom behavior.
- Joint-limit safety during the full task.
- Velocity and acceleration quality.
- Collision safety.
- Hardware transferability.
- Clinical usability.

## Known issue to investigate

The 320 s run produced 11,856 samples over 320 s.

Expected samples at 50 Hz would be:

```text
320 * 50 = 16,000
```

Actual samples:

```text
11,856
```

Effective rate:

```text
11,856 / 320 = 37.05 Hz
```

This is not necessarily a problem, but it should be understood before relying on derivatives, timing analysis, or high-frequency diagnostics.

Possible causes include:

- Loop overhead from visualization publishing.
- ROS spin and callback overhead.
- Sleep timing not exactly matching the intended sample period.
- Blocking operations inside the replay loop.
- Logging overhead.

This should be measured later, but it should not block the next integration step.

## Next planned work

The agreed sequence is:

1. Run additional optimizer trajectories and designs to verify that the executor generalizes.
2. Make minor executor improvements.
3. Build the broader visualization suite.
4. Tune only after the visualization and baseline evaluation are complete.
5. Clean up the implementation.
6. Write the final reports.

### Immediate next execution target

Run several additional selected plans using the same 320 s duration and the same baseline configuration:

```bash
python3 scripts/run_selected_knee_baseline.py \
  --plan-dir '<other_selected_plan_dir>' \
  --duration 320
```

For each run, record:

- Run directory
- Design ID
- Path ID
- Task duration
- Sample count
- Effective sample rate
- Position error: max, mean, final
- Orientation error: max, mean, final
- Faults
- JIC initialization result
- Visualization behavior
- Any abort, stall, or unexpected motion

### Required analysis before tuning

Before changing gains or enabling null-space control, compute:

- Phase-wise position and orientation error.
- Error percentiles, not only mean and maximum.
- Time of maximum error.
- Joint-limit room over time.
- Joint velocity and acceleration.
- Commanded and measured torque.
- Secondary torque, once enabled.
- Null-space projection residual, once enabled.
- Reference freshness.
- Phase coverage.
- Repeatability across repeated runs.

## Current assessment

The current 320 s baseline is a successful full-trajectory integration result.

It shows that the pipeline can execute the selected optimizer plan with low aggregate tracking error and no faults. It is a valid baseline for beginning broader executor testing.

It is not yet a final performance claim.

