# Task envelope setup and use

## Purpose

A task envelope defines the anatomical motion, timing, wrench requirements,
reference frames, and tracking policy for a knee-assessment task.

It does not define robot placement, attachment, joint posture, torque budget,
or controller gains.

## Task identity conventions

- Motion specifications live in `configs/tasks/`.
- Compiled motion bundles live in `inputs/compiled_motion/`.
- Final task envelopes live in `inputs/task_envelopes/`.
- The envelope `task_id` identifies the finalized kinematic-and-loading task.
- The motion-specification `task_id` identifies the authored anatomical motion.
- `source_spec` records the motion specification used to generate an envelope.

## Supported schema

Current schema: `kinematic_wrench_task_envelope_v1`.

- Pose: `X_FT(t)` maps tibia/task coordinates `T` into the fixed femur
  reference `F`.
- Placement: one constant `X_BF` is supplied separately.
- Attachment: `A = X_ET` is one constant transform.
- Required end-effector pose: `X_BE = X_BF X_FT A^{-1}`.
- Wrenches are expressed in `T`, about the `T` origin, with robot-on-tibia
  action.
- Pose interpolation uses linear translation and rotation SLERP.
- Wrench cases are alternative signed peak-capacity cases, not simultaneous
  loads or commanded force waveforms.
- Case activation is the union of applicable closed phase intervals;
  alternatives are not summed.
- Tracking tolerance at phase joins uses the strictest applicable phase value.

No collision, dynamics, clinical-safety, or global-optimality claim is implied
by an envelope.

## Task definitions

### Varus-valgus distal force

- Motion specification: `configs/tasks/varus_valgus_motion.json`
- Compiled motion: `inputs/compiled_motion/varus_valgus_motion/`
- Task envelope: `inputs/task_envelopes/varus_valgus_distal_force/`
- Flexion stations: 0, 15, 30, 45, 60 degrees
- Active coordinate: varus-valgus
- Required peak moment: 10 Nm
- Positive wrench case: `[0, 0, 0, 0, 10, 0]`
- Negative wrench case: `[0, 0, 0, 0, -10, 0]`
- 90-degree flexion is excluded because it is outside the model flexion range.

### Internal-external rotation

- Motion specification:
  `configs/tasks/internal_external_rotation_motion.json`
- Compiled motion:
  `inputs/compiled_motion/internal_external_rotation_motion/`
- Task envelope: `inputs/task_envelopes/internal_external_rotation/`
- Flexion stations: 0, 15, 30, 45, 60 degrees
- Active coordinate: internal-external rotation
- Required peak moment: 5 Nm
- Positive wrench case: `[0, 0, 0, 5, 0, 0]`
- Negative wrench case: `[0, 0, 0, -5, 0, 0]`
- 90-degree flexion is excluded because it is outside the model flexion range.

## Timing and station order

Default timing:

```json
{
  "positive_out_s": 10,
  "positive_return_s": 10,
  "negative_out_s": 10,
  "negative_return_s": 10,
  "transfer_s": 15,
  "max_sample_dt_s": 0.5
}
```

At each flexion station, the motion sequence is:

```text
neutral -> positive -> neutral -> negative -> neutral
```

A zero-coordinate transfer then moves to the next flexion station. Transitions
use a quintic profile with zero first and second derivatives at phase endpoints.

Optional station timing overrides use the integer station label as the key:

```json
"station_timing_overrides": {
  "30": {
    "positive_out_s": 12,
    "negative_return_s": 8,
    "transfer_s": 20
  }
}
```

The transfer duration belongs to the station being left. All durations and the
maximum sample interval must be positive.

## Creating or modifying a task

1. Copy an existing motion specification to a new file and change its
   `task_id`.
2. Edit flexion stations, positive/negative source magnitudes, the angle
   multiplier, and the active coordinate index.
3. Set the required peak moment. Angle scaling does not automatically scale
   load.
4. Configure timing and intentional station exclusions.
5. Confirm coordinate sign conventions. Do not infer a wrench axis from the
   `_rz` suffix.
6. Compile and validate the motion before finalizing an envelope.
7. Load the completed envelope and run regression checks before optimization.

Changing the coordinate list, load semantics, spatial wrench frame, or wrench
origin requires corresponding validation. Do not silently edit model limits or
omit stations other than intentional configured exclusions.

## Envelope contents

`task.json` contains the schema, frames, load conventions, interpolation rules,
phase intervals, objective groups, tolerances, source-specification metadata,
and integrity hashes.

`task_arrays.npz` contains:

- `time_s`
- `X_FT`
- `angles_rad`
- `wrench_cases_T`
- `active_cases`
- `X_GF_reference`

Validation reports record model-motion reproduction, axis validation, and
applicable regression checks. They do not assert robot-path feasibility.

At the positive-return/negative-out boundary, both wrench cases apply as
alternatives. There are five such samples in the current tasks. Transfer
interiors have no assessment-capacity case, although this does not imply zero
physical gravitational or inertial loading.

## Reader API

```python
from task_envelope import TaskEnvelope

task = TaskEnvelope("inputs/task_envelopes/varus_valgus_distal_force")
X_FT = task.pose_at(t)
X_BE = task.required_ee(t, X_BF, A)
indices = task.case_indices(t)
groups = task.groups_at(t)
position_tol_m, orientation_tol_rad = task.tolerances(t)
planning_times = task.planning_times(2.0)
```

No extrapolation outside the task time range is allowed. Wrenches are available
through `task.wrenches[index]`. A finite set of wrench cases is not a
continuous wrench distribution. Group names are identifiers, not solver
instructions.

## Validation policy

Before using a modified envelope:

1. Validate the motion specification.
2. Compile the motion and inspect endpoints, phase joins, and axis probes.
3. Finalize the envelope only after moment-axis validation.
4. Load the envelope with `TaskEnvelope`.
5. Run the applicable interface regression and preflight checks.

## Extension policy

Simultaneous load combinations, time-varying wrench cases, other wrench frames
or origins, changing attachment length, collision constraints, gravity-aware
bias, acceleration-smooth robot paths, and global search certification are not
implemented by the current schema.

Implement and test new semantics explicitly rather than editing array keys to
bypass the reader.
