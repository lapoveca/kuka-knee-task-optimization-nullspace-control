# Task envelope setup and use

Current workspace: `knee_task_optimizer`. Status: 3 November 2026.

## 1. Purpose and boundaries

An envelope defines intrinsic anatomical motion, time, phase/groups, wrench requirements, reference frames, and tracking policy. It does NOT choose an attachment, robot placement, joint posture, robot torque budget or joint reserve. Those belong to evaluation configuration.

Current supported task schema: `kinematic_wrench_task_envelope_v1`.
- Pose: `X_FT(t)`, mapping tibia/task coordinates T into fixed femur-reference F.
- Placement: one constant `X_BF` supplied separately.
- Attachment: `A = X_ET`, one constant transform supplied separately.
- Required EE pose: `X_BE = X_BF @ X_FT @ inverse(A)`.
- Wrenches: `[Fx,Fy,Fz,Mx,My,Mz]`, expressed in T, about T origin, robot-on-tibia action.
- Current load semantics: alternative constant signed peak-capacity cases, NOT simultaneous loads or commanded force waveforms.
- Pose interpolation: linear translation plus rotation SLERP.
- Case activation: union of applicable closed phase intervals; no summation of alternatives.
- Tracking tolerance at joins: strictest applicable phase value.

No collision, dynamics, clinical safety or global optimality claim. Pure-moment requirements do not by themselves reproduce a physical torque/angle constitutive law. Missing CAD assets must not be interpreted as a validated mesh model.

## 2. Editable specification versus compiled envelope

The editable JSON contains compact station tables, source scale factors, OpenSim identifiers and development timing. The motion compiler generates poses; the finalizer verifies load axes and adds load/tolerance semantics. `TaskEnvelope` is the strict reader used by the evaluator.

These three layers must not be confused:
1. `configs/tasks/*.json`: human-editable table-based task definition.
2. `inputs/compiled_motion/TASK/`: checked OpenSim motion and axis diagnostics.
3. `inputs/task_envelopes/TASK/`: optimizer-facing pose/load bundle.

The table-driven compiler is currently specific to the three-coordinate mechanical-knee model. The evaluator uses group/phase metadata rather than task names. A future arbitrary waveform or simultaneous-load task needs a versioned compiler/reader extension; do not squeeze it into the present signed-alternative semantics.

## 3. Established task definitions

### Supervisor-scaled VV

- Flexion stations: `[0,15,30,45,60]` deg.
- Source one-sided VV magnitudes: `[2.5,5,5,6,7]` deg for both directions.
- `angle_multiplier = 2`, supervisor instruction confirmed by user.
- Compiled one-sided peaks: `[5,10,10,12,14]` deg.
- The moment remains10Nm, NOT20Nm.
- Coordinate index1 in the declared coordinate list.
- Negative/positive wrench cases: `[0,0,0,0,-10,0]`, `[0,0,0,0,10,0]`.
- Fully reproduces the historical task at saved samples.

### IE

- Same five flexion stations.
- Positive/internal magnitudes: `[10,17,20,21,19]` deg.
- Negative/external magnitudes: `[10,17,17,17,18]` deg.
- `angle_multiplier = 1`. The VV supervisor scaling does not apply to IE.
- Coordinate index2.
- Negative/positive wrench cases: `[0,0,0,-5,0,0]`, `[0,0,0,5,0,0]`.
- Positive anatomical labels follow the existing interface convention; geometric numerical checks establish axis direction but not independent clinical labeling.

90deg flexion is excluded: declared flexion upper range is approximately80deg. VV/IE coordinate bounds are approximately+/-25deg. Never clamp an unsupported target to make a task pass.

Tasks are optimized separately; the user has not requested one shared mounting solution or a transition between VV and IE trials.

## 4. Timing and ordering

Current defaults in each task spec:

```json
"timing_defaults": {
  "positive_out_s": 10,
  "positive_return_s": 10,
  "negative_out_s": 10,
  "negative_return_s": 10,
  "transfer_s": 15,
  "max_sample_dt_s": 0.5
}
```

At each station: neutral -> positive -> neutral -> negative -> neutral. Then transfer at zero VV/IE to the next flexion station. The coordinate schedule uses a quintic transition profile with zero first/second derivatives at phase endpoints. Robot joint paths generated later do not automatically inherit acceleration smoothness.

Optional station timing overrides (key must match integer station label):

```json
"station_timing_overrides": {
  "30": {
    "positive_out_s": 12,
    "negative_return_s": 8,
    "transfer_s": 20
  }
}
```

The transfer duration belongs to the station it leaves. Every duration and max sample interval must be positive. With defaults, both tasks have521 samples,24 phases,5groups,260s. Custom timing can legitimately change sample count and duration. The supplied baseline regression test includes assertions for the unchanged development tasks; it is not a universal test for changed timing.

## 5. Creating a new similar task

1. Copy an existing spec to a NEW filename and change its task ID.
2. Edit flexion station values, positive/negative source magnitudes, explicit multiplier and provenance.
3. Specify the active coordinate index and model coordinate/body names correctly.
4. Set the peak moment magnitude; angle scaling does not automatically scale load.
5. Configure timing and intentional excluded stations.
6. Confirm coordinate sign conventions. Do not infer a wrench axis from the `_rz` suffix.
7. Keep source model hash fixed, or deliberately version/update it after reviewing a new model.
8. Compile motion and inspect report, endpoints, phase joins and axis probes.
9. Finalize only after full-motion moment-axis validation.
10. Load the completed envelope and run regression/preflight checks before optimization.

Changing the coordinate list, load semantics, spatial wrench frame or origin requires corresponding validation. The compiler must not silently edit model limits or omit stations other than intentional configured exclusions.

## 6. Build commands for the current tasks

Run from the workspace root. Use new output directories; compilers refuse overwriting.

```bash
python3 compile_knee_motion_v1.py \
  --spec configs/tasks/vv_supervisor_scaled_v1.json \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --legacy-vv-dir inputs/legacy_vv \
  --output-dir inputs/compiled_motion/vv_supervisor_scaled_v1

python3 finalize_knee_task_envelope_v1.py \
  --motion-dir inputs/compiled_motion/vv_supervisor_scaled_v1 \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --moment-axis y \
  --legacy-vv-dir inputs/legacy_vv \
  --output-dir inputs/task_envelopes/vv_supervisor_scaled_v1
```

For IE:

```bash
python3 compile_knee_motion_v1.py \
  --spec configs/tasks/ie_table_v1.json \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --output-dir inputs/compiled_motion/ie_table_v1

python3 finalize_knee_task_envelope_v1.py \
  --motion-dir inputs/compiled_motion/ie_table_v1 \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --moment-axis x \
  --output-dir inputs/task_envelopes/ie_table_v1
```

These commands document the completed build. Do NOT rerun against existing folders; no regeneration is needed unless the task changes.

## 7. Compiled bundle contents

`task.json`: schema/frame/load/interpolation conventions; phase intervals; objective groups; position/orientation tolerances; source spec; model and payload hashes.

`task_arrays.npz`: `time_s`, `X_FT`, `angles_rad`, `wrench_cases_T`, `active_cases`, `X_GF_reference`.

`validation_report.json`: model-motion reproduction,521-posture axis validation, legacy regression if applicable; explicitly does not assert robot-path feasibility.

`task_validation.png`: coordinate schedule, case masks and axis-deviation diagnostic. Case-mask plots are not moment command plots.

At the positive-return/negative-out boundary both cases apply as alternatives. There are5such samples in the current tasks. Transfer interiors have no assessment capacity case, although this does not mean zero physical gravitational/inertial loading.

## 8. Reader API

```python
from task_envelope import TaskEnvelope

task = TaskEnvelope("inputs/task_envelopes/ie_table_v1")
X_FT = task.pose_at(t)
X_BE = task.required_ee(t, X_BF, A)
indices = task.case_indices(t)
groups = task.groups_at(t)
position_tol_m, orientation_tol_rad = task.tolerances(t)
planning_times = task.planning_times(2.0)
```

No extrapolation outside task time is allowed. Wrenches are obtained as `task.wrenches[index]`. A finite set of case vectors is not a continuous wrench distribution. Group names are identifiers, not solver instructions.

## 9. Baseline interface regression

```bash
python3 verify_task_envelope_interface.py \
  --vv inputs/task_envelopes/vv_supervisor_scaled_v1 \
  --ie inputs/task_envelopes/ie_table_v1 \
  --legacy inputs/legacy_vv \
  --reference-path inputs/reference_paths/pitch90_stride2_historical.npz \
  --budget configs/screening_budget_historical_v1.json \
  --output-dir output/task_envelope_interface_regression_v1
```

Already demonstrated by user: status PASS,1041 target comparison times, max matrix difference1.6156e-27, fixed reference min lambda4.015329051631654, reference hard joint margin9.171280217628958deg. Tests include activation union, transfer interiors, strict shared tolerances, no extrapolation, payload corruption rejection, fractional-time target comparison and fixed-path wrench regression.

The resulting `reference_placement_X_BF.json` is an explicit baseline placement. Its matrix is `historical_X_BG @ X_GF_reference`. It is not a constraint on later placement optimization.

Historical path generation used3deg reserve; unchanged path is now directly shown to exceed5deg. Do not relabel its historical generation provenance. New path searches default to5deg and reject settings below5deg in the new integration evaluator.

## 10. New evaluator integration (runtime validation pending)

New files: `evaluate_task_mounts_v1.py`, `audit_task_mount_run_v1.py`. They read the TaskEnvelope API and explicit placement JSON with `frame: X_BF` and `X_BF: 4x4`.

- Attachment candidates remain JSON list of id/roll_deg/pitch_deg/yaw_deg.
- Fixed0.35m geometry/cone remain in attachment.py.
- Explicit budget JSON remains `[200,200,176,176,110,40,40]`Nm, zero bias.
- Separate VV and IE runs.
- Search: multiple initial seeds, parent-warm-start continuation, sampled verified edges, bounded histories; no global path optimality.
- Reports use `group_capability` and `mean_group_min_lambda`, not legacy flexion-specific fields.
- Final path reports remain sampled kinematic acceptance, not collision/dynamic/clinical certification.
- Wrench capability is evaluated at active saved task samples. Dense tracking audit does not guarantee dense capacity minima.

Saved NPZ includes X_BF and derived X_BG for provenance. Do not assume the old VV visualizer can render IE correctly: it defaults to legacy VV target files and needs a new envelope-aware adapter before use on new IE paths.

## 11. Future extensions not implemented by this schema

Simultaneous load combinations, time-varying wrench cases, other wrench frames/origins, changing attachment length, collision constraints, gravity-aware bias, acceleration-smooth robot paths and global search certification. Implement and test new semantics explicitly rather than editing array keys to bypass the reader.

The next stage is direct-placement integration and small VV/IE fixed-placement runs. Joint6DOF anatomical-placement plus attachment optimization follows only after their separate audits pass.
