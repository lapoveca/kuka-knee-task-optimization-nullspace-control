# BARR HPM KUKA Project — Implementation Handoff

**Date:** 19 November 2026  
**Purpose:** Enable a separate chat/session to run selected optimizer campaign trajectories through the existing ROS 2 / Gazebo **Cartesian Impedance Controller (CIC)** workflow, then evaluate baseline CIC behavior before introducing or tuning null-space control.

## 1. Project context

This is the BARR HPM KUKA knee-rehabilitation project at TU Delft. The user’s control contribution is a torque-based Cartesian impedance controller with null-space optimization in Gazebo for the KUKA LBR Med 14, built on ROS 2 Jazzy and the `lbr_fri_ros2_stack`.

The project combines:

- A **612-design VV campaign optimizer** that selects robot base placements, tibia attachments, and joint-space paths.
- A validated **selected VV plan** with an absolute EE/tibia task reference and saved posture path.
- A preserved **ROS 2 Jazzy KUKA LBR Med 14 controller stack** with JIC and CIC.
- A Gazebo simulation workflow that initializes the robot to the selected plan’s start posture, switches to CIC, and streams the selected Cartesian task.
- A planned null-space-controller evaluation, initially comparing secondary-off baseline behavior against existing local NS objectives.

The immediate objective is **not** to rerun optimization. It is to demonstrate that the controller can accept and execute selected optimizer campaign tasks in Gazebo, then use that baseline to evaluate null-space control.

## 2. Current state

### Completed

- The 612-design VV campaign smoke run completed successfully, including generation, screening, promotion, evaluation, and recovery behavior.
- A 36-design campaign smoke run completed successfully and validated the campaign workflow.
- The full 612-design VV campaign completed and produced a completed, ranked campaign directory.
- The reviewer was expanded to produce fixed-axis GIFs, capability extrema, cross-design comparisons, ranking sensitivity, richer profiles, and a controller-test manifest.
- The selected VV plan was validated and exported with provenance, task data, transforms, posture path, and numerical validation results.
- Offline/runtime kinematic agreement was verified at floating-point level for the relevant chain.
- The combined Gazebo launch starts with JIC active, CIC inactive, and the joint-state broadcaster active.
- A JIC initialization attempt reached the selected start posture but returned an action abort; the cause was not conclusively classified.

### Outstanding

- A successful, measured JIC initialization and CIC static-hold gate.
- A short original-timing CIC baseline segment.
- Full-duration CIC baseline replay.
- Null-space comparison under identical conditions.
- A polished, reproducible release workflow and final handoff.

## 3. Core architecture

### Optimizer side

The optimizer generates and evaluates robot placements, attachments, and joint-space paths for a prescribed rehabilitation task. It does not directly control the robot.

The verified runtime chain is:

1. `generate_global_placements_v3.py`
2. `screen_joint_proposals_v2.py`
3. `promote_screened_designs_v1.py`
4. `run_joint_design_eval_v2.py`
5. `evaluate_task_mounts_v1.py`
6. `audit_task_mount_run_v1.py`
7. `score_joint_design_paths_v1.py`
8. `build_supervisor_review_v2.py`

The runner requires the optimizer’s attachment, robot-model, task-envelope, transform, tibia-task-map, workspace, wrench-margin modules, task JSON/NPZ, budget, and runtime URDF.

### Controller side

The controller source is in the ROS workspace, under `lbr_impedance_control`. It builds:

- `joint_impedance_controller.cpp`
- `cartesian_impedance_controller.cpp`

into `lbr_impedance_control_controllers`.

The intended execution sequence is:

```text
Gazebo launch
  -> JIC active, CIC inactive
  -> JIC initializes robot to selected q_start
  -> measured start-state verification
  -> CIC activation with secondary control off
  -> static hold verification
  -> CIC receives selected Cartesian task reference
  -> baseline task replay
  -> null-space comparison
```

The controller must not be modified merely to replay the selected plan. Controller extension is separate from baseline replay.

## 4. Frame and transform convention

The selected plan uses explicit transforms. Do not infer them from the robot’s current pose.

| Symbol | Meaning |
|---|---|
| `X_BF` | Femur placement transform, robot base to femur |
| `A = X_ET` | Attachment transform, end-effector to tibia |
| `X_TE` | Inverse attachment transform, tibia to end-effector |
| `X_FT(t)` | Task pose at time `t`, femur to tibia |
| `X_BT(t)` | Requested tibia pose in base frame |
| `X_BE(t)` | Required EE pose in base frame |

The required EE reference is:

```text
X_BE(t) = X_BF * X_FT(t) * inverse(A)
```

The realized tibia pose from measured joint state is:

```text
X_BT_actual(t) = FK(q(t)) * A
```

The selected plan explicitly stores `X_BF`, `A`, `X_TE`, task poses, joint reference, and start posture.

The historical `knee_mount_config.py` contained an improper rotation with determinant `-1`, so it must not be used as the selected-plan attachment source. The optimizer’s audited attachment transform must be used instead.

## 5. Selected VV plan

The previously validated selected VV result was:

```text
Design: global_robotinformed_p000_a09
Seed: 42
Attempt: 001
Path: path_03
Canonical ranking: VV/guarded_review_v1/design_ranking.csv
```

Its offline numerical validation produced:

| Metric | Value |
|---|---:|
| Tracking samples | 26,001 |
| Sampled violations | 0 |
| Maximum EE position error | 4.081314855 mm |
| Maximum EE orientation error | 0.00870158346 rad, about 0.499 deg |
| Maximum tibia-origin error | 2.700924728 mm |
| Maximum tracking-error/tolerance ratio | 0.2492820037 |
| Minimum actual-limit room | 19.6164711656 deg |
| Extra room beyond 5-degree reserve | 14.6164711656 deg |
| Maximum segment velocity utilization | 0.07714466662, about 7.71% |
| Active-case entries | 410 |
| Loaded duration | 200 s |
| Minimum capability | 27.25079097576053 |
| Active-time mean capability | 30.389816236736223 |
| Guarded score | 1.6530305431843657 |

These are offline sampled kinematic/model checks, not controller tracking measurements or physical-load validation.

## 6. Current 612 campaign

The current campaign is the newer **612-design VV campaign**, not necessarily the earlier selected `global_robotinformed_p000_a09` plan.

The campaign directory is identified by:

```bash
FULL_CAMPAIGN_DIR="$(cat output/LAST_VV_612_CAMPAIGN.txt)"
```

The campaign contains:

```text
campaign_manifest.json
campaign_summary.json
generation/
screening/
promotion/
evaluation/
collision_checks.json
ranking_guarded/
```

The reviewer validates the campaign structure, hashes, ranking consistency, collision coverage, eligibility, and selected trajectory identity before generating plots.

The reviewer now produces:

| Output | Purpose |
|---|---|
| `01_stage_yield` | Campaign and path funnel |
| `02_scores` | Weighted-score decomposition |
| `03_tradeoff` | Capability versus joint room |
| `04_placement_coverage` | Placement coverage and stage progression |
| `05_attachment_yield` | Arm and attachment retention |
| `06_selected_path_profiles` | Capability, joint room, and velocity profiles |
| `07_collision_rejections` | Collision-rejection overview and CSV |
| `08_*_scenes` | Full-robot and interface 3D scenes |
| `09_*_errors` | Joint, EE, and tibia tracking-error plots |
| `10_*_motion.gif` | Four-view animation with fixed full-robot axes |
| `11_best_worst_capability_postures` | Rank 1 and rank 5 capability extrema |
| `12_cross_design_bottlenecks` | Cross-design bottleneck comparison |
| `13_ranking_sensitivity` | Alternative-policy ranking sensitivity |
| `14_limiting_joints` | Limiting-joint frequency |
| `15_per_joint_room` | Per-joint room profiles |
| `16_ee_error_and_velocity` | EE error and velocity utilization |
| `controller_test_manifest.json/.csv` | Controller-ready selected-design manifest |

The controller manifest identifies the selected `whole_trajectory_path.npz` as the controller-ready EE reference. The CIC executor should derive the EE reference from that file; it should not command the saved joint trajectory directly.

## 7. Reviewer fixes applied

The reviewer had three runtime issues that were fixed:

1. **World-frame mesh transform error**  
   `robot_geometry()` already returns transformed mesh triangles. The fixed bounding-box calculation incorrectly attempted another transform:

   ```python
   move_points(mesh, pose.homogeneous)
   ```

   The corrected expression is:

   ```python
   mesh.reshape(-1, 3)
   ```

2. **String path used as `Path`**  
   In the limiting-joint plot:

   ```python
   path_directory = review["ranked"]["path"]
   ```

   was corrected to:

   ```python
   path_directory = Path(review["ranked"]["path"])
   ```

3. **Missing `room` in trajectory review**  
   The trajectory-review dictionary did not include the per-joint room array. It was corrected by adding:

   ```python
   "room": result["room"],
   ```

The GIF behavior was also corrected:

- The two whole-robot GIF panels use one fixed world bounding box.
- The robot base and axes remain fixed throughout the animation.
- The two interface close-up panels retain adaptive framing.
- The fixed bounding box includes base origin, femur placement, requested/actual tibia poses, actual/required EE poses, and robot meshes at start, middle, and end configurations.

## 8. Gazebo execution contract

### Simulation assumptions

The current Gazebo workflow is a **simulation-only** test environment with gravity disabled. That is a declared test condition, not a demonstration of hardware gravity compensation, payload/contact dynamics, or physical loading.

The launch uses a no-gravity world, `lbr_system_config.yaml`, `joint_limits.yaml`, the LBR ROS 2 control xacro, and initial positions. The exact referenced resources must be verified from the installed environment before a clean-checkout demonstration.

The controller manager and both JIC and CIC report `use_sim_time: true`. The installed simulation controller configuration is:

```text
lbr_ros2_control/config/controllers/gazebo.yaml
```

This resolves the earlier ambiguity about the unrelated hardware YAML.

### Initialization

The selected plan must initialize the robot to its own `q_start`, not the old fixed home posture:

```text
[0, 0.75, 0, -1.4, 0, -1.257, 0] rad
```

The JIC implementation interpolates joint references linearly and checks measured goal tolerance. It is not collision-aware.

The initialization workflow should:

1. Verify plan hashes and model identity.
2. Verify live controller state and simulation time.
3. Verify fresh joint state and stationary hold.
4. Confirm the current posture matches the reviewed approach start.
5. Execute JIC initialization only after explicit operator confirmation.
6. Require action success.
7. Require every measured joint error at most 0.01 rad.
8. Require every measured joint speed at most 0.01 rad/s continuously for at least one simulation second.
9. Record joint state and a machine-readable initialization report.

A successful JIC action alone is not sufficient to activate CIC.

### CIC baseline

After initialization:

1. Keep JIC active until measured start criteria pass.
2. Switch to CIC.
3. Use secondary control off.
4. Verify a stable static hold.
5. Stream the selected task reference using elapsed simulation time.
6. Execute a short baseline segment first.
7. Extend to the full task only after the short segment passes.

The selected VV task spans 0–260 seconds, with five 40-second assessment stations and four 15-second transfers. The loaded duration is 200 seconds and transfer duration is 60 seconds.

Station tolerances are 20 mm and 2 degrees. Transfer tolerances are 30 mm and 3 degrees. These are offline evaluation tolerances, not automatically the desired measured-control target.

## 9. What CIC must receive

The CIC executor should receive:

- Task time from simulation time.
- Required EE pose `X_BE(t)`.
- Required EE twist, if required by the controller interface.
- The selected attachment transform `A`.
- The selected femur placement `X_BF`.
- The original task phase structure.
- The selected posture path only if explicitly implementing optimizer-guided posture assistance.

It must not:

- Re-anchor the task to the current robot pose.
- Replace the selected placement with a newly inferred one.
- Use the historical flexion-only task generator.
- Treat the optimizer’s joint path as the primary Cartesian reference.
- Automatically command knee load from capability cases.

The old executor re-anchored the task to the current EE pose by default. That behavior is useful for a locally anchored controller experiment but is not replay of an optimized fixed placement.

## 10. Null-space control plan

The null-space controller should be evaluated only after a stable CIC baseline exists.

### Baseline

The baseline is:

```text
CIC active
Secondary null-space objective off
Same selected task
Same q_start
Same timing
Same primary gains
```

### First comparison

The first comparison should use existing local null-space objectives, especially joint-limit avoidance.

The prior implementation review found that the joint-limit objective cost and gradient were correct, but the damped pseudo-inverse made the null-space projector inexact, allowing null-space torque to leak into the task space.

The key diagnostic is:

```text
|| J M^-1 N ||_F ~ 0
```

If this residual is not small, the projector is leaking null-space torque into the task space, or the inertia matrix used in the projection is inconsistent.

### Metrics to record

For each run, record:

- EE position and orientation error.
- Tibia position and orientation error.
- Measured joint positions and velocities.
- Reference versus actual posture deviation.
- Joint room and limiting joint/time.
- Commanded and measured torque.
- Secondary torque magnitude.
- Null-space projection residual.
- Task tracking error.
- Phase coverage.
- Reference freshness and timing.
- Fault and saturation events.

The final integrated claim should distinguish:

1. **Local NS comparison:** existing objectives improve or change realized posture on the selected task.
2. **Optimizer-guided posture execution:** a synchronized secondary posture reference follows the selected offline plan.

These are different claims. Do not call full joint-path replay null-space control.

## 11. Execution sequence

### 1. Locate campaign

```bash
cd ~/ros2_ws/src/kuka-knee-task-optimization-nullspace-control/optimizer

FULL_CAMPAIGN_DIR="$(cat output/LAST_VV_612_CAMPAIGN.txt)"
echo "$FULL_CAMPAIGN_DIR"
cat "$FULL_CAMPAIGN_DIR/campaign_summary.json"
```

### 2. Generate reviewer atlas

```bash
ATLAS_DIR="${FULL_CAMPAIGN_DIR%/}_atlas_$(date +%Y%m%d_%H%M%S)"

python3 review_vv_campaign.py \
  "$FULL_CAMPAIGN_DIR" \
  "$ATLAS_DIR" \
  --top 5 \
  --no-gif
```

### 3. Inspect controller manifest

```bash
cat "$ATLAS_DIR/controller_test_manifest.json"
```

The selected design should be identified by rank, design ID, arm, attachment ID, selected path, hashes, audit report, capability metrics, joint room, and controller-ready EE reference.

### 4. Export selected plan

Use the selected path from the controller manifest, not an older campaign’s selected design.

The export must include:

```text
plan.json
reference.npz
validation_report.json
budget.json
score_policy.json
README.md
task/task.json
task/task_arrays.npz
models/med14_runtime.urdf
provenance/
```

The exported `reference.npz` contains:

- `time_q_s`
- `q_ref_rad`
- `qdot_segment_rad_s`
- `time_task_s`
- `X_BE_task`
- `X_BT_task`
- `A_X_ET`
- `X_TE`
- `X_BF`
- `q_start_rad`
- required and FK start poses

The primary task and posture grids are independent. Playback must interpolate each by the same task time, not use one shared sample index.

### 5. Launch Gazebo

Use the existing combined launch. Do not use the old homing script or old executor as the primary workflow.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lbr_ros2_control <combined_launch_file>
```

The exact launch file and package prefix must be confirmed from the installed environment before execution.

### 6. Initialize with JIC

Use the selected plan’s `q_start`.

```bash
python3 initialize_selected_knee_plan.py \
  --plan <selected_plan_dir>/plan.json \
  --dry-run
```

Only after dry-run success and operator review:

```bash
python3 initialize_selected_knee_plan.py \
  --plan <selected_plan_dir>/plan.json \
  --execute \
  --confirm-reviewed-approach
```

The JIC profile should remain:

```text
Global stiffness: 100 Nm/rad
Damping ratio: 0.7
Separate-joint mode: off
Reference speed: 0.10 rad/s
```

A different profile should stop preflight rather than be silently overwritten.

### 7. Switch to CIC

Only after measured initialization criteria pass:

```bash
ros2 control set_controller_state \
  joint_impedance_controller inactive

ros2 control set_controller_state \
  cartesian_impedance_controller active
```

The exact controller names, lifecycle service names, and namespace must be confirmed from the running system before issuing commands.

### 8. Static hold

Before streaming the task:

- Verify CIC is active.
- Verify secondary control is off.
- Verify measured joint state is stable.
- Verify EE pose matches the required start pose within the declared threshold.
- Verify simulation time advances.
- Record a static-hold report.

### 9. Short baseline segment

Run the first assessment plus transfer using original timing:

```text
0 to 55 seconds
```

This covers the first positive excursion and first transfer.

Do not run the full 260-second task until the short segment passes.

### 10. Full baseline replay

After the short segment passes:

```text
0 to 260 seconds
```

Record complete reference and actual state, including phase identity and timestamps.

### 11. Null-space comparison

Only after baseline replay succeeds:

```text
Baseline: secondary off
Comparison: selected existing NS objective
Same task, timing, q_start, and primary gains
```

Do not enable untested objective combinations or high-gain demonstration presets.

## 12. Required telemetry

The executor must record:

```text
time
simulation_time
phase_id
phase_name
reference_ee_pose
reference_ee_twist
actual_ee_pose
actual_tibia_pose
measured_q
measured_qdot
reference_q
reference_qdot
joint_room
limiting_joint
commanded_torque
measured_torque
secondary_torque
null_space_projection_residual
reference_age
fault_flags
```

The old executor CSV did not record full measured joint state, measured orientation, phase identity, posture reference, plan identity, or timestamp alignment. That is insufficient for realized-posture performance evaluation.

## 13. Success criteria

### Initialization

```text
JIC action success
Every measured joint error <= 0.01 rad
Every measured joint speed <= 0.01 rad/s
Stable for at least 1 simulation second
```

### CIC static hold

```text
CIC active
Secondary off
No unexpected motion
Required start pose achieved
Reference and state fresh
Simulation time advancing
```

### Short baseline

```text
Task reference streamed correctly
No controller fault
No unbounded wait
No stale reference
No invalid state
Phase coverage correct
Tracking error within declared test threshold
```

### Full baseline

```text
Full 0-260 s task completed
No unhandled fault
Complete telemetry recorded
Automatic tracking/limits report generated
```

### Null-space comparison

```text
Same baseline conditions
Explicit NS mode
Measured posture difference
No unjustified claim of optimizer-path tracking
Projection residual reported
```

## 14. Known limitations

- Capability is a nominal task-wrench model prediction, not a commanded load or measured motor/patient capacity.
- Collision checking is sampled, not continuous swept-volume certification.
- Joint clearance is favorable under the offline model but does not establish physical clearance to tibia, table, floor, fixtures, or self-collision.
- Low velocity utilization does not certify acceleration, torque transients, stability, or smooth finite-rate streaming.
- The offline joint path already has approximately 4.08 mm EE approximation error. It passed declared tolerances but does not realize the anatomical task to numerical precision.
- Controller-realized joint state can differ from `q_ref` even with good Cartesian tracking. Capability and clearance must be recomputed along measured motion.
- The initialization approach is not covered by the full-task audit.
- The JIC initialization abort has not been conclusively diagnosed.
- No clinical, collision-certification, global-optimality, or measured-load-capacity claim is supported.

## 15. Immediate next actions

The next chat should proceed in this order:

1. Confirm the current 612 campaign directory and inspect `campaign_summary.json`.
2. Run the reviewer and inspect `controller_test_manifest.json`.
3. Export the selected plan from the current campaign.
4. Launch the existing combined Gazebo workflow.
5. Run the JIC initialization dry-run.
6. Resolve the prior JIC abort only with measured evidence.
7. Initialize to the selected `q_start`.
8. Switch to CIC with secondary off.
9. Verify static hold.
10. Run the 0–55 second baseline segment.
11. Run the full 0–260 second baseline replay.
12. Compare existing null-space modes under identical conditions.
13. Only then consider optimizer-guided posture-reference assistance.

The first integration target is simulation, with declared gravity/contact assumptions. The working C++ controller remains unchanged until baseline replay is established.

## 16. Handoff instruction for next chat

Use this exact instruction:

```text
I am continuing the BARR HPM KUKA project. The optimizer campaign is complete. I want to run selected campaign trajectories through the existing ROS 2 / Gazebo CIC workflow.

Do not rerun optimization. Do not modify the campaign artifacts. Do not modify the C++ controller yet. Use the selected plan from the current 612-design VV campaign, identified by output/LAST_VV_612_CAMPAIGN.txt.

First inspect the campaign summary, run the reviewer, and inspect controller_test_manifest.json. Then export the selected plan. Launch the existing combined Gazebo workflow. Initialize the robot to the selected q_start using JIC. Verify measured joint state and velocity. Switch to CIC with secondary control off. Verify static hold. Then run a short original-timing baseline segment, followed by the full task.

The required EE reference is:

X_BE(t) = X_BF * X_FT(t) * inverse(A)

The realized tibia pose is:

X_BT_actual(t) = FK(q(t)) * A

Do not re-anchor the task to the current robot pose. Do not use the old flexion-only executor. Do not command knee load from capability cases. Do not treat capability as measured load capacity.

After baseline replay succeeds, compare existing null-space modes under identical conditions. Record EE/tibia tracking, measured joint state, joint room, limiting joint, commanded torque, secondary torque, projection residual, phase coverage, reference freshness, and faults.
```
