# Integration implementation plan

## Purpose

This plan defines the work required to make the optimizer-to-controller
pipeline a professional, verifiable research tool for the KUKA LBR Med 14
knee-assessment project.

The intended final outcome is a reproducible workflow that can:

1. Validate a selected optimizer plan and its task/model evidence.
2. Preview the planned task without commanding the robot.
3. Initialize the robot to the plan’s required start posture.
4. Execute the Cartesian task with a declared null-space configuration.
5. Record sufficient evidence to reconstruct what was commanded and measured.
6. Generate an automatic planned-versus-realized report.
7. Compare named controller configurations under identical task and
   initialization conditions.
8. Preserve all evidence, limitations, failures, and provenance for the
   report and supervisor handover.

This is not a clinical-safety system, collision-certification system, global
optimizer, or measured-load-capacity validation. It is a simulation-first
integration and evaluation workflow, with clearly bounded claims.

## Current baseline

The current pushed baseline is:

```text
94799a92b9a65bdce346489120e690659e5764fd
```

The local working tree is clean at that revision.

The installed package is:

```text
/home/lapoveca/ros2_ws/install/lbr_impedance_control
```

The installed controller library is:

```text
/home/lapoveca/ros2_ws/install/lbr_impedance_control/lib/liblbr_impedance_control_controllers.so
```

The installed package exposes the controller plugin:

```text
share/lbr_impedance_control/lbr_impedance_control.xml
```

The installed package contains generated action interfaces for both
`JointTrajectoryExecution` and `CartesianTrajectoryExecution`.

The exact source tree used to build the installed controller library has not
yet been established. Two source packages currently declare the same package
name:

```text
/home/lapoveca/ros2_ws/src/lbr_impedance_control/package.xml
/home/lapoveca/ros2_ws/src/kuka-knee-task-optimization-nullspace-control/controller/package.xml
/home/lapoveca/ros2_ws/src/kuka-knee-task-optimization-nullspace-control_nov10/controller/package.xml
```

This duplicate-name situation is acceptable only while the installed overlay
identity is explicitly known. It is not acceptable for a final release,
because an ambiguous source/build relationship can make the controller under
test impossible to identify.

## Supported launch configuration

The supported initial integration configuration is:

```bash
ros2 launch lbr_impedance_control lbr_JIC_CIC_gazebo.launch.py
```

This is equivalent to:

```bash
ros2 launch lbr_impedance_control lbr_JIC_CIC_gazebo.launch.py \
  model:=med14 \
  robot_name:=lbr \
  gravity:=false
```

The committed launch file states that the validated initial replay target is
`gravity=false`, and that `gravity=true` requires separate validation.

The launch configuration starts:

- Gazebo and the simulation clock bridge.
- Robot state publisher and robot spawning.
- Joint-state broadcaster.
- Joint impedance controller active.
- Cartesian impedance controller loaded but inactive.
- RViz with the packaged visualization configuration.

The integration executor performs initialization, controller switching, and
Cartesian task replay. The launch file does not execute a knee task.

## Non-negotiable principles

### Preserve working behavior

The C++ controller is the core deliverable. Do not rewrite it as part of
integration work.

The first integration changes must preserve:

- JIC-to-CIC switching.
- Existing task visualizations.
- Existing selected VV example.
- Existing controller interfaces.
- Existing optimizer artifacts.

### Separate claims

Never merge these into one result:

| Claim | What it requires |
|---|---|
| Execution completed | The task reached its defined end state or stop condition |
| Data are valid | Required signals were present, sampled, and internally consistent |
| Cartesian tracking passed | Position/orientation errors met predeclared tolerances |
| Posture followed the optimizer plan | Measured joint posture matched the saved posture path within declared criteria |
| NS improved behavior | A controlled comparison showed a defined benefit |
| Nominal capability improved | The task-wrench model evaluated measured posture under the declared budget |
| System is safe | A separately justified safety analysis, not this integration workflow |

### No silent corrections

Do not silently:

- Retry an initialization goal.
- Re-anchor a task frame to the measured robot pose.
- Retime or smooth a reference.
- Fill missing telemetry.
- Treat a rejected parameter as accepted.
- Treat an aborted action as successful because the robot later appears settled.
- Overwrite a prior run.
- Change archived optimizer evidence.

### Every run is reproducible

A run must identify:

- The exact source revision and local dirty state.
- The selected task, placement, attachment, and path.
- The controller configuration requested and actually accepted.
- The initialization policy.
- The replay representation.
- Input/output hashes.
- The environment and package provenance.
- All stage events and failures.

## Repository architecture

The current repository is broadly well organized:

```text
controller/     ROS 2 controller package
optimizer/      Offline task/placement/posture evaluation
integration/    Selected-plan export, validation, execution, comparison
scripts/        Current executable entry points
results/        Curated results
tests/          Tests
docs/           Documentation
```

However, the current `integration/` directory contains only `.gitkeep`, while
the actual integration executable is in `scripts/`. Do not perform a large
file move now. Instead, establish a stable internal architecture while
preserving the existing user entry point.

### Intended responsibilities

| Area | Owns | Must not own |
|---|---|---|
| `optimizer/` | Task envelopes, placement/attachment evaluation, posture candidates, saved-path audits, ranking and export | ROS control |
| `integration/` | Plan loading/validation, execution stages, run recording, report generation | Optimizer search |
| `controller/` | ROS 2 controller implementation, interfaces, launch/configuration | Optimizer-specific logic |
| `scripts/` | Thin user-facing executable entry points | Reimplementing controller or optimizer algorithms |
| `tests/` | Contract, unit, integration, and regression tests | Production control logic |
| `results/` | Curated, immutable or explicitly versioned outputs | Raw uncontrolled experiment output |

### Proposed integration modules

These are implementation targets, not files to create immediately. Exact names
and boundaries must be finalized after the remaining source review.

| Proposed module | Responsibility |
|---|---|
| `integration/plan_io.py` | Load and validate selected-plan/task artifacts |
| `integration/plan_preview.py` | Generate offline preview data and figures |
| `integration/run_recording.py` | Create run directories, manifests, event logs, and final outcomes |
| `integration/execution_stages.py` | Initialization, controller switching, replay, stop/failure behavior |
| `integration/ros_interfaces.py` | Bounded service/action/topic access and feedback validation |
| `integration/simulation_clock.py` | Simulation-time progress, pause/reset/stale-data handling |
| `integration/reference_stream.py` | Task-time interpolation and twist generation |
| `integration/posture_comparison.py` | Measured versus planned posture alignment |
| `integration/reporting.py` | Data validity, tracking, limits, effort, NS and posture reporting |
| `integration/controller_profiles.py` | Named, versioned baseline/NS configurations |

The user-facing command remains:

```bash
python3 scripts/execute_knee_task_from_optimizer.py
```

Internal modules should be imported by that command, not launched as separate
user commands.

## Work package 1: Environment and controller provenance

### Objective

Establish exactly which controller source and build produced the currently
installed package.

### Required evidence

1. Confirm the installed package prefix.
2. Identify the source package used to build it.
3. Confirm whether the installed package contains the expected plugin library.
4. Confirm the active controller plugin and controller-manager configuration.
5. Record ROS, Gazebo, controller-manager, DDS/RMW, Python, and relevant
   package versions.
6. Record the exact launch command and launch arguments used in the working
   simulation.

### Acceptance criteria

- The installed controller source can be identified unambiguously.
- The installed plugin library can be identified.
- The launch command is documented.
- No controller source is modified during this package.

### Deliverable

Create a dated environment/provenance record, for example:

```text
docs/environment_provenance_2026-11-10.md
```

It must state what is known, what is uncertain, and what must be rebuilt or
relocated before a clean release.

## Work package 2: Plan and selected-example validation

### Objective

Make the selected VV plan trustworthy before it is used to initialize or
command the robot.

### Inputs to validate

- `examples/vv_p000_a09/path_03/whole_trajectory_path.npz`
- `optimizer/inputs/task_envelopes/varus_valgus_distal_force/task.json`
- Task NPZ arrays
- Placement \(X_{BF}\)
- Attachment \(A = X_{ET}\)
- Saved path \(q_{\mathrm{ref}}(t)\)
- Task phases and active-case metadata
- Audit/eligibility evidence where available

### Required checks

- Task JSON and NPZ identities and hashes.
- Model identity and joint ordering.
- Rigid-transform validity.
- Fixed 0.35 m distal offset consistency.
- Required EE pose composition:

\[
X_{BE}(t) = X_{BF} X_{FT}(t) A^{-1}
\]

- Task time coverage, monotonicity, and sampling.
- Phase chronology and overlap checks.
- Saved-path start posture consistency.
- Start EE pose consistency.
- Joint-limit and velocity feasibility checks using the declared model/budget.
- Explicit statement of whether the saved path is a feasible reference, an
  optimizer output, or a closed-loop tracking target.

### Acceptance criteria

- A validation command loads the selected example without ROS.
- It reports explicit pass/fail for each check.
- It preserves input hashes and validation settings.
- It does not alter archived optimizer outputs.

### Deliverable

A supported command, for example:

```bash
python3 -m integration.validate_selected_plan \
  --path-dir examples/vv_p000_a09/path_03 \
  --envelope-dir optimizer/inputs/task_envelopes/varus_valgus_distal_force
```

The final command name and interface must be finalized during implementation.

## Work package 3: Reliable initialization

### Objective

Make initialization trustworthy and observable.

### Current problem

The executor currently sends the same JIC goal up to three times. A later
success does not independently establish reliable arrival. The current
settling logic uses sleeps and cached joint-state values.

### Required behavior

1. Validate the selected plan before controller changes.
2. Create a unique run directory.
3. Validate the target \(q_0\), model ordering, and task start pose.
4. Activate JIC only after required parameters are accepted.
5. Send exactly one JIC goal.
6. Record:
   - Goal acceptance.
   - Terminal action status.
   - Action result.
   - Fresh measured joint positions and velocities.
   - Sustained settling result.
7. Do not activate CIC unless initialization passes.
8. On failure, preserve the run outcome and attempt bounded cancellation only
   when appropriate.
9. Provide an initialization-only mode.

### Proposed initialization policy

The following values are engineering acceptance criteria for simulation, not
patient-safety limits. They must be recorded with every run and may be changed
only explicitly.

| Quantity | Provisional criterion | Rationale |
|---|---:|---|
| Maximum joint-position error | 0.01 rad | Approximately 0.57 degrees |
| Maximum joint velocity | 0.01 rad/s | Ensures near-rest initialization |
| Sustained settling duration | 1.0 s | Requires repeated evidence, not one lucky sample |
| Maximum sample gap | 0.25 s | Detects missing or stale feedback |
| Initialization action timeout | To be selected from controller behavior | Must not be arbitrary |
| Initialization settling timeout | To be selected from controller behavior | Must not be arbitrary |

These are provisional until the controller action behavior and simulation
clock behavior are inspected. They must not be adjusted after seeing a result
in order to make a trial pass.

### Action outcome semantics

The action status and `success` result must be recorded separately.

Possible outcomes:

| Action outcome | Measured settling | Integration decision |
|---|---|---|
| Succeeded | Passed | Initialization passed |
| Aborted | Passed | Do not silently accept; investigate controller semantics before deciding |
| Aborted | Failed | Initialization failed |
| Timeout | Passed | Do not silently accept; investigate |
| Timeout | Failed | Initialization failed |
| Goal rejected | Not applicable | Initialization failed |
| Parameter rejected | Not applicable | Initialization failed |

### Acceptance tests

Before proceeding to replay:

- Initialization-only succeeds from a displaced simulation posture.
- Already being at the target does not bypass measured validation.
- Missing or stale feedback fails within a bounded interval.
- A rejected goal or parameter does not activate CIC.
- An interruption produces a recorded outcome.
- Two attempts create two separate run records.
- Existing homing visualization still works.

## Work package 4: Simulation-time replay

### Objective

Make task progress depend on simulation time, not wall-clock execution speed.

### Current problem

The current executor schedules samples by wall-clock time and uses a fixed
loop rate. It does not correctly handle simulation pauses, clock resets, or
feedback latency.

### Required behavior

1. Subscribe to `/clock` and maintain a task clock.
2. Determine task time from elapsed simulation time.
3. Evaluate pose and twist at the same task time.
4. Do not advance the task during a simulation pause.
5. Handle clock reset or backward jumps explicitly.
6. Do not burst through a backlog of old samples.
7. Keep feedback acquisition non-blocking during normal publication.
8. Detect stale feedback.
9. Record hold and stop stages separately.
10. Preserve the task placement and attachment; do not re-anchor them to the
    measured start.

### Reference representation

The executor must document and test its exact reference representation:

- Task time grid.
- Interpolation method.
- Twist computation.
- Hold behavior.
- Whether reference is streamed continuously.
- Whether the controller applies filtering or internal interpolation.
- What happens at the final task time.

Piecewise-linear joint paths do not certify acceleration continuity. If
smoothing, filtering, or retiming materially changes the reference, it must be
documented and checked.

### Acceptance tests

1. Offline reference and time-grid tests.
2. Initialization plus initial hold.
3. One complete assessment phase and its following transfer, selected from
   actual phase metadata.
4. Pause/resume test.
5. Clock-reset and feedback-loss tests.
6. Full VV baseline replay.

## Work package 5: Run recording

### Objective

Ensure every execution produces sufficient evidence for later analysis.

### Minimum run contents

| Artifact | Required information |
|---|---|
| Manifest | Source revision/local-change status, input identities, model identity, configuration, requested and read-back controller settings |
| Events | Initialization, controller switch, replay start, phase changes, hold, stop, failures |
| Signals | Task time, source timestamps where available, receive times, reference pose/twist, actual pose, named joint position/velocity, published command torque |
| Outcome | Completion state, failure reason, coverage, data-validity status |

Controller reference and NS diagnostics should be recorded wherever the actual
controller interface provides them. We must inspect the published message
definitions before promising particular diagnostic fields. Unstamped signals
must not be presented as perfectly synchronized measurements.

### Required run directory

Use a unique, timestamped directory:

```text
runs/<UTC timestamp>_<profile>/
```

with at least:

```text
manifest.json
events.jsonl
initialization_result.json
initialization.csv
execution.csv
report.json
report.md
```

The exact schema may be refined during implementation, but it must be
documented and versioned.

## Work package 6: Automatic report

### Objective

Turn a valid run into a concise, interpretable report.

### Minimum report sections

1. Run identity and provenance.
2. Data-validity and completion summary.
3. Task-phase tracking: position and orientation.
4. Published reference versus controller reference versus actual motion.
5. Joint motion and minimum joint room.
6. Commanded/secondary effort.
7. Scaled conditioning and the selected objective.
8. Realized versus planned posture, aligned by task time.
9. Reconstructed tibia tracking.
10. Nominal task-wrench capability along measured posture.
11. Known limitations and interpretation.

### Report semantics

The report must explicitly separate:

- Execution completed.
- Data valid.
- Cartesian tracking passed.
- Posture tracking passed.
- NS comparison passed.
- Capability improved.

A run can complete while tracking fails. A run can produce valid data while
posture tracking fails. A negative or mixed result is valid evidence if
reproducible and correctly interpreted.

## Work package 7: Controlled NS comparison

### Objective

Compare controller configurations under identical task, initialization,
timing, and primary-gain conditions.

### First comparison

Use the existing secondary-off baseline and one selected existing NS
objective.

Before tuning:

- Freeze the task, placement, attachment, initialization policy, primary
  gains, replay representation, and metric definitions.
- Verify exactly what “secondary off” disables in the C++ implementation.
- Evaluate one NS objective at a time.
- Use bounded, documented parameter ranges—not the historical million-scale
  presets as experimental defaults.
- Repeat shortlisted configurations under the same conditions.
- Retain unsuccessful and negative trials with their reasons.
- Compare primary tracking against secondary benefits.

### Existing profiles

The current executor contains many NS presets, including joint-limit,
torque-minimization, singularity, directional, and combined configurations.
These should not all be treated as equally supported experimental
configurations.

The first supported set should be:

```text
baseline
ns_only
jl
tm
sing
```

Other presets remain available for development but are not automatically part
of the final comparison.

### Required outputs

For each configuration:

- Task tracking.
- Joint-room profile.
- Measured posture versus planned posture.
- Effort/secondary torque.
- Conditioning/singularity behavior.
- Completion and failure status.
- Repeatability across repeated runs.

## Work package 8: Optimizer posture-path endpoint

### Objective

Determine whether the controller can follow the optimizer’s complete redundant
posture path.

### Important distinction

Two claims remain distinct:

1. Local NS comparison: existing objectives improve or change realized posture
   on the selected task.
2. Optimizer-guided posture execution: a synchronized secondary posture
   reference follows the selected offline plan sufficiently well under declared
   criteria.

The second is the stronger integrated endpoint, but it is not a prerequisite
for baseline replay.

### Required decision

If the final endpoint requires explicit \(q_{\mathrm{ref}}(t)\) assistance and
the current interface cannot supply it, use a narrowly scoped, separately
tested extension.

Do not:

- Quietly relabel local-objective execution as offline-path tracking.
- Substitute full joint-path replay for Cartesian/null-space control.
- Add a controller feature without a corresponding test and documented
  interface.

## Work package 9: Optimizer release completeness

### Objective

Make the optimizer workflow independently usable and auditable.

### Current issue

The existing runner’s `--preflight` is not merely a read-only check. It
creates an output directory, obtains a lock, imports dependencies, loads
models/envelopes, and runs live evaluation jobs for VV and IE.

It expects manifests:

```text
inputs/global_promotion_VV_robotinformed_v1.json
inputs/global_promotion_IE_robotinformed_v1.json
```

Those manifests were not present in the earlier committed-file inventory.
Their presence must be verified before that command can be documented as
usable.

The runner also explicitly ends by saying guarded rescoring and figures follow
separately. Therefore, the copied search chain is not, by itself, the complete
search → rank → select → report workflow.

### Required separation

| Operation | Meaning |
|---|---|
| Read-only input/dependency validation | No expensive computation |
| Small smoke evaluation | Explicitly computational, bounded |
| Full search | Full optimization run |
| Saved-result audit and ranking | Uses existing outputs |
| Selected-plan export | Produces a self-contained execution plan |

### Required checks

- Verify the staged files against `release_inventory.json`.
- Compare original/snapshot and staged inventories.
- Enumerate omitted files.
- Classify omissions.
- Verify task/model/budget identities.
- Verify unique audited path membership.
- Verify expected/checked path counts and per-path pass flags.
- Reproduce the saved VV ranking through copied files only.
- Keep old result signatures intact.
- Do not edit manifests to make an archived resumable batch accept a new path
  or source signature.

### Expected historical VV regression targets

These must be verified from full-precision records, not assumed from rounded
printed values:

- 400 screening records.
- 37 screening passes.
- 37 reported audited designs.
- 148 indexed paths.
- 98 eligible and 50 excluded.
- 27 eligible designs.
- Winner `global_robotinformed_p000_a09 / path_03`.
- Rounded metrics:
  - Minimum lambda: 27.251.
  - Active-time mean: 30.390.
  - Actual-limit room: 19.616 degrees.
  - Score: 1.6530.

## Work package 10: Final release and handover

### Objective

Make the repository usable by another researcher without undocumented edits or
experimental history.

### Deliverables

- Supported source with a local checksummed version manifest.
- Tested environment/build instructions.
- Required model/world/config resources.
- Task-definition guide and existing-envelope example.
- Selected VV plan and preserved numerical evidence.
- Named baseline and supported NS profiles.
- A short validate/preview/initialize/run/report workflow.
- Automatic planned-versus-realized report and experiment index.
- Tests, known limitations, troubleshooting, supported and unsupported
  scenarios.
- An independent run-through by another user.

### Clean-workspace requirement

A clean installation must contain only one source package named:

```text
lbr_impedance_control
```

The current development workspace contains two source packages with that name.
This must be resolved before a clean release build.

## Execution order

1. Establish controller source/build provenance.
2. Validate the selected plan offline.
3. Implement reliable initialization and run identity.
4. Test initialization-only.
5. Implement simulation-time replay.
6. Implement complete recording.
7. Generate the first baseline report.
8. Freeze the baseline and compare named NS configurations.
9. Decide whether optimizer-guided posture execution requires a controller
   extension.
10. Complete optimizer release scope.
11. Perform clean-workspace validation.
12. Polish documentation, results, and report.

## Immediate next actions

1. Provide controller source/build provenance output.
2. Provide build metadata output.
3. Confirm the exact launch command.
4. Confirm the selected example.
5. I inspect the remaining source and produce a file-specific implementation
   specification.
6. Implement the first bounded increment only after the specification is
   approved.
7. Run initialization-only and return its recorded outcome.
8. Resolve any genuine controller/action discrepancy before enabling replay.
9. Implement simulation-time replay and complete recording.
10. Obtain a valid baseline report.
11. Then tune, compare, and write up verified results.