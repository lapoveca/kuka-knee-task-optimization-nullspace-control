# KUKA Knee-Assessment Project  
## Progress Report and Remaining Work — 14 November 2026

**Platform:** KUKA LBR Med 14; ROS 2 Jazzy; `lbr_fri_ros2_stack`; `lbr_impedance_control`  
**Project focus:** control development, particularly null-space control, for knee assessment and rehabilitation.  
**Status:** working research components; integration and evaluation are progressing, but not yet finalized.

---

## 1. Executive summary

The project has reached an important integration milestone: offline-selected knee-task references can now be executed through the JIC-to-CIC workflow, and three baseline executions have been reported as completing their full reference paths.

The current work connects two main components:

1. An offline optimizer for selecting robot–task placement, tibia attachment, and connected robot posture paths.
2. A ROS 2 Cartesian impedance controller, extended with modular null-space objectives and execution diagnostics.

Earlier project reports identified the optimizer–controller–evaluation connection as the main unfinished integration task. The latest execution results represent progress beyond that earlier status: JIC initialization, task-reference loading, Cartesian streaming, visualization, logging, and complete baseline playback have now worked together in reported simulation runs.

**Current status:** the baseline execution pipeline works on multiple tested paths, but remains a development configuration. The next phase is to consolidate the optimizer and its task definitions, regenerate candidates with revised placement sampling and longer transfers, and improve execution usability and reliability before systematic null-space evaluation.

This report distinguishes:

- Features documented in available source and historical reports.
- Results directly supplied in the recent conversation.
- Remaining work and proposed next steps.

The exact identities of the two most recently completed paths were not supplied with their summaries; they are therefore recorded by run ID rather than assigned IE/VV labels.

---

## 2. Project objective and scope

The project supports robot-assisted knee assessment and rehabilitation using the KUKA LBR Med 14. The user’s principal contribution is control development, particularly null-space control, alongside the setup/posture optimization and execution tools needed to evaluate it.

The software platform is ROS 2 Jazzy with the `lbr_fri_ros2_stack` and the supervisor’s `lbr_impedance_control` foundation. The project extends that foundation rather than replacing it with an unrelated controller implementation.

Three decisions must remain distinct:

| Decision | Role |
|---|---|
| Robot–task placement | Defines the relative physical setup between the robot and anatomical task. |
| Attachment | Defines the fixed rigid relationship between the end effector and tibia/task frame. |
| Robot posture | Defines the joint configuration realizing the task pose and can vary throughout execution. |

The offline optimizer evaluates setup and posture alternatives. The controller must realize the primary anatomical motion while managing redundancy. Measured execution is needed to determine whether the predicted advantages survive actual controller behavior.

The intended outcome is a reproducible research workflow—not clinical validation, globally optimal placement, or comprehensive dynamic and collision certification.

---

## 3. Offline optimizer: progress to date

### 3.1 Established workflow

The documented optimizer workflow has progressed beyond isolated station or prefix checks to whole-task evaluation with saved connected paths.

The verified runtime chain in the November 9 consolidation report consists of:

1. Placement and attachment proposal generation.
2. Bounded pointwise screening.
3. Promotion of screened candidates.
4. Resumable evaluation orchestration.
5. Whole-task path generation.
6. Independent saved-path auditing.
7. Path-level eligibility checking, rescoring, and supervisor-oriented reporting.

The documented runtime includes robot modeling, attachment transforms, task-envelope evaluation, anatomical mapping, workspace checking, and wrench-margin evaluation. The release snapshot also preserves task inputs, actuator budgets, and the runtime robot model.

A failed bounded search must remain labeled “no solution/path found within budget,” not “physically impossible.”

### 3.2 Task definitions and capability interpretation

The project uses task envelopes to describe anatomical motion and associated loading requirements. Placement and attachment determine the required end-effector reference.

The documented transform convention is:

\[
X_{BE,\mathrm{required}}(t)
=
X_{BT,\mathrm{required}}(t)\,A^{-1},
\qquad A=X_{ET}.
\]

The realized anatomical pose is reconstructed using:

\[
X_{BT,\mathrm{realized}}(t)=FK(q_{\mathrm{measured}}(t))\,A.
\]

Consequently, end-effector position error and anatomical tracking error are not interchangeable: orientation error at an offset attachment can produce additional anatomical position error.

The historical capability model is a nominal actuator-budget calculation. Unless an explicitly verified bias model is added, it excludes gravity, inertia, friction, and unmodeled patient or fixture loading. Its capability factors are not permitted patient loads or measured physical headroom.

### 3.3 Historical search evidence

The November 8 handover records the following VV batch:

| Stage | Recorded result |
|---|---:|
| Screened proposals | 400 |
| Pointwise screen passes | 37 |
| Completed whole-task evaluations | 37 |
| Reported audited-path successes | 37 |
| Indexed paths in guarded review | 148 |
| Eligible paths | 98 |
| Designs retaining an eligible path | 27 |

These are historical batch results, not counts for the current example paths or a newly regenerated search.

The same handover recorded IE evaluation as incomplete at its last confirmed checkpoint: 18 of 73 whole-task evaluations completed, with 16 reported audited successes and two searches without a path within budget. This report does not infer the final outcome of that historical IE batch. Recent execution successes also do not establish its final audit coverage.

### 3.4 Consolidation already started

The November 9 report documents a release-staging implementation that preserves the established optimizer layout, copies runtime dependencies and evidence, verifies copied hashes, and records inventory/status information.

That work was staging—not proof of standalone operation from a clean checkout. Task-authoring entry points, relocation of archived absolute paths, and complete release validation remained open.

---

## 4. Controller implementation and integration

### 4.1 Controller capabilities documented in source

The available project records describe:

- Cartesian impedance pose tracking.
- Streamed pose and desired-twist interfaces.
- Cartesian trajectory actions and reference-management services.
- Modular null-space enable/disable behavior.
- Joint-limit, torque-minimization, singularity-related, and directional objectives.
- Null-space damping and torque limiting.
- Joint, tracking, torque, and capability-related diagnostics.

These are implementation capabilities. They do not imply that every objective, gain combination, or task has been experimentally validated.

Earlier review also identified limitations requiring final reconciliation:

- A regularized projector does not establish exact dynamic task decoupling.
- Headroom-scaling parameters must be checked against the torque path actually used.
- Controller timing and real-time behavior require measurement.
- Source snapshots do not prove which installed plugin is loaded.

### 4.2 Execution integration achieved

Recent terminal outputs demonstrate an integrated workflow that:

1. Loads a saved optimizer path and task envelope.
2. Loads or constructs an end-effector reference.
3. Initializes the robot toward the optimizer’s starting configuration using JIC.
4. Checks the starting Cartesian pose.
5. Activates CIC task execution.
6. Publishes references and visualization.
7. Records tracking and torque-related data.
8. Produces completion or abort summaries.

Multiple runs reported successful initialization followed by complete task playback. The user nevertheless reports remaining JIC initialization problems; initialization must therefore be described as working in successful cases, not fully reliable.

---

## 5. Tracking investigation: findings and unresolved issues

### 5.1 Straight-line diagnostic

A diagnostic script streamed a 50 mm Cartesian line over 20 seconds, followed by a hold.

Inspection of the supplied script and ROS topic information established that this test published pose references only: the reference-twist topic had no publisher.

Subsequent line tests showed:

- Several millimetres of motion error.
- Persistent hold errors around 4.9–5.8 mm in the reported configurations.
- No substantial convergence during an extended 30-second hold.
- Predominantly along-path error in the axis-resolved example.

These observations establish residual tracking error, but do not identify its physical cause.

### 5.2 Controller modification under test

The supplied local controller excerpt includes:

```cpp
tau_damping_ff = d_joint * q_dot;
```

added to the final torque sum.

This is measured-velocity damping compensation, rather than desired-velocity feedforward. It vanishes at zero joint velocity and therefore cannot directly remove a static error by itself.

Its coefficient, runtime value, and relationship to the loaded simulation model should be captured in the experiment configuration before final evaluation.

### 5.3 Earlier causal conclusions require correction

Several explanations offered during debugging were stronger than the evidence justified:

- Nonzero commanded torque at rest does not prove friction.
- A healthy Jacobian at one stopped configuration does not validate the entire path.
- Better tracking at higher stiffness does not uniquely identify the disturbance.
- Position-error logs alone do not establish that rotational stiffness was inadequate.
- Lower error in some segments does not prove a purely velocity-dependent limitation.

At the inspected stopped configuration, the reported scaled Jacobian condition number was approximately 3.90, with a minimum scaled singular value of approximately 0.207. Those diagnostics did not support the proposed near-singularity explanation at that configuration.

The root cause of all remaining tracking error is still unresolved.

---

## 6. Current baseline execution results

The following completion summaries were supplied:

| Run / identification | Status | Maximum position error | Mean position error | Final position error | Reported optimizer maximum |
|---|---|---:|---:|---:|---:|
| `vv_pitch90_path00`, run `20261114T093636_657641Z_baseline` | Done | 21.5 mm | 7.8 mm | 6.3 mm | 3.4 mm |
| Run `20261114T095206_550441Z_baseline`; path not supplied | Done | 20.7 mm | 11.6 mm | 12.9 mm | 1.8 mm |
| Run `20261114T100410_916909Z_baseline`; path not supplied | Done | 11.2 mm | 3.1 mm | 2.1 mm | 4.1 mm |

The first run was explicitly reported with:

```text
Kt = 3000
Kr = 150
TIME_SCALE = 2.0
Position gate = 30 mm
Null-space mode = baseline
```

The latest two summaries were presented as subsequent baseline testing, but their exact gains, timing configuration, envelope identities, and path names should be recovered from the run records rather than assumed.

### Interpretation

Three complete baseline executions are a meaningful integration milestone. They establish that the current pipeline can traverse the tested references without the earlier reported aborts or indefinite gate stalls.

They do not yet establish:

- Acceptable anatomical position/orientation accuracy.
- Suitability of the current stiffness for patient interaction.
- Correct nominal protocol timing.
- Reproducibility across resets and repeated trials.
- Null-space benefit.
- Agreement between optimizer-predicted and realized posture behavior.

The optimizer’s reported maximum and the controller’s measured tracking error should remain separately labeled until their definitions and reference constructions are reconciled. A predicted maximum must not be described as a predicted mean.

---

## 7. Important execution issues to resolve

### 7.1 Time scaling is not yet verified as genuine slowing

The earlier proposed edit multiplied `t_dense` without necessarily changing index advancement.

The successful run shows:

- Reference index advancing by approximately 50 samples per wall-clock second.
- Printed reference time advancing by approximately two seconds per wall-clock second.

For example, index 6450 at printed time 258 seconds advances to index 6500 at 260 seconds roughly one wall-clock second later.

This confirms doubled time labels/sample spacing in `t_dense`; it does not confirm half-speed geometric playback. If the pose array was unchanged and still advanced once per 50 Hz tick, playback speed was unchanged while the differentiated reference twist was reduced.

The previous claim that this demonstrated a 520-second playback was therefore not supported.

Before final experiments, implement and verify one authoritative progress clock. A genuine slowdown must change how quickly the pose progresses through the path, not merely its timestamps.

### 7.2 Gate and desired twist must be consistent

The gate patch can hold the pose sample while continuing to publish the nominal nonzero twist calculated at that sample.

That creates inconsistent commands: a stationary desired pose and a desired velocity indicating continued movement.

The corrected executor must derive both pose and twist from the same task-progress variable. When progress pauses, desired linear and angular velocities must reflect that pause.

### 7.3 A gate is not a tracking guarantee

A 30 mm gate is a progression threshold, not a validated accuracy bound or safety guarantee.

Remaining work includes:

- Explicit prolonged-stall handling.
- Orientation monitoring.
- Fresh-feedback handling.
- Defined cancellation and exit behavior.
- Zero/properly retimed twist during holds.
- Clear final-settling criteria.

The current completion condition should not be interpreted as proof that the final pose meets a reviewed task tolerance.

### 7.4 Current stiffness is provisional

The higher translational stiffness improved observed tracking, but it is substantially above the supervisor’s previously used range.

The current gains should remain a named simulation-development profile—not an approved clinical or hardware setting. Final tuning should follow timing correction and review with the supervisor.

---

## 8. Remaining optimizer work

The user has identified the following next scope.

### 8.1 Revise placement sampling

- Specify the revised admissible placement region and orientation ranges.
- Implement the sampling change in the authoritative generator.
- Record units, frames, bounds, and sampling policy.
- Keep previous search evidence immutable.

### 8.2 Lengthen transfers

- Identify the transfer phases that were too fast.
- Modify timing in the task-envelope definition.
- Verify smooth transitions and consistent derivatives.
- Regenerate dependent reference caches.
- Recheck resulting artifacts rather than reusing old audit labels unchanged.

### 8.3 Rerun IE and VV optimization

After placement and timing changes:

1. Validate envelopes.
2. Generate candidates.
3. Screen and evaluate.
4. Audit saved paths.
5. Apply documented eligibility/ranking.
6. Export selected examples and alternatives.

Each selected path must retain its correct task-envelope association. IE examples must not automatically be run with a VV envelope.

### 8.4 Clean and consolidate optimizer code

- Identify supported entry points.
- Separate active tools from historical experiments.
- Consolidate task-envelope generation and validation.
- Remove hidden working-directory assumptions where practical.
- Document cache invalidation and provenance.
- Add a clean-checkout smoke test.
- Preserve historical results and configurations.

---

## 9. Remaining execution and controller work

### 9.1 Add an explicit “Start task” stage

Desired sequence:

```text
Load and validate plan
→ Preview setup/start posture
→ JIC homing
→ Verify measured settling
→ Operator-ready state
→ Explicit task start
→ CIC task execution
→ Final hold and report
```

The wait state must define controller ownership, pose/twist references, cancellation, and recording behavior.

### 9.2 Improve JIC initialization

Document the remaining failure cases and address them individually:

- Initialization result versus measured settling.
- Timeouts and feedback freshness.
- Controller switching.
- Start-pose compatibility.
- Recovery after failure.

The exact unresolved initialization defects have not all been supplied, so this report does not assign a single cause.

### 9.3 Improve visualization and live reporting

Add or consolidate:

- Current station/transfer identity.
- Reference versus actual EE and anatomical frames.
- Position and orientation errors.
- Gate state and stall duration.
- Wall time versus task-progress time.
- Active gains, mode, plan, and recording directory.

### 9.4 Tune and evaluate null-space control

After the corrected baseline is repeatable:

- Review primary gains and damping composition.
- Verify secondary torque bounds and active implementation.
- Test objectives separately before combinations.
- Compare primary tracking, joint clearance, posture, capability, torque usage, and runtime.

The offline joint path and the posture actually realized by baseline CIC must remain distinct. Cartesian playback alone does not establish execution of the optimizer’s selected joint path.

---

## 10. Planned repository structure

Two repositories are planned.

### Repository A: optimizer

Owns:

- Task envelopes and authoring/validation.
- Placement and attachment sampling.
- Robot/setup models.
- Screening, path generation, auditing, and ranking.
- Offline visualization and export.
- Reproducible examples and tests.

### Repository B: controller extension

Remains a modification/extension of the supervisor’s `lbr_impedance_control`, preserving upstream attribution and a clear record of changes.

Owns the controller-side additions and associated configuration, diagnostics, launch integration, and documentation.

The task executor’s final location remains an architectural decision. Whichever repository owns it, the connection should use a documented plan interface rather than requiring the entire optimizer search environment during execution.

The earlier handover already identified plan identity, frames, transforms, timing, joint ordering, model provenance, and validation status as necessary parts of that interface.

---

## 11. Recommended order from here

| Priority | Work | Completion evidence |
|---|---|---|
| 1 | Archive the three completed runs and current source/configuration | Exact path, envelope, gains, timing, and revision recorded |
| 2 | Revise placement sampling and lengthen envelope transfers | Reviewed configuration and validated task inputs |
| 3 | Correct executor timing and pose/twist synchronization | Measured wall/progress timing and consistent paused references |
| 4 | Consolidate optimizer and rerun IE/VV | Audited outputs and reproducible selected examples |
| 5 | Improve initialization, explicit start, and visualization | Repeatable operator workflow with documented failures |
| 6 | Reassess primary gains with supervisor | Supported simulation tuning and tracking criteria |
| 7 | Evaluate null-space modes | Controlled comparisons against repeatable baseline |
| 8 | Finalize two repositories and handover | Clean-checkout reproduction and documented limitations |

## 12. Overall assessment

The project has moved from separate research components to demonstrated end-to-end baseline execution on multiple tested paths.

The remaining work is principally consolidation, corrected timing/reference handling, initialization reliability, controlled tuning, and evaluation—not starting the architecture again.

The appropriate current claim is:

> An offline setup/posture optimization workflow and a ROS 2 impedance-control execution pipeline have been integrated sufficiently to complete multiple baseline knee-task reference paths in reported simulation runs. Tracking remains configuration-dependent, the current high-stiffness profile is provisional, and timing, initialization, and null-space evaluation require further work before final handover.

The two missing path identities and their saved configurations can be filled into the results table from the run records without changing this assessment.
