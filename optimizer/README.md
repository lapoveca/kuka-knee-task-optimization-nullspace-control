# Knee-task optimizer

This directory searches fixed robot-relative femur placements and nine tibia
attachments for a KUKA LBR Med 14 knee-rehabilitation task. It generates
candidate mounts, screens them with pointwise inverse kinematics, evaluates
connected trajectories, audits the results, filters sampled robot-tibia
collisions, and applies a guarded offline ranking policy.

The supported campaign entry point is `run_vv_campaign.py`. It coordinates the
numerical pipeline and records reproducibility information in each campaign
directory. It does not control physical hardware or make clinical safety claims.

## Quick start

Run from `optimizer/` in the validated Python environment:

```bash
python3 run_vv_campaign.py --help
```

The standard population is:

```text
17 placements per arm x 4 arms x 9 attachments = 612 candidate designs
```

This is the generated candidate count, not a promised number of feasible
designs. Every screening pass is promoted for whole-task evaluation.

Create and retain one campaign directory, then launch:

```bash
CAMPAIGN_DIR="output/vv_612_$(date +%Y%m%d_%H%M%S)"

printf 'Full campaign: %s\n' "$CAMPAIGN_DIR"
printf '%s\n' "$CAMPAIGN_DIR" > output/LAST_VV_612_CAMPAIGN.txt

df -h .
systemd-inhibit \
  --what=sleep:idle \
  --mode=block \
  --why="VV 612-design optimization campaign" \
  python3 -u run_vv_campaign.py \
    --campaign-dir "$CAMPAIGN_DIR" \
    --placements-per-arm 17 \
    --workers 2 \
    --rng-seed 42 \
    --screen-seeds 4 \
    --screen-seconds 15 \
    --timeout-s 1800
```

Keep the computer connected to power, awake, and with sufficient free storage.
`systemd-inhibit` requests that sleep and automatic idle handling be blocked
while the campaign runs; it does not protect against power loss or every
privileged shutdown action.

## Resume an interrupted run

Do not create a new timestamped campaign directory when resuming. Recover the
saved root and repeat the identical launch command:

```bash
export FULL_CAMPAIGN_DIR="$(cat output/LAST_VV_612_CAMPAIGN.txt)"

systemd-inhibit \
  --what=sleep:idle \
  --mode=block \
  --why="VV 612-design optimization campaign" \
  python3 -u run_vv_campaign.py \
    --campaign-dir "$FULL_CAMPAIGN_DIR" \
    --placements-per-arm 17 \
    --workers 2 \
    --rng-seed 42 \
    --screen-seeds 4 \
    --screen-seconds 15 \
    --timeout-s 1800
```

The coordinator validates completed work and skips it. Interrupted designs
restart in a new attempt; completed attempts remain preserved. Resume is per
design, not per IK layer.

Do not edit runtime source, task inputs, models, or configuration files during
a campaign. Input or source changes intentionally prevent resume into
incompatible results.

Do not use `--retry-failed` automatically. A normal interruption and a recorded
software error or timeout are different situations. Inspect the relevant log
before requesting retries.

## Monitor a running campaign

Use a second terminal in `optimizer/`. These commands read existing artifacts
and do not interfere with the campaign.

Recover the active campaign root:

```bash
FULL_CAMPAIGN_DIR="$(cat output/LAST_VV_612_CAMPAIGN.txt)"
```

Follow the coordinator log:

```bash
tail -f "$FULL_CAMPAIGN_DIR/campaign.log"
```

Follow detailed screening progress:

```bash
tail -f "$FULL_CAMPAIGN_DIR/logs/02_screening.log"
```

Once evaluation starts, follow detailed evaluation progress:

```bash
tail -f "$FULL_CAMPAIGN_DIR/logs/04_evaluation.log"
```

Show the current screening counts:

```bash
python3 - "$FULL_CAMPAIGN_DIR" <<'PY'
import json
import sys
from collections import Counter
from pathlib import Path

root = Path(sys.argv[1])
path = root / "screening" / "summary.json"

if not path.exists():
    print("No screening summary written yet.")
else:
    data = json.loads(path.read_text())
    counts = Counter(row["status"] for row in data["results"])
    done = data["recorded_designs"]
    total = data["scheduled_designs"]

    print(f"Recorded: {done}/{total} ({100*done/total:.1f}%)")
    for status, count in sorted(counts.items()):
        print(f"  {status}: {count}")
PY
```

Estimate remaining evaluation time from completed jobs:

```bash
python3 - "$FULL_CAMPAIGN_DIR" <<'PY'
import re
import sys
from datetime import datetime, timedelta
from pathlib import Path

root = Path(sys.argv[1])
campaign = (root / "campaign.log").read_text()
evaluation = (root / "logs" / "04_evaluation.log").read_text()

match = re.search(
    r"^(\S+) START 04_evaluation\s*$",
    campaign,
    re.MULTILINE,
)
if not match:
    raise SystemExit("Evaluation start timestamp not found.")

start = datetime.fromisoformat(match.group(1))
now = datetime.now()
elapsed = (now - start).total_seconds()

completed = set(re.findall(r"^DONE\s+(\S+)", evaluation, re.MULTILINE))
started = set(re.findall(r"^START\s+(\S+)", evaluation, re.MULTILINE))
total = 85

print(f"Completed: {len(completed)}/{total}")
print(f"Started but not DONE: {len(started - completed)}")
print(f"Elapsed: {elapsed / 60:.1f} minutes")

if completed:
    remaining = max(0, total - len(completed)) * elapsed / len(completed)
    print(f"Throughput-based remaining: ~{remaining / 60:.0f} minutes")
    print(f"Estimated evaluation finish: "
          f"{(now + timedelta(seconds=remaining)):%H:%M}")
    print("Estimate assumes remaining designs have similar average runtime.")
PY
```

The evaluation estimate is indicative only. It assumes similar average runtime
for the remaining designs; long-running final jobs can extend completion.

## Campaign outputs

A campaign directory contains:

```text
campaign_manifest.json   frozen source/input/configuration hashes
campaign.log             coordinator progress
campaign_summary.json    final or interrupted/error status
stages/                  subprocess stage records
logs/                    detailed subprocess logs
generation/              proposal pool
screening/               manifest and per-design records
promoted.json            verified promotion manifest
evaluation/              parallel worker outputs, attempts, audits
collision_checks.json    incremental collision results and identities
ranking_guarded/         per-path scores and design ranking
```

The evaluation directory contains nested per-design batch directories. Use the
paths recorded in checkpoints, audit reports, and ranking artifacts rather than
assuming a shallow layout or recursively accepting every trajectory under
interrupted attempts.

## Pipeline

1. `generate_global_placements.py` generates an arm-balanced, robot-informed
   proposal pool. It validates the placement domain and a necessary outer-reach
   bound, not inverse kinematics.
2. `screen_joint_proposals.py` performs bounded pointwise IK and records a
   checkpoint for each design.
3. `promote_screened_designs.py` verifies proposal, task, and screening
   identities and promotes screening passes. Promotion precedes whole-task
   evaluation.
4. `run_parallel_design_eval.py` runs isolated per-design processes in batches.
   It calls `run_joint_design_eval.py`, which invokes the evaluator and auditor.
5. `tibia_collision.py` filters audited paths using sampled robot-versus-
   attached-tibia collision checks.
6. `rescore_saved_designs.py` scores eligible saved paths and selects the best
   path per design under the selected policy.

## Current VV task

The active envelope is:

```text
inputs/task_envelopes/varus_valgus_distal_force_attachment_origin/
```

The task lasts 320 seconds: five 40-second loaded stations and four 30-second
unloaded transfers. The source motion specification is:

```text
configs/tasks/varus_valgus_motion.json
```

Loading uses:

```text
catalog_normalized_attachment_origin_distal_force_v1
```

Full-precision attachment-specific constant forces are embedded in `task.json`.
Minimum-norm reference forces were verified to produce 10 Nm generalized
varus-valgus moment at every saved anatomical posture. Signs represent the two
alternative `[-1, +1]` cases.

Force is expressed in the tibia frame and applied at the selected attachment
origin, with zero direct couple there. The resulting moment is shifted
consistently to the tibia origin. Legacy NPZ wrench vectors are retained for
compatibility but are not the authoritative loads for this branch.

## Screening and evaluation

The placement domain is:

```text
configs/placement_domain.json
```

The provisional zero-bias torque budget is:

```text
configs/screening_budget_historical.json
```

Evaluation defaults are:

- 5-degree position reserve
- 12 initial seeds
- up to 5 tracks
- 2-second planning stride
- 0.1-second edge checks
- 0.01-second audit checks
- 200 IK function evaluations

Screening uses 4 seeds per pose and a 15-second outer design budget. This is
not a strict process timeout for every individual IK solve.

## Feasibility and winner eligibility

A complete, independently audited, collision-accepted path may still be
excluded from winner selection.

The guarded scorer requires 2 degrees of additional room beyond the 5-degree
reserve, for approximately 7 degrees of total minimum hard-limit room within
numerical tolerance.

The weighted score is:

```text
w_min * minimum_active_lambda / lambda_ref
+ w_mean * active_time_mean_lambda / lambda_ref
+ w_joint * joint_comfort_utility
```

The active-time mean integrates the worst active case within each loaded phase
and excludes transfers. It is not the evaluator's legacy mean of group minima.

Joint utility uses the worst saved-path room beyond reserve, not mean joint
room:

```text
1 - saturation ** (-max(0, extra_room_deg) / comfort_extra_room_deg)
```

The active policy is:

```text
configs/joint_score_weights_guarded_v2.json
```

with weights:

```text
0.6 capability minimum
0.1 active-time capability
0.3 joint comfort
```

Existing lambda reference values are retained as scoring normalizers. They are
not new physical limits. Scores are not constrained to the interval [0, 1].

Weights and normalizers can be changed offline with `rescore_saved_designs.py`,
using a new ranking output directory. Preserve all alternative paths because
the selected path can change with policy. Physical loading or torque-budget
changes are not merely weight changes. Offline reranking cannot recover paths
that the search did not retain.

The evaluator's internal lexicographic selection is not the final weighted
ranking rule.

## Collision scope and accepted assumptions

All eight loaded robot collision geometries are checked against a tibia
cylinder with radius 0.03 m, length 0.31 m, axis `+x_T`, and center
`[0.155, 0, 0]` m in the tibia frame.

Only `lbr_link_7_0` has a 0.0001 m interface-penetration allowance. Other
links retain strict rejection. Raw collision results remain visible. The
allowance is specific to the named geometry, not a separately resolved physical
contact patch.

Collision checks use saved nodes and joint-linear subdivisions with a maximum
joint step of 0.01 rad. This is not continuous swept-volume certification.

The study models sampled linear-joint kinematics and zero-bias task-torque
capacity. Acceleration, jerk, gravity, inertia, friction, real-controller
tracking, robot self-collision, fixtures, room geometry, and patient anatomy
are outside its scope. These are declared assumptions, not unimplemented
campaign acceptance gates.

## Completion statuses

| Status | Meaning |
|---|---|
| `complete` | At least one eligible ranked design |
| `complete_no_screen_passes` | Valid completed search with no screening passes |
| `complete_no_collision_accepted_paths` | No accepted trajectories, including cases where whole-task search found none |
| `complete_no_eligible_winner` | Accepted paths exist, but none passes ranking eligibility |
| `interrupted` | Resume using the same campaign root and configuration |
| `error` | Inspect `campaign.log` and `logs/` before retrying |

`complete_no_eligible_winner` is a valid completed search outcome, not an
orchestration or audit failure.

## Review and historical tools

- `review_robot_tibia.py`: robot meshes, frame-chain skeleton, actual and
  requested tibia/end-effector poses, error plots, and multiview trajectory GIFs.
- `report_historical_pilot.py`: historical capability, extrema, and domain
  diagnostics. Its lexicographic figures are not the final guarded weighted
  selection.
- `inspect_optimizer_geometry.py`: domain coverage and successful-path geometry.
- `make_optimizer_report.py` and `make_pipeline_geometry_report.py`: report
  generation tools.
- `check_saved_paths_tibia_collision.py`: standalone collision diagnostic.
- `diagnose_catalog_vv_loading.py`: pre-integration normalization diagnostic;
  it expects an obsolete single-force field and is not the current loader.
- `validate_*.py`: validation tools, some of which intentionally test legacy
  loading behavior.

Match historical trajectories to their original task identities and timing. Do
not silently overlay their old times against the 320-second task.

## Source and evidence policy

Generated results belong under `output/`. Preserve prior numerical evidence.

Do not edit `baseline_records/`; its manifests preserve the provenance of prior
frozen experiments.

Do not run blanket `git clean` or delete historical outputs during source
cleanup. One-off patch or installation scripts are development artifacts, not
campaign entry points. Archive rather than delete when uncertain.

The historical release snapshot under `nov9_knee_release_snap/` records that
copied pilot and auditor tools were reference tools rather than generic
envelope implementations, and that collision checks were excluded from that
historical study. Treat it as historical evidence, not current campaign
documentation.
