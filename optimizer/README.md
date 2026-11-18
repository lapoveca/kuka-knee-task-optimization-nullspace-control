# Knee-task optimizer

This directory generates, screens, evaluates, promotes, and audits robot mounting
proposals for KUKA LBR Med 14 knee-rehabilitation tasks.

## Pipeline

1. Generate a globally sampled, arm-balanced placement and attachment pool:

   ```bash
   python3 generate_global_placements.py \
     --task-envelope inputs/task_envelopes/varus_valgus_distal_force \
     --placements-per-arm 10 \
     --max-proposal-attempts 100000 \
     --max-attachment-attempts-per-placement 1000 \
     --output-dir output/global_placement_pilot_40x1_YYYYMMDD_HHMMSS
   ```

   The generator accepts exactly 10 placements per enabled arm. Arm identity is
   determined after forward kinematics; no arm may be silently backfilled by
   another arm.

2. Screen every attachment proposal with pointwise IK:

   ```bash
   python3 screen_joint_proposals.py \
     --proposals output/<generation_dir>/proposals.json \
     --output-dir output/screen_<run_name> \
     --max-designs 160 \
     --seeds-per-pose 8 \
     --max-nfev 2000 \
     --rng-seed 42 \
     --design-seconds 15.0
   ```

   `pointwise_screen_pass` means IK succeeded at all task poses. It does not
   imply a connected path, collision-free motion, wrench capability, dynamic
   feasibility, or global optimality.

3. Evaluate promoted/screened candidates and produce ranking and capability
   artifacts.

4. Promote candidates only after evaluation.

5. Audit the final run for reproducibility and provenance.

6. Generate human-readable report figures from the evaluation outputs.

## Current active configuration

- Placement domain: `configs/placement_domain.json`
- Screening budget: `configs/screening_budget_historical.json`
- Robot model: `models/med14_runtime.urdf`
- VV task envelope: `inputs/task_envelopes/varus_valgus_distal_force`
- Internal/external-rotation task envelope:
  `inputs/task_envelopes/internal_external_rotation`

## Output policy

Generated results belong in `output/`. Do not commit generated result
directories. Historical baseline manifests under `baseline_records/` must not
be edited because they preserve the provenance of prior frozen experiments.
