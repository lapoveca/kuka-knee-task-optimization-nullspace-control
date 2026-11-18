# Task envelope provenance

This file records the external build and validation provenance for the task
envelopes in this repository. It is not required to use or evaluate an existing
envelope.

The motion compiler and envelope finalizer are external to this repository.
Do not rerun these commands against existing output directories.

## Varus-valgus distal force

- Motion specification: `configs/tasks/varus_valgus_motion.json`
- Compiled motion: `inputs/compiled_motion/varus_valgus_motion/`
- Task envelope: `inputs/task_envelopes/varus_valgus_distal_force/`

### Build commands

```bash
python3 compile_knee_motion_v1.py \
  --spec configs/tasks/varus_valgus_motion.json \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --legacy-vv-dir inputs/legacy_vv \
  --output-dir inputs/compiled_motion/varus_valgus_motion

python3 finalize_knee_task_envelope_v1.py \
  --motion-dir inputs/compiled_motion/varus_valgus_motion \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --moment-axis y \
  --legacy-vv-dir inputs/legacy_vv \
  --output-dir inputs/task_envelopes/varus_valgus_distal_force
```

### Validation

The completed envelope was validated with the task-envelope interface
regression:

- 1041 target comparison times
- Maximum matrix difference: 1.6156e-27
- Fixed-reference minimum lambda: 4.015329051631654
- Reference hard joint margin: 9.171280217628958 degrees

## Internal-external rotation

- Motion specification:
  `configs/tasks/internal_external_rotation_motion.json`
- Compiled motion:
  `inputs/compiled_motion/internal_external_rotation_motion/`
- Task envelope: `inputs/task_envelopes/internal_external_rotation/`

### Build commands

```bash
python3 compile_knee_motion_v1.py \
  --spec configs/tasks/internal_external_rotation_motion.json \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --output-dir inputs/compiled_motion/internal_external_rotation_motion

python3 finalize_knee_task_envelope_v1.py \
  --motion-dir inputs/compiled_motion/internal_external_rotation_motion \
  --model /home/lapoveca/ros2_ws/src/mechanical_knee_testbed_cbl.osim \
  --moment-axis x \
  --output-dir inputs/task_envelopes/internal_external_rotation
```

### Validation

The completed envelope was included in the task-envelope interface regression
described above.
