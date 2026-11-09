# KUKA Knee Task Optimization and Null-Space Control

Personal contribution to the BARR research project on robot-assisted knee
assessment and rehabilitation, completed within the Honours Master’s
Programme (HPM) at TU Delft.

This repository contains my implementation, evaluation, and documentation for:

1. Offline evaluation of robot placement, tibia attachment orientation, and
   connected null-space posture candidates for knee-assessment tasks.
2. Null-space/posture-guided Cartesian impedance control for the KUKA LBR
   Med 14.
3. Integration, validation, and comparison of optimizer-selected plans with
   realized robot motion.

## Repository structure

```text
docs/           Project documentation, methods, and report-facing material
optimizer/      Offline task, placement, attachment, and posture evaluation
controller/     ROS 2 Cartesian impedance and null-space control
integration/    Selected-plan export, validation, execution, and comparison
results/        Curated figures, tables, and experiment summaries
tests/          Unit, contract, and integration tests
```

## Scope and limitations

This repository does **not** contain the complete BARR project, clinical
datasets, private supervisor material, or the complete work of other project
members.

The optimizer results are model-derived and depend on the declared task,
coordinate frames, robot model, computational budget, and evaluation policy.
They do not by themselves certify collision avoidance, hardware capacity,
clinical safety, or global optimality.

## Starting point and attribution

The controller work builds on [`lbr_impedance_control`](https://github.com/sklevering/lbr_impedance_control)
by S. Klevering. That upstream package provides ROS 2 Jazzy torque-based
Cartesian and joint impedance controllers for the KUKA LBR Med 14.

My contribution focuses on null-space control and the offline task/placement
evaluation workflow.