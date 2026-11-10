# External simulation patch

## Target

Repository: https://github.com/lbr-stack/lbr_fri_ros2_stack
Base commit: dab3f7561e8a2cbefccd48b5edaf1a1663a8c6d9

Patch: lbr_fri_ros2_stack_simulation.patch

## Purpose

- Configure the simulation controller manager at 200 Hz.
- Register the project JIC and CIC controllers and their initial parameters.
- Expose effort command interfaces in Gazebo.
- Retain position and effort command interfaces outside Gazebo.

This records the working development setup. It is not a general-purpose
upstream compatibility patch.

## Validation status

- Reverse applicability checked against the modified development checkout.
- Forward application and build from a clean pinned checkout: pending.
- Hardware behavior: not validated by this simulation workflow.

Do not apply the patch a second time to an already patched checkout.
Check applicability before applying it.