# Development status

## Current scope

This repository contains:
- A curated modified lbr_impedance_control ROS 2 package.
- Offline knee-task placement/attachment/posture optimization tools.
- Selected task envelopes and a saved VV example.
- A developing integration executor and RViz visualization.

The controller, optimizer, and integrated workflow are intended to remain
independently usable.

## Established evidence

- The original controller package builds in the development workspace.
- The curated controller C++ sources, headers, and plugin XML were compared
  with the working source and matched.
- The selected VV example loads and covers 0–260 s.
- Its saved start posture agrees numerically with the offline task-start
  end-effector pose.
- Full simulation replays have completed.
- Homing-target, task-path, actual-path, and schematic tibia visuals work.
- Release resource-presence, launch syntax, and package XML checks passed.

These checks do not establish a successful clean-workspace release build.

## Known limitations

- JIC retry handling can report success without proving measured arrival.
- The executor's settling check needs fresh, named joint-state validation.
- Replay currently uses wall-time/sample-index scheduling.
- Reference-cache identity checks and complete run recording are pending.
- Current replay tracking has not been accepted against task tolerances.
- The saved-path approximation error is not a closed-loop tracking prediction.
- Replay does not follow the complete saved redundant joint-posture path.
- Extensive NS tuning and controlled comparison trials are pending.
- License/redistribution metadata for inherited controller code is unresolved.

## Dependency provenance

Controller upstream base:
8961fa5f8231d69e80105ebe17ee85f4848a7ca9

LBR stack base:
dab3f7561e8a2cbefccd48b5edaf1a1663a8c6d9

The stack simulation patch is preserved under patches/.
Forward application and build in a clean pinned workspace remain pending.

## Development-workspace exception

controller/COLCON_IGNORE is local and intentionally excluded from Git because
the original controller package is also present in the development workspace.

A clean installation must contain only one source package named
lbr_impedance_control.

## Next implementation increment

Reliable initialization and run identity:
- Validate inputs before motion.
- Create a unique run directory.
- Send one JIC action goal.
- Record action status and result.
- Verify sustained settling using fresh joint states.
- Provide an initialization-only mode.