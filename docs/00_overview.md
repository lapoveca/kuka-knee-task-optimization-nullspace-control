# Project overview

This repository documents my personal contribution to the BARR project on
robot-assisted knee assessment and rehabilitation.

The project combines two related contributions:

1. **Offline task optimization:** evaluation of robot placement, tibia
   attachment orientation, and connected null-space posture candidates.
2. **Null-space control:** Cartesian impedance control with posture-guided
   null-space assistance for the KUKA LBR Med 14.

The offline optimizer identifies and audits promising setup and posture
candidates. A selected plan can then provide a posture reference to the
Cartesian impedance controller. The controller aims to preserve the primary
Cartesian knee-assessment task while using null-space control to influence
redundant posture behaviour.

## Main components

| Component | Purpose |
|---|---|
| `optimizer/` | Task-envelope loading, placement/attachment evaluation, path search, auditing, and guarded ranking |
| `controller/` | Cartesian impedance control and null-space/posture assistance |
| `integration/` | Selected-plan export, validation, execution, and measured-versus-planned comparison |
| `docs/` | Methods, assumptions, limitations, and report-facing documentation |
| `results/` | Curated figures, tables, and experiment summaries |

## Important boundary

This repository contains my own implementation, analysis, documentation, and
selected non-sensitive results. It does not represent the complete BARR
project or the work of other project members.