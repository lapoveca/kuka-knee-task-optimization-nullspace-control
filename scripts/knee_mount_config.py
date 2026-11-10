#!/usr/bin/env python3
"""
Single source of truth for the fixed tibia-frame -> KUKA-EE-frame transform.

Frame notation:
  T = OpenSim tibia body frame
  E = KUKA end-effector frame (lbr_link_ee)

  X_TE maps coordinates expressed in T into E:
      p_E = X_TE @ p_T

The runtime trajectory chain is ALWAYS:

    X_BE(q) = X_BG @ X_GT(q) @ X_TE

where:
  B = KUKA base frame (lbr_link_0)
  G = OpenSim ground frame
  X_BG = OpenSim-ground -> KUKA-base calibration
  X_GT = OpenSim-ground -> tibia pose from forward kinematics

Do NOT add a tibia-length vector elsewhere in the planner or executor.
The complete physical EE mounting offset belongs here in X_TE.

IMPORTANT INITIAL CONDITION
---------------------------
Use identity first. This means the KUKA EE is treated as coincident with
OpenSim's tibia-body origin. It is intentionally a calibration/logic test,
not a claim about your real physical tibia length.

After the identity pipeline is verified (same-angle test has ~zero home
error and no jump), replace X_TE with a measured rigid transform:

    X_TE[:3, 3] = [dx, dy, dz]

The translation must be expressed in the OpenSim TIBIA BODY FRAME, not
KUKA base coordinates. If the EE is at the distal tibial end and OpenSim
local +Z truly points from knee to distal attachment, use:

    X_TE[:3, 3] = [0.0, 0.0, measured_knee_to_ee_distance_m]

Only do that after verifying the local-axis direction. A sign/axis guess
belongs here, never as a 90/180-degree patch to X_BG.
"""



import numpy as np

def make_transform(R=None, p=None):
    X = np.eye(4)
    if R is not None:
        X[:3, :3] = np.asarray(R, dtype=float)
    if p is not None:
        X[:3, 3] = np.asarray(p, dtype=float)
    return X


# We want:
#   EE +Z = tibia -X
#
# Choose a proper right-handed rotation:
#   EE +X = tibia +Y
#   EE +Y = tibia +Z
#   EE +Z = tibia -X
#
# Columns are E-frame axes expressed in T-frame coordinates.
R_T_E = np.array([
    [0.0, 0.0, -1.0],  # T.x components of [E.x, E.y, E.z]
    [1.0, 0.0,  0.0],  # T.y components
    [0.0, 1.0,  0.0],  # T.z components
])

# Initially retain zero translation. Validate orientation first.
X_TIBIA_EE = make_transform(R=R_T_E)


# Uncomment only after a physical measurement and an OpenSim-axis check.
DISTAL_OFFSET_M = 0.35 # replace with physical measurement

X_TIBIA_EE = make_transform(
    R=R_T_E,
    p=[DISTAL_OFFSET_M, 0.0, 0.0],
)

def invert_homogeneous(X):
    """Inverse of a rigid homogeneous 4x4 transform."""
    R = X[:3, :3]
    p = X[:3, 3]
    X_inv = np.eye(4)
    X_inv[:3, :3] = R.T
    X_inv[:3, 3] = -R.T @ p
    return X_inv
