"""Offline attachment geometry with a forbidden-direction cone.

X_TE = {}^T T_E maps EE coordinates into tibia coordinates.
A = {}^E T_T = inverse(X_TE).
X_BE_required = X_BT_required @ X_TE.

The forbidden EE +z directions lie within 45 degrees of tibia +x.
This is a user-defined orientation filter, not collision checking.
The exact 45-degree boundary is accepted (within numerical tolerance).
"""
from __future__ import annotations
import numpy as np
from transforms import homogeneous_transform, invert_transform, validate_transform

REFERENCE_TIBIA_OFFSET_M = 0.35
REFERENCE_R_TE = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])
REFERENCE_MOUNTING_DIRECTION = np.array([-1., 0., 0.])
FORBIDDEN_MOUNTING_DIRECTION = np.array([1., 0., 0.])
EXCLUSION_HALF_ANGLE_RAD = np.deg2rad(45.0)
REFERENCE_X_TE = homogeneous_transform(
    rotation=REFERENCE_R_TE,
    translation=np.array([REFERENCE_TIBIA_OFFSET_M, 0., 0.]),
)


def xte_to_attachment(xte: np.ndarray) -> np.ndarray:
    xte = np.asarray(xte, dtype=float)
    validate_transform(xte)
    return invert_transform(xte)


def attachment_to_xte(attachment: np.ndarray) -> np.ndarray:
    attachment = np.asarray(attachment, dtype=float)
    validate_transform(attachment)
    return invert_transform(attachment)


def _rotation_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed tibia axes: x first, then y, then z (not intrinsic XYZ)."""
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1., 0., 0.], [0., cx, -sx], [0., sx, cx]])
    ry = np.array([[cy, 0., sy], [0., 1., 0.], [-sy, 0., cy]])
    rz = np.array([[cz, -sz, 0.], [sz, cz, 0.], [0., 0., 1.]])
    return rz @ ry @ rx


def _direction_angle(xte: np.ndarray, direction: np.ndarray) -> float:
    xte = np.asarray(xte, dtype=float)
    validate_transform(xte)
    cosine = float(np.dot(direction, xte[:3, 2]))
    return float(np.arccos(np.clip(cosine, -1., 1.)))


def mounting_deviation_rad(xte: np.ndarray) -> float:
    """Angle of EE +z from the reference direction, tibia -x."""
    return _direction_angle(xte, REFERENCE_MOUNTING_DIRECTION)


def forbidden_direction_separation_rad(xte: np.ndarray) -> float:
    """Angle of EE +z from the forbidden central direction, tibia +x."""
    return _direction_angle(xte, FORBIDDEN_MOUNTING_DIRECTION)


def orientation_is_admissible(
    xte: np.ndarray,
    exclusion_half_angle_rad: float = EXCLUSION_HALF_ANGLE_RAD,
) -> bool:
    half_angle = float(exclusion_half_angle_rad)
    if not np.isfinite(half_angle) or not 0. < half_angle < np.pi:
        raise ValueError("exclusion_half_angle_rad must be between 0 and pi")
    separation = forbidden_direction_separation_rad(xte)
    return bool(separation >= half_angle - 1e-12)


def make_attachment_candidate(
    roll_angle_rad: float = 0.0,
    tibia_offset_m: float = REFERENCE_TIBIA_OFFSET_M,
    *,
    pitch_angle_rad: float = 0.0,
    yaw_angle_rad: float = 0.0,
    enforce_admissibility: bool = True,
) -> np.ndarray:
    """Return A = {}^E T_T.

    Preserve the original positional API: (roll, offset).
    Pitch/yaw must be named explicitly to avoid positional ambiguity.
    enforce_admissibility=False is for plotting rejected designs only.
    """
    angles = np.asarray([roll_angle_rad, pitch_angle_rad, yaw_angle_rad], dtype=float)
    offset = float(tibia_offset_m)
    if not np.all(np.isfinite(angles)):
        raise ValueError("orientation angles must be finite")
    if not np.isfinite(offset) or offset <= 0.:
        raise ValueError("tibia_offset_m must be finite and positive")
    rotation = _rotation_xyz(*angles) @ REFERENCE_R_TE
    xte = homogeneous_transform(rotation=rotation, translation=np.array([offset, 0., 0.]))
    if enforce_admissibility and not orientation_is_admissible(xte):
        separation_deg = np.rad2deg(forbidden_direction_separation_rad(xte))
        raise ValueError(
            f"EE +z is only {separation_deg:.2f} deg from forbidden tibia +x; "
            "at least 45 deg separation is required"
        )
    return xte_to_attachment(xte)


def required_ee_transform(
    tibia_transform_in_base: np.ndarray,
    attachment: np.ndarray,
) -> np.ndarray:
    tibia_transform_in_base = np.asarray(tibia_transform_in_base, dtype=float)
    attachment = np.asarray(attachment, dtype=float)
    validate_transform(tibia_transform_in_base)
    validate_transform(attachment)
    return tibia_transform_in_base @ attachment_to_xte(attachment)