"""Tibia attachment geometry for the KUKA knee-task optimizer.

Coordinate conventions
----------------------

Tibia frame ``T``:

* ``+x_T`` points distally, from the knee toward the ankle.
* ``+y_T`` and ``+z_T`` span the tibia cross-section.
* The tibia-frame origin is the reference origin used by the task envelope.

End-effector frame ``E``:

* ``+z_E`` is the mounting/approach direction.
* For the axial mount, ``+z_E`` points toward the knee, i.e. along ``-x_T``.
* For each peripheral mount, ``+z_E`` points from the mount toward the
  tibia cylinder centerline.

Transform conventions
---------------------

``X_TE`` maps end-effector coordinates into tibia coordinates:

    p_T = X_TE @ p_E

``A`` is the attachment transform used by the optimizer:

    A = inverse(X_TE) = {}^E T_T

Thus:

    X_BT = X_BE @ A

and the required end-effector pose for a tibia pose is:

    X_BE_required = X_BT @ X_TE

Attachment catalog
------------------

There are nine equally important, discrete attachment candidates:

* one axial in-line mount at 0.31 m;
* eight peripheral mounts at 0.28 m and 0.03 m radial offset.

The peripheral mounts are spaced at 45 degree intervals around the tibia
cylinder. Their EE ``+z`` axes point inward, toward the cylinder centerline.

Forbidden-direction cone
------------------------

``orientation_is_admissible()`` implements a user-defined orientation filter:
it rejects an EE ``+z`` direction lying within 45 degrees of tibia ``+x``.

This is **not** collision checking. It was originally useful for excluding
nearly axial mounting directions in the old rotation-only attachment model.

It is retained as an optional utility for future custom attachments, but it
must not be applied to the nine fixed hardware candidates:

* the axial mount points along tibia ``-x`` (180 degrees from tibia ``+x``);
* every peripheral mount points radially inward (90 degrees from tibia ``+x``).

Therefore, none of the nine catalog attachments can enter the forbidden cone.
"""
from __future__ import annotations

import numpy as np

from transforms import homogeneous_transform, invert_transform, validate_transform

# Legacy constant retained only for backward compatibility with the existing
# 0.35 m rotation-only proposal pool. New attachment candidates must not use it.
LEGACY_TIBIA_OFFSET_M = 0.35

# New discrete attachment geometry.
AXIAL_INLINE_OFFSET_M = 0.31
PERIPHERAL_AXIAL_OFFSET_M = 0.28
PERIPHERAL_RADIAL_OFFSET_M = 0.03
PERIPHERAL_MOUNT_COUNT = 8
PERIPHERAL_AZIMUTH_STEP_DEG = 360.0 / PERIPHERAL_MOUNT_COUNT

# Legacy reference orientation: EE +z is tibia -x, i.e. toward the knee.
REFERENCE_R_TE = np.array([
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])

REFERENCE_MOUNTING_DIRECTION = np.array([-1.0, 0.0, 0.0])
FORBIDDEN_MOUNTING_DIRECTION = np.array([1.0, 0.0, 0.0])
EXCLUSION_HALF_ANGLE_RAD = np.deg2rad(45.0)

LEGACY_REFERENCE_X_TE = homogeneous_transform(
    rotation=REFERENCE_R_TE,
    translation=np.array([LEGACY_TIBIA_OFFSET_M, 0.0, 0.0]),
)


def xte_to_attachment(xte: np.ndarray) -> np.ndarray:
    """Return ``A = inverse(X_TE)``."""
    xte = np.asarray(xte, dtype=float)
    validate_transform(xte)
    return invert_transform(xte)


def attachment_to_xte(attachment: np.ndarray) -> np.ndarray:
    """Return ``X_TE = inverse(A)``."""
    attachment = np.asarray(attachment, dtype=float)
    validate_transform(attachment)
    return invert_transform(attachment)


def _rotation_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed tibia-axis XYZ rotation used by the legacy attachment API."""
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cx, -sx],
        [0.0, sx, cx],
    ])
    ry = np.array([
        [cy, 0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy],
    ])
    rz = np.array([
        [cz, -sz, 0.0],
        [sz, cz, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rz @ ry @ rx


def _direction_angle(xte: np.ndarray, direction: np.ndarray) -> float:
    xte = np.asarray(xte, dtype=float)
    validate_transform(xte)
    cosine = float(np.dot(direction, xte[:3, 2]))
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def mounting_deviation_rad(xte: np.ndarray) -> float:
    """Angle between EE ``+z`` and the reference direction, tibia ``-x``."""
    return _direction_angle(xte, REFERENCE_MOUNTING_DIRECTION)


def forbidden_direction_separation_rad(xte: np.ndarray) -> float:
    """Angle between EE ``+z`` and the forbidden central direction, tibia ``+x``."""
    return _direction_angle(xte, FORBIDDEN_MOUNTING_DIRECTION)


def orientation_is_admissible(
    xte: np.ndarray,
    exclusion_half_angle_rad: float = EXCLUSION_HALF_ANGLE_RAD,
) -> bool:
    """Optional 45-degree forbidden-cone filter.

    This rejects EE ``+z`` directions within ``exclusion_half_angle_rad`` of
    tibia ``+x``. It is a user-defined geometric/orientation filter, not a
    collision check.

    Do not apply this filter to the nine fixed attachment candidates: none of
    them lies inside the cone.
    """
    half_angle = float(exclusion_half_angle_rad)
    if not np.isfinite(half_angle) or not 0.0 < half_angle < np.pi:
        raise ValueError("exclusion_half_angle_rad must be between 0 and pi")

    separation = forbidden_direction_separation_rad(xte)
    return bool(separation >= half_angle - 1e-12)


def make_attachment_candidate(
    roll_angle_rad: float = 0.0,
    tibia_offset_m: float = LEGACY_TIBIA_OFFSET_M,
    *,
    pitch_angle_rad: float = 0.0,
    yaw_angle_rad: float = 0.0,
    enforce_admissibility: bool = True,
) -> np.ndarray:
    """Legacy rotation-only attachment constructor.

    .. deprecated::

        This function supports the existing 0.35 m rotation-only proposal pool
        and must not be used for the new nine-mount attachment catalog.

    It preserves the original API so current proposals, screening results, and
    evaluation artifacts remain reproducible.
    """
    angles = np.asarray(
        [roll_angle_rad, pitch_angle_rad, yaw_angle_rad], dtype=float
    )
    offset = float(tibia_offset_m)

    if not np.all(np.isfinite(angles)):
        raise ValueError("orientation angles must be finite")
    if not np.isfinite(offset) or offset <= 0.0:
        raise ValueError("tibia_offset_m must be finite and positive")

    rotation = _rotation_xyz(*angles) @ REFERENCE_R_TE
    xte = homogeneous_transform(
        rotation=rotation,
        translation=np.array([offset, 0.0, 0.0]),
    )

    if enforce_admissibility and not orientation_is_admissible(xte):
        separation_deg = np.rad2deg(
            forbidden_direction_separation_rad(xte)
        )
        raise ValueError(
            f"EE +z is only {separation_deg:.2f} deg from forbidden tibia +x; "
            "at least 45 deg separation is required"
        )

    return xte_to_attachment(xte)


def make_axial_inline_attachment() -> np.ndarray:
    """Return the axial in-line attachment ``A``.

    The EE origin is at tibia ``[0.31, 0, 0]`` m. EE ``+z`` points toward the
    knee, along tibia ``-x``.
    """
    xte = homogeneous_transform(
        rotation=REFERENCE_R_TE,
        translation=np.array([
            AXIAL_INLINE_OFFSET_M,
            0.0,
            0.0,
        ]),
    )
    return xte_to_attachment(xte)


def make_peripheral_attachment(azimuth_deg: float) -> np.ndarray:
    """Return one peripheral attachment ``A``.

    Parameters
    ----------
    azimuth_deg:
        Azimuth around the tibia cylinder, measured from tibia ``+y`` toward
        tibia ``+z``. Valid catalog values are 0, 45, 90, ..., 315 degrees.

    The EE origin is at:

        [0.28, 0.03 cos(theta), 0.03 sin(theta)] m

    EE ``+z`` points from the mount toward the tibia cylinder centerline.
    """
    azimuth_deg = float(azimuth_deg)
    if not np.isfinite(azimuth_deg):
        raise ValueError("azimuth_deg must be finite")

    theta = np.deg2rad(azimuth_deg)
    radial_offset = PERIPHERAL_RADIAL_OFFSET_M

    translation = np.array([
        PERIPHERAL_AXIAL_OFFSET_M,
        radial_offset * np.cos(theta),
        radial_offset * np.sin(theta),
    ])

    # EE +z is the inward radial direction.
    # EE +x is tibia +x.
    # EE +y completes the right-handed EE frame.
    rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -np.sin(theta), -np.cos(theta)],
        [0.0, np.cos(theta), -np.sin(theta)],
    ])

    xte = homogeneous_transform(
        rotation=rotation,
        translation=translation,
    )
    return xte_to_attachment(xte)

def attachment_origin_wrench_about_tibia(
    attachment: np.ndarray,
    wrench_at_attachment: np.ndarray,
) -> np.ndarray:
    """Convert an attachment-origin wrench into a tibia-origin wrench.

    Parameters
    ----------
    attachment:
        Attachment transform ``A = {}^E T_T``.
    wrench_at_attachment:
        Six-vector ``[Fx, Fy, Fz, Mx, My, Mz]`` expressed in the tibia frame
        ``T`` and applied at the attachment origin.

    Returns
    -------
    numpy.ndarray
        Equivalent six-vector wrench expressed in ``T`` and referred to the
        tibia-frame origin.

    The physical distal force is applied at the selected attachment origin.
    Its moment about the tibia-frame origin therefore depends on the
    attachment location:

        M_T = M_attachment + p_attachment_T x F_T

    This function does not change the force vector; it only moves the
    reference point about which the moment is expressed.
    """
    wrench_at_attachment = np.asarray(wrench_at_attachment, dtype=float)

    if wrench_at_attachment.shape != (6,) or not np.all(
        np.isfinite(wrench_at_attachment)
    ):
        raise ValueError(
            "wrench_at_attachment must be a finite length-6 vector"
        )

    attachment = np.asarray(attachment, dtype=float)
    validate_transform(attachment)

    xte = attachment_to_xte(attachment)
    attachment_origin_t = xte[:3, 3]

    force_t = wrench_at_attachment[:3]
    moment_at_attachment_t = wrench_at_attachment[3:]

    moment_about_t_origin = (
        moment_at_attachment_t
        + np.cross(attachment_origin_t, force_t)
    )

    return np.concatenate((force_t, moment_about_t_origin))


def make_attachment_catalog() -> list[dict]:
    """Return the nine equally important attachment candidates.

    The ordering is deterministic and is only an indexing convention. It does
    not imply that one attachment is primary or preferred.
    """
    catalog = []

    axial_a = make_axial_inline_attachment()
    axial_xte = attachment_to_xte(axial_a)

    catalog.append({
        "attachment_id": "axial_inline",
        "attachment_type": "AXIAL_INLINE",
        "axial_offset_m": AXIAL_INLINE_OFFSET_M,
        "radial_offset_m": 0.0,
        "azimuth_deg": None,
        "A": axial_a,
        "X_TE": axial_xte,
    })

    for index in range(PERIPHERAL_MOUNT_COUNT):
        azimuth_deg = index * PERIPHERAL_AZIMUTH_STEP_DEG
        peripheral_a = make_peripheral_attachment(azimuth_deg)
        peripheral_xte = attachment_to_xte(peripheral_a)

        catalog.append({
            "attachment_id": f"peripheral_{index:02d}",
            "attachment_type": "PERIPHERAL_RING",
            "axial_offset_m": PERIPHERAL_AXIAL_OFFSET_M,
            "radial_offset_m": PERIPHERAL_RADIAL_OFFSET_M,
            "azimuth_deg": azimuth_deg,
            "A": peripheral_a,
            "X_TE": peripheral_xte,
        })

    return catalog


def required_ee_transform(
    tibia_transform_in_base: np.ndarray,
    attachment: np.ndarray,
) -> np.ndarray:
    """Return the required EE pose in the robot base frame.

    ``X_BE_required = X_BT @ X_TE``
    """
    tibia_transform_in_base = np.asarray(
        tibia_transform_in_base, dtype=float
    )
    attachment = np.asarray(attachment, dtype=float)
    validate_transform(tibia_transform_in_base)
    validate_transform(attachment)

    return tibia_transform_in_base @ attachment_to_xte(attachment)
