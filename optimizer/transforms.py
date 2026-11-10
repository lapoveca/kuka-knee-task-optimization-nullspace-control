"""Rigid-transform, twist, and wrench utilities.

All homogeneous transforms use the convention

    T_AC maps coordinates expressed in frame C into frame A:

        p_A = T_AC @ p_C

A transform has the block form

    T_AC = [[R_AC, p_AC],
            [0,    1]]

where R_AC is the orientation of C expressed in A and p_AC is the
position of C's origin expressed in A.

Twists use the ordering

    [linear velocity, angular velocity]

Wrenches use the ordering

    [force, moment]

A twist and wrench are paired only when they are expressed about the
same reference point and in the same coordinate frame.

This module contains no robot-model, OpenSim, ROS, optimization,
or capability calculations.
"""

from __future__ import annotations

import numpy as np


def homogeneous_transform(
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> np.ndarray:
    """Create a homogeneous transform.

    Parameters
    ----------
    rotation:
        Optional ``(3, 3)`` rotation matrix.
    translation:
        Optional length-3 translation vector.

    Returns
    -------
    np.ndarray
        A ``(4, 4)`` homogeneous transform.
    """
    transform = np.eye(4)

    if rotation is not None:
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3):
            raise ValueError(
                f"rotation must have shape (3, 3), got {rotation.shape}"
            )
        transform[:3, :3] = rotation

    if translation is not None:
        translation = np.asarray(translation, dtype=float).reshape(3)
        transform[:3, 3] = translation

    validate_transform(transform)
    return transform


def identity_transform() -> np.ndarray:
    """Return a valid identity homogeneous transform."""
    return np.eye(4)


def validate_transform(transform: np.ndarray, atol: float = 1e-8) -> None:
    """Validate that an array is a proper rigid homogeneous transform.

    Parameters
    ----------
    transform:
        Candidate ``(4, 4)`` transform.
    atol:
        Absolute numerical tolerance.

    Raises
    ------
    ValueError
        If the input is not a valid rigid transform.
    """
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        raise ValueError(
            f"transform must have shape (4, 4), got {transform.shape}"
        )

    if not np.allclose(transform[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=atol):
        raise ValueError("last row must be [0, 0, 0, 1]")

    rotation = transform[:3, :3]

    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError("rotation block is not orthogonal")

    if not np.isclose(np.linalg.det(rotation), 1.0, atol=atol):
        raise ValueError(
            f"rotation determinant must be 1, got {np.linalg.det(rotation)}"
        )


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid homogeneous transform.

    For

        T_AC = [[R, p],
                [0, 1]]

    the inverse is

        T_CA = [[R.T, -R.T @ p],
                [0,     1     ]]

    Parameters
    ----------
    transform:
        A valid ``(4, 4)`` rigid transform.

    Returns
    -------
    np.ndarray
        The inverse transform.
    """
    transform = np.asarray(transform, dtype=float)
    validate_transform(transform)

    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse


def compose_transforms(
    first: np.ndarray,
    second: np.ndarray,
) -> np.ndarray:
    """Compose two homogeneous transforms.

    With the project convention,

        T_AC @ T_CD = T_AD

    Parameters
    ----------
    first:
        Transform from frame `C` to frame `A`.
    second:
        Transform from frame `D` to frame `C`.

    Returns
    -------
    np.ndarray
        Transform from frame `D` to frame `A`.
    """
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)

    validate_transform(first)
    validate_transform(second)

    return first @ second


def transform_point(
    transform: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    """Transform a 3D point using a homogeneous transform.

    Parameters
    ----------
    transform:
        A valid ``(4, 4)`` rigid transform.
    point:
        Length-3 point expressed in the child frame.

    Returns
    -------
    np.ndarray
        Length-3 point expressed in the parent frame.
    """
    transform = np.asarray(transform, dtype=float)
    point = np.asarray(point, dtype=float).reshape(3)

    validate_transform(transform)

    return transform[:3, :3] @ point + transform[:3, 3]


def relative_transform(
    transform_parent_to_child: np.ndarray,
    transform_child_to_grandchild: np.ndarray,
) -> np.ndarray:
    """Return the transform from parent to grandchild.

    This is a named wrapper around composition:

        T_parent_grandchild
        =
        T_parent_child @ T_child_grandchild

    Parameters
    ----------
    transform_parent_to_child:
        Transform from child to parent.
    transform_child_to_grandchild:
        Transform from grandchild to child.

    Returns
    -------
    np.ndarray
        Transform from grandchild to parent.
    """
    return compose_transforms(
        transform_parent_to_child,
        transform_child_to_grandchild,
    )


def rotation_about_z(angle_rad: float) -> np.ndarray:
    """Return a right-handed rotation matrix about the z axis.

    Parameters
    ----------
    angle_rad:
        Rotation angle in radians.

    Returns
    -------
    np.ndarray
        A ``(3, 3)`` rotation matrix.
    """
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)

    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix such that skew(v) @ x = v x x.

    Parameters
    ----------
    vector:
        Length-3 vector.

    Returns
    -------
    np.ndarray
        A ``(3, 3)`` skew-symmetric matrix.
    """
    vector = np.asarray(vector, dtype=float).reshape(3)

    return np.array(
        [
            [0.0, -vector[2], vector[1]],
            [vector[2], 0.0, -vector[0]],
            [-vector[1], vector[0], 0.0],
        ]
    )


def rotate_twist(
    rotation: np.ndarray,
    twist: np.ndarray,
) -> np.ndarray:
    """Change only the coordinate axes of a twist.

    The twist is assumed to be defined at the same physical reference
    point in both coordinate frames.

    With twist ordering

        [linear velocity, angular velocity]

    the transformation is

        v_parent = R v_child
        omega_parent = R omega_child

    Parameters
    ----------
    rotation:
        Rotation from child coordinates to parent coordinates.
    twist:
        Length-6 twist expressed in child coordinates.

    Returns
    -------
    np.ndarray
        Length-6 twist expressed in parent coordinates.
    """
    rotation = np.asarray(rotation, dtype=float)
    twist = np.asarray(twist, dtype=float).reshape(6)

    if rotation.shape != (3, 3):
        raise ValueError(
            f"rotation must have shape (3, 3), got {rotation.shape}"
        )

    linear_child = twist[:3]
    angular_child = twist[3:]

    linear_parent = rotation @ linear_child
    angular_parent = rotation @ angular_child

    return np.concatenate((linear_parent, angular_parent))


def rotate_wrench(
    rotation: np.ndarray,
    wrench: np.ndarray,
) -> np.ndarray:
    """Change only the coordinate axes of a wrench.

    The wrench is assumed to be defined about the same physical
    reference point in both coordinate frames.

    With wrench ordering

        [force, moment]

    the transformation is

        f_parent = R f_child
        m_parent = R m_child

    Parameters
    ----------
    rotation:
        Rotation from child coordinates to parent coordinates.
    wrench:
        Length-6 wrench expressed in child coordinates.

    Returns
    -------
    np.ndarray
        Length-6 wrench expressed in parent coordinates.
    """
    rotation = np.asarray(rotation, dtype=float)
    wrench = np.asarray(wrench, dtype=float).reshape(6)

    if rotation.shape != (3, 3):
        raise ValueError(
            f"rotation must have shape (3, 3), got {rotation.shape}"
        )

    force_child = wrench[:3]
    moment_child = wrench[3:]

    force_parent = rotation @ force_child
    moment_parent = rotation @ moment_child

    return np.concatenate((force_parent, moment_parent))


def shift_twist_reference_point(
    twist: np.ndarray,
    offset: np.ndarray,
) -> np.ndarray:
    """Move the reference point of a twist on the same rigid body.

    Let `offset` point from the old reference point to the new
    reference point, expressed in the same coordinate frame.

    The angular velocity is unchanged. The linear velocity at the new
    point is

        v_new = v_old + omega x offset

    Parameters
    ----------
    twist:
        Length-6 twist at the old reference point.
    offset:
        Length-3 vector from the old reference point to the new
        reference point.

    Returns
    -------
    np.ndarray
        Length-6 twist at the new reference point.
    """
    twist = np.asarray(twist, dtype=float).reshape(6)
    offset = np.asarray(offset, dtype=float).reshape(3)

    linear_old = twist[:3]
    angular = twist[3:]

    linear_new = linear_old + np.cross(angular, offset)

    return np.concatenate((linear_new, angular))


def shift_wrench_reference_point(
    wrench: np.ndarray,
    offset: np.ndarray,
) -> np.ndarray:
    """Move the reference point of a wrench on the same rigid body.

    Let `offset` point from the old reference point to the new
    reference point, expressed in the same coordinate frame.

    The force is unchanged. The moment about the new point is

        m_new = m_old - offset x f

    Parameters
    ----------
    wrench:
        Length-6 wrench at the old reference point.
    offset:
        Length-3 vector from the old reference point to the new
        reference point.

    Returns
    -------
    np.ndarray
        Length-6 wrench at the new reference point.
    """
    wrench = np.asarray(wrench, dtype=float).reshape(6)
    offset = np.asarray(offset, dtype=float).reshape(3)

    force = wrench[:3]
    moment_old = wrench[3:]

    moment_new = moment_old - np.cross(offset, force)

    return np.concatenate((force, moment_new))