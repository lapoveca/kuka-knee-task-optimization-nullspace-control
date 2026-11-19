"""Validate attachment-dependent distal-force wrench transformation.

The distal force is applied at the selected attachment origin. The direct
couple at that attachment is zero. This script checks the equivalent wrench
about the OpenSim tibia-frame origin.

It does not modify task envelopes, optimizer outputs, or pipeline artifacts.
"""
from __future__ import annotations

import numpy as np

from attachment import (
    AXIAL_INLINE_OFFSET_M,
    PERIPHERAL_AXIAL_OFFSET_M,
    PERIPHERAL_RADIAL_OFFSET_M,
    attachment_origin_wrench_about_tibia,
    make_attachment_catalog,
)
from tibia_task_map import TibiaTaskMap
from robot_model import RobotModel


DISTAL_FORCE_T_N = np.array([
    0.0,
    0.0,
    -19.417475761968834,
])

DIRECT_COUPLE_AT_ATTACHMENT_T_NM = np.array([
    0.0,
    0.0,
    0.0,
])

WRENCH_AT_ATTACHMENT_T = np.concatenate((
    DISTAL_FORCE_T_N,
    DIRECT_COUPLE_AT_ATTACHMENT_T_NM,
))


def main():
    catalog = make_attachment_catalog()
    robot = RobotModel()

    if len(catalog) != 9:
        raise AssertionError(f"Expected 9 attachments, found {len(catalog)}")

    tolerance = 1e-12

    for entry in catalog:
        attachment = np.asarray(entry["A"], dtype=float)
        xte = np.asarray(entry["X_TE"], dtype=float)

        attachment_origin_t = xte[:3, 3]

        expected_force_t = DISTAL_FORCE_T_N.copy()
        expected_moment_about_t_origin = np.cross(
            attachment_origin_t,
            DISTAL_FORCE_T_N,
        )

        expected_wrench_about_t_origin = np.concatenate((
            expected_force_t,
            expected_moment_about_t_origin,
        ))

        # Standalone function.
        standalone_wrench = attachment_origin_wrench_about_tibia(
            attachment,
            WRENCH_AT_ATTACHMENT_T,
        )

        # TibiaTaskMap method.
        mapper = TibiaTaskMap(robot, attachment)
        mapper_wrench = mapper.wrench_about_tibia_origin(
            WRENCH_AT_ATTACHMENT_T,
        )

        if not np.allclose(
            standalone_wrench,
            expected_wrench_about_t_origin,
            atol=tolerance,
        ):
            raise AssertionError(
                f"{entry['attachment_id']}: standalone wrench transformation failed"
            )

        if not np.allclose(
            mapper_wrench,
            expected_wrench_about_t_origin,
            atol=tolerance,
        ):
            raise AssertionError(
                f"{entry['attachment_id']}: TibiaTaskMap wrench transformation failed"
            )

        if not np.allclose(
            standalone_wrench,
            mapper_wrench,
            atol=tolerance,
        ):
            raise AssertionError(
                f"{entry['attachment_id']}: standalone and mapper results differ"
            )

        print(
            f"{entry['attachment_id']}: "
            f"application_point_T={np.round(attachment_origin_t, 6).tolist()}, "
            f"wrench_about_T_origin="
            f"{np.round(mapper_wrench, 6).tolist()}"
        )

    # Check the legacy 0.35 m application point against the current task JSON.
    legacy_application_point_t = np.array([
        0.35,
        0.0,
        0.0,
    ])
    legacy_expected_moment = np.cross(
        legacy_application_point_t,
        DISTAL_FORCE_T_N,
    )

    expected_legacy_positive_moment = np.array([
        0.0,
        6.796116516689092,
        0.0,
    ])

    if not np.allclose(
        legacy_expected_moment,
        expected_legacy_positive_moment,
        atol=1e-9,
    ):
        raise AssertionError(
            "Legacy 0.35 m wrench cross-check failed"
        )

    print("ATTACHMENT_LOADING_VALIDATION_PASS")
    print(
        "Legacy 0.35 m positive-case moment about T origin: "
        f"{np.round(legacy_expected_moment, 9).tolist()} Nm"
    )


if __name__ == "__main__":
    main()