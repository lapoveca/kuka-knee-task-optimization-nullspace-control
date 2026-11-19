"""Validate the nine discrete tibia attachment candidates.

This script is read-only with respect to optimizer outputs. It validates
geometry and, optionally, creates a visualization in its own output directory.
"""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from attachment import (
    AXIAL_INLINE_OFFSET_M,
    PERIPHERAL_AXIAL_OFFSET_M,
    PERIPHERAL_RADIAL_OFFSET_M,
    attachment_to_xte,
    forbidden_direction_separation_rad,
    make_attachment_catalog,
)


def homogeneous_to_numpy(transform) -> np.ndarray:
    return np.asarray(transform, dtype=float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Create a 3D visualization in output/attachment_validation_<timestamp>/",
    )
    args = parser.parse_args()

    catalog = make_attachment_catalog()

    if len(catalog) != 9:
        raise AssertionError(f"Expected 9 attachments, found {len(catalog)}")

    expected_ids = ["axial_inline"] + [
        f"peripheral_{index:02d}" for index in range(8)
    ]
    actual_ids = [entry["attachment_id"] for entry in catalog]
    if actual_ids != expected_ids:
        raise AssertionError(f"Unexpected attachment IDs: {actual_ids}")

    tolerance = 1e-12

    for entry in catalog:
        attachment = homogeneous_to_numpy(entry["A"])
        xte = homogeneous_to_numpy(entry["X_TE"])

        # A and X_TE must be exact inverses.
        if not np.allclose(attachment @ xte, np.eye(4), atol=tolerance):
            raise AssertionError(
                f"{entry['attachment_id']}: A @ X_TE is not identity"
            )
        if not np.allclose(xte @ attachment, np.eye(4), atol=tolerance):
            raise AssertionError(
                f"{entry['attachment_id']}: X_TE @ A is not identity"
            )

        # Both transforms must be valid rigid transforms.
        for transform in (attachment, xte):
            if not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-10):
                raise AssertionError(
                    f"{entry['attachment_id']}: rotation is not orthonormal"
                )
            if abs(np.linalg.det(transform[:3, :3]) - 1.0) > 1e-10:
                raise AssertionError(
                    f"{entry['attachment_id']}: rotation determinant is not 1"
                )
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=tolerance):
                raise AssertionError(
                    f"{entry['attachment_id']}: homogeneous bottom row is invalid"
                )

        origin_t = xte[:3, 3]
        ee_z_t = xte[:3, 2]

        if entry["attachment_type"] == "AXIAL_INLINE":
            expected_origin = np.array([
                AXIAL_INLINE_OFFSET_M,
                0.0,
                0.0,
            ])
            expected_ee_z = np.array([-1.0, 0.0, 0.0])

            if not np.allclose(origin_t, expected_origin, atol=tolerance):
                raise AssertionError(
                    f"{entry['attachment_id']}: incorrect axial origin"
                )
            if not np.allclose(ee_z_t, expected_ee_z, atol=tolerance):
                raise AssertionError(
                    f"{entry['attachment_id']}: EE +z does not point toward knee"
                )

            separation_deg = np.rad2deg(
                forbidden_direction_separation_rad(xte)
            )
            if abs(separation_deg - 180.0) > 1e-9:
                raise AssertionError(
                    f"{entry['attachment_id']}: unexpected forbidden-cone separation"
                )

        elif entry["attachment_type"] == "PERIPHERAL_RING":
            azimuth_rad = np.deg2rad(entry["azimuth_deg"])
            expected_origin = np.array([
                PERIPHERAL_AXIAL_OFFSET_M,
                PERIPHERAL_RADIAL_OFFSET_M * np.cos(azimuth_rad),
                PERIPHERAL_RADIAL_OFFSET_M * np.sin(azimuth_rad),
            ])
            radial_outward = np.array([
                0.0,
                np.cos(azimuth_rad),
                np.sin(azimuth_rad),
            ])
            expected_ee_z = -radial_outward

            if not np.allclose(origin_t, expected_origin, atol=tolerance):
                raise AssertionError(
                    f"{entry['attachment_id']}: incorrect peripheral origin"
                )
            if not np.allclose(ee_z_t, expected_ee_z, atol=tolerance):
                raise AssertionError(
                    f"{entry['attachment_id']}: EE +z does not point toward centerline"
                )

            radial_distance = np.linalg.norm(origin_t[1:])
            if abs(radial_distance - PERIPHERAL_RADIAL_OFFSET_M) > tolerance:
                raise AssertionError(
                    f"{entry['attachment_id']}: incorrect radial offset"
                )

            separation_deg = np.rad2deg(
                forbidden_direction_separation_rad(xte)
            )
            if abs(separation_deg - 90.0) > 1e-9:
                raise AssertionError(
                    f"{entry['attachment_id']}: unexpected forbidden-cone separation"
                )

        else:
            raise AssertionError(
                f"{entry['attachment_id']}: unknown attachment type"
            )

    print("ATTACHMENT_CATALOG_VALIDATION_PASS")
    print(f"attachments: {len(catalog)}")

    for entry in catalog:
        xte = homogeneous_to_numpy(entry["X_TE"])
        origin = xte[:3, 3]
        ee_z = xte[:3, 2]
        separation = np.rad2deg(
            forbidden_direction_separation_rad(xte)
        )
        print(
            f"{entry['attachment_id']}: "
            f"origin_T={np.round(origin, 6).tolist()}, "
            f"ee_z_T={np.round(ee_z, 6).tolist()}, "
            f"forbidden_cone_separation_deg={separation:.3f}"
        )

    if args.plot:
        plot_dir = Path("output") / (
            f"attachment_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        plot_dir.mkdir(parents=True, exist_ok=True)

        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        cylinder_radius_m = 0.03
        cylinder_length_m = 0.31
        arrow_length_m = 0.0225

        mount_colors = {
            "axial_inline": "tab:red",
            "peripheral_00": "tab:blue",
            "peripheral_01": "tab:orange",
            "peripheral_02": "tab:green",
            "peripheral_03": "tab:purple",
            "peripheral_04": "tab:brown",
            "peripheral_05": "tab:pink",
            "peripheral_06": "tab:gray",
            "peripheral_07": "tab:olive",
        }

        def draw_cylinder(axis):
            theta = np.linspace(0.0, 2.0 * np.pi, 48)
            x = np.array([0.0, cylinder_length_m])
            theta_grid, x_grid = np.meshgrid(theta, x)
            y_grid = cylinder_radius_m * np.cos(theta_grid)
            z_grid = cylinder_radius_m * np.sin(theta_grid)

            axis.plot_surface(
                x_grid,
                y_grid,
                z_grid,
                alpha=0.18,
                color="lightsteelblue",
                linewidth=0,
                antialiased=True,
            )

            # End faces.
            for x_face in (0.0, cylinder_length_m):
                angle = np.linspace(0.0, 2.0 * np.pi, 100)
                face_y = cylinder_radius_m * np.cos(angle)
                face_z = cylinder_radius_m * np.sin(angle)
                axis.plot(
                    x_face * np.ones_like(face_y),
                    face_y,
                    face_z,
                    color="steelblue",
                    linewidth=1.0,
                    alpha=0.7,
                )

            # Centerline.
            axis.plot(
                [0.0, cylinder_length_m],
                [0.0, 0.0],
                [0.0, 0.0],
                color="black",
                linewidth=1.5,
                linestyle="--",
                label="Tibia centerline",
            )

        def annotate_mount(axis, entry, offset):
            xte = homogeneous_to_numpy(entry["X_TE"])
            origin = xte[:3, 3]
            ee_z = xte[:3, 2]
            color = mount_colors[entry["attachment_id"]]

            axis.scatter(
                origin[0],
                origin[1],
                origin[2],
                color=color,
                s=55,
                depthshade=False,
                label=entry["attachment_id"],
            )

            axis.quiver(
                origin[0],
                origin[1],
                origin[2],
                arrow_length_m * ee_z[0],
                arrow_length_m * ee_z[1],
                arrow_length_m * ee_z[2],
                color=color,
                linewidth=2.2,
                arrow_length_ratio=0.18,
            )

            label_position = origin + offset
            axis.text(
                label_position[0],
                label_position[1],
                label_position[2],
                entry["attachment_id"],
                color=color,
                fontsize=8,
                weight="bold",
            )

        # ---------------------------- 3D view ----------------------------
        figure_3d = plt.figure(figsize=(12, 8))
        axis_3d = figure_3d.add_subplot(projection="3d")

        draw_cylinder(axis_3d)

        axis_3d.quiver(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            arrow_length_m,
            color="black",
            linewidth=2.0,
            arrow_length_ratio=0.18,
            label="EE +z axis (colored arrows)",
        )

        for entry in catalog:
            if entry["attachment_id"] == "axial_inline":
                offset = np.array([0.015, 0.0, 0.05])
            else:
                azimuth_rad = np.deg2rad(entry["azimuth_deg"])
                offset = 0.045 * np.array([
                    0.0,
                    np.cos(azimuth_rad),
                    np.sin(azimuth_rad),
                ])
            annotate_mount(axis_3d, entry, offset)

        axis_3d.set_xlim(-0.08, 0.40)
        axis_3d.set_ylim(-0.10, 0.10)
        axis_3d.set_zlim(-0.10, 0.10)
        axis_3d.set_box_aspect((0.48, 0.20, 0.20))
        axis_3d.set_xlabel("Tibia x [m]\n(knee → ankle)")
        axis_3d.set_ylabel("Tibia y [m]")
        axis_3d.set_zlabel("Tibia z [m]")
        axis_3d.set_title(
            "Nine tibia attachment candidates\n"
            "Colored arrows: end-effector +z axis (EE +z)"
)
        axis_3d.legend(
            loc="upper left",
            bbox_to_anchor=(1.02, 1.0),
            fontsize=8,
            framealpha=0.9,
        )
        axis_3d.view_init(elev=18, azim=-58)

        figure_3d_path = plot_dir / "attachment_catalog_3d.png"
        figure_3d.savefig(figure_3d_path, dpi=220, bbox_inches="tight")

        # ------------------------ 2D: x-y view ---------------------------
        figure_xy, axis_xy = plt.subplots(figsize=(11, 5))
        axis_xy.add_patch(
            Circle(
                (0.155, 0.0),
                cylinder_radius_m,
                facecolor="lightsteelblue",
                edgecolor="steelblue",
                alpha=0.35,
                linewidth=1.5,
                label="Tibia cylinder cross-section",
            )
        )

        axis_xy.plot(
            [0.0, cylinder_length_m],
            [0.0, 0.0],
            color="black",
            linewidth=1.5,
            linestyle="--",
            label="Tibia centerline",
        )

        for entry in catalog:
            xte = homogeneous_to_numpy(entry["X_TE"])
            origin = xte[:3, 3]
            ee_z = xte[:3, 2]
            color = mount_colors[entry["attachment_id"]]

            axis_xy.scatter(
                origin[0],
                origin[1],
                color=color,
                s=60,
                zorder=5,
                label=entry["attachment_id"],
            )
            axis_xy.quiver(
                origin[0],
                origin[1],
                arrow_length_m * ee_z[0],
                arrow_length_m * ee_z[1],
                angles="xy",
                scale_units="xy",
                scale=1.0,
                color=color,
                width=0.004,
                zorder=6,
            )

            if entry["attachment_id"] == "axial_inline":
                text_offset = (0.008, 0.012)
            else:
                text_offset = (
                    0.004 * np.sign(ee_z[1]),
                    0.008 * np.sign(ee_z[1]),
                )

            axis_xy.annotate(
                entry["attachment_id"],
                (origin[0], origin[1]),
                xytext=(origin[0] + text_offset[0], origin[1] + text_offset[1]),
                fontsize=8,
                color=color,
                weight="bold",
            )

        axis_xy.set_xlim(-0.10, 0.40)
        axis_xy.set_ylim(-0.10, 0.10)
        axis_xy.set_aspect("equal", adjustable="box")
        axis_xy.set_xlabel("Tibia x [m]\n(knee → ankle)")
        axis_xy.set_ylabel("Tibia y [m]")
        axis_xy.set_title(
            "Attachment positions and EE +z directions: x-y view\n"
            "Colored arrows: end-effector +z axis (EE +z)"
        )
        axis_xy.grid(True, alpha=0.3)
        axis_xy.legend(
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            fontsize=8,
            framealpha=0.9,
        )

        figure_xy_path = plot_dir / "attachment_catalog_xy.png"
        figure_xy.savefig(figure_xy_path, dpi=220, bbox_inches="tight")

        # ------------------------ 2D: x-z view ---------------------------
        figure_xz, axis_xz = plt.subplots(figsize=(11, 5))
        axis_xz.add_patch(
            Circle(
                (0.155, 0.0),
                cylinder_radius_m,
                facecolor="lightsteelblue",
                edgecolor="steelblue",
                alpha=0.35,
                linewidth=1.5,
                label="Tibia cylinder cross-section",
            )
        )

        axis_xz.plot(
            [0.0, cylinder_length_m],
            [0.0, 0.0],
            color="black",
            linewidth=1.5,
            linestyle="--",
            label="Tibia centerline",
        )

        for entry in catalog:
            xte = homogeneous_to_numpy(entry["X_TE"])
            origin = xte[:3, 3]
            ee_z = xte[:3, 2]
            color = mount_colors[entry["attachment_id"]]

            axis_xz.scatter(
                origin[0],
                origin[2],
                color=color,
                s=60,
                zorder=5,
                label=entry["attachment_id"],
            )
            axis_xz.quiver(
                origin[0],
                origin[2],
                arrow_length_m * ee_z[0],
                arrow_length_m * ee_z[2],
                angles="xy",
                scale_units="xy",
                scale=1.0,
                color=color,
                width=0.004,
                zorder=6,
            )

            if entry["attachment_id"] == "axial_inline":
                text_offset = (0.008, 0.012)
            else:
                text_offset = (
                    0.004 * np.sign(ee_z[2]),
                    0.008 * np.sign(ee_z[2]),
                )

            axis_xz.annotate(
                entry["attachment_id"],
                (origin[0], origin[2]),
                xytext=(origin[0] + text_offset[0], origin[2] + text_offset[1]),
                fontsize=8,
                color=color,
                weight="bold",
            )

        axis_xz.set_xlim(-0.10, 0.40)
        axis_xz.set_ylim(-0.10, 0.10)
        axis_xz.set_aspect("equal", adjustable="box")
        axis_xz.set_xlabel("Tibia x [m]\n(knee → ankle)")
        axis_xz.set_ylabel("Tibia z [m]")
        axis_xz.set_title(
            "Attachment positions and EE +z directions: x-z view\n"
            "Colored arrows: end-effector +z axis (EE +z)"
        )
        axis_xz.grid(True, alpha=0.3)
        axis_xz.legend(
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            fontsize=8,
            framealpha=0.9,
        )

        figure_xz_path = plot_dir / "attachment_catalog_xz.png"
        figure_xz.savefig(figure_xz_path, dpi=220, bbox_inches="tight")

        # ------------------------ 2D: y-z view ---------------------------
        figure_yz, axis_yz = plt.subplots(figsize=(7, 7))
        axis_yz.add_patch(
            Circle(
                (0.0, 0.0),
                cylinder_radius_m,
                facecolor="lightsteelblue",
                edgecolor="steelblue",
                alpha=0.35,
                linewidth=1.5,
                label="Tibia cylinder cross-section",
            )
        )

        for entry in catalog:
            if entry["attachment_type"] != "PERIPHERAL_RING":
                continue

            xte = homogeneous_to_numpy(entry["X_TE"])
            origin = xte[:3, 3]
            ee_z = xte[:3, 2]
            color = mount_colors[entry["attachment_id"]]

            axis_yz.scatter(
                origin[1],
                origin[2],
                color=color,
                s=60,
                zorder=5,
                label=entry["attachment_id"],
            )
            axis_yz.quiver(
                origin[1],
                origin[2],
                arrow_length_m * ee_z[1],
                arrow_length_m * ee_z[2],
                angles="xy",
                scale_units="xy",
                scale=1.0,
                color=color,
                width=0.006,
                zorder=6,
            )

            text_offset = 0.012 * np.array([
                ee_z[1],
                ee_z[2],
            ])
            axis_yz.annotate(
                entry["attachment_id"],
                (origin[1], origin[2]),
                xytext=(
                    origin[1] + text_offset[0],
                    origin[2] + text_offset[1],
                ),
                fontsize=8,
                color=color,
                weight="bold",
                ha="center",
                va="center",
            )

        axis_yz.set_xlim(-0.10, 0.10)
        axis_yz.set_ylim(-0.10, 0.10)
        axis_yz.set_aspect("equal", adjustable="box")
        axis_yz.set_xlabel("Tibia y [m]")
        axis_yz.set_ylabel("Tibia z [m]")
        axis_yz.set_title(
            "Peripheral attachments: y-z cross-section\n"
            "Colored arrows: EE +z axis, pointing toward cylinder centerline"
        )
        axis_yz.grid(True, alpha=0.3)
        axis_yz.legend(
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            fontsize=8,
            framealpha=0.9,
        )

        figure_yz_path = plot_dir / "attachment_catalog_yz.png"
        figure_yz.savefig(figure_yz_path, dpi=220, bbox_inches="tight")

        print(f"visualization directory: {plot_dir}")
        print(f"3D view: {figure_3d_path}")
        print(f"x-y view: {figure_xy_path}")
        print(f"x-z view: {figure_xz_path}")
        print(f"y-z view: {figure_yz_path}")

        # Open interactive GUI windows for inspection.
        plt.show()

if __name__ == "__main__":
    main()