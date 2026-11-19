#!/usr/bin/env python3
"""Read-only 3D campaign atlas for a completed ranked VV campaign.

Run from optimizer/:

    python3 review_vv_campaign.py CAMPAIGN_DIR OUTPUT_DIR --top 5

The campaign artifacts are read-only. The output directory must be new and
outside the campaign directory.
"""

import argparse
import csv
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation

from attachment import make_attachment_catalog
from robot_model import RobotModel
from task_envelope import TaskEnvelope
from tibia_collision import TibiaCollisionChecker
from transforms import validate_transform


ARM_COLORS = {
    "+X": "#1f77b4",
    "-X": "#d62728",
    "+Y": "#2ca02c",
    "-Y": "#9467bd",
}


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def matrix(value):
    value = np.asarray(value, dtype=float)
    require(
        value.shape == (4, 4) and np.isfinite(value).all(),
        "Expected a finite 4x4 transform",
    )
    validate_transform(value)
    return value


def move_points(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def make_cylinder():
    theta = np.linspace(0.0, 2.0 * np.pi, 49)
    faces = []

    for first, second in zip(theta[:-1], theta[1:]):
        p = np.array([
            [0.0, 0.03 * np.cos(first), 0.03 * np.sin(first)],
            [0.31, 0.03 * np.cos(first), 0.03 * np.sin(first)],
            [0.31, 0.03 * np.cos(second), 0.03 * np.sin(second)],
            [0.0, 0.03 * np.cos(second), 0.03 * np.sin(second)],
        ])
        faces.extend([
            p[[0, 1, 2]],
            p[[0, 2, 3]],
            np.array([[0, 0, 0], p[0], p[3]]),
            np.array([[0.31, 0, 0], p[2], p[1]]),
        ])

    return np.asarray(faces)


def read_stl(path):
    raw = Path(path).read_bytes()

    if len(raw) >= 84:
        triangle_count = struct.unpack_from("<I", raw, 80)[0]
        expected_size = 84 + triangle_count * 50

        if len(raw) == expected_size:
            dtype = np.dtype([
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ])
            triangles = np.frombuffer(
                raw,
                dtype=dtype,
                count=triangle_count,
                offset=84,
            )["vertices"].astype(float)
            return triangles

    vertices = []

    for line in raw.decode("ascii").splitlines():
        words = line.split()

        if words and words[0].lower() == "vertex":
            require(len(words) == 4, "Invalid ASCII STL vertex")
            vertices.append([float(value) for value in words[1:]])

    triangles = np.asarray(vertices, dtype=float).reshape(-1, 3, 3)
    require(len(triangles) > 0, f"Empty STL file: {path}")
    return triangles


def draw_frame(axis, transform, label, length=0.06, dashed=False):
    origin = transform[:3, 3]

    for index, color in enumerate(("red", "green", "blue")):
        end = origin + length * transform[:3, index]
        axis.plot(
            *np.stack([origin, end]).T,
            color=color,
            linestyle="--" if dashed else "-",
            linewidth=1.7,
        )

    axis.text(*origin, label, fontsize=7)


def equal_limits(axis, points):
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    center = (lower + upper) / 2
    half = max(float(np.max(upper - lower)) * 0.57, 0.06)

    axis.set_xlim(center[0] - half, center[0] + half)
    axis.set_ylim(center[1] - half, center[1] + half)
    axis.set_zlim(center[2] - half, center[2] + half)
    axis.set_box_aspect((1, 1, 1))
    axis.set_proj_type("ortho")
    axis.view_init(24, -55)


def save_figure(figure, output_dir, stem):
    for extension in ("png", "pdf"):
        figure.savefig(
            output_dir / f"{stem}.{extension}",
            bbox_inches="tight",
            dpi=200,
        )

    plt.close(figure)


def horizontal_bars(axis, labels, values, title):
    bars = axis.barh(labels, values, color="#39768c")
    axis.bar_label(bars, padding=4)
    axis.invert_yaxis()
    axis.margins(x=0.16)
    axis.set_title(title)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--top", type=int, default=5)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--no-gif", action="store_true")
    arguments = parser.parse_args()

    campaign = arguments.campaign.resolve()
    output = arguments.output.resolve()

    require(arguments.top > 0, "--top must be positive")
    require(not output.exists(), f"Output directory already exists: {output}")
    require(
        not output.is_relative_to(campaign),
        "Output directory must be outside the campaign directory",
    )

    manifest = read_json(campaign / "campaign_manifest.json")
    summary = read_json(campaign / "campaign_summary.json")
    require(
        summary["status"] == "complete",
        "Expected a completed ranked campaign",
    )

    ranking = campaign / "ranking_guarded"
    ranking_manifest = read_json(ranking / "ranking_manifest.json")
    design_ranking = read_csv(ranking / "design_ranking.csv")
    path_scores = read_csv(ranking / "path_scores.csv")

    collision_path = campaign / "collision_checks.json"
    collision = read_json(collision_path)["results"]
    require(
        sha256(collision_path) == ranking_manifest["collision_report_sha256"],
        "Collision-report hash mismatch",
    )

    proposals = read_json(campaign / "generation" / "proposals.json")
    proposal_by_id = {
        design["id"]: design
        for design in proposals["designs"]
    }

    screening = read_json(campaign / "screening" / "summary.json")
    screening_by_id = {
        result["design_id"]: result
        for result in screening["results"]
    }

    evaluation = read_json(campaign / "evaluation" / "summary.json")
    evaluation_by_id = {
        job["design_id"]: job
        for job in evaluation["jobs"]
    }

    screen_passes = {
        design_id
        for design_id, result in screening_by_id.items()
        if result["status"] == "pointwise_screen_pass"
    }

    audited_designs = {
        design_id
        for design_id, job in evaluation_by_id.items()
        if job["status"] == "audited_path_found"
    }

    accepted_paths = {
        str(Path(result["path"]).parent): result
        for result in collision
        if result["result"]["sampled_pass"]
    }

    eligible_paths = [
        result
        for result in path_scores
        if result["eligible"] == "True"
    ]

    best_path_by_design = {}

    for result in eligible_paths:
        design_id = result["design_id"]
        candidate = (
            float(result["score"]),
            result["path"],
        )

        if (
            design_id not in best_path_by_design
            or candidate > (
                float(best_path_by_design[design_id]["score"]),
                best_path_by_design[design_id]["path"],
            )
        ):
            best_path_by_design[design_id] = result

    expected_ranking = sorted(
        best_path_by_design.values(),
        key=lambda result: (-float(result["score"]), result["path"]),
    )

    require(
        set(proposal_by_id) == set(screening_by_id),
        "Proposal and screening coverage differ",
    )
    require(
        screen_passes == set(evaluation_by_id),
        "Screen-pass and evaluation coverage differ",
    )
    require(
        set(accepted_paths) == {result["path"] for result in path_scores},
        "Collision-accepted path coverage differs",
    )
    require(
        all(
            result["path_sha256"]
            == accepted_paths[result["path"]]["path_sha256"]
            and result["design_id"]
            == accepted_paths[result["path"]]["design_id"]
            for result in path_scores
        ),
        "Scored-path identity differs from collision record",
    )
    require(
        [
            {
                key: value
                for key, value in result.items()
                if key != "rank"
            }
            for result in design_ranking
        ]
        == expected_ranking,
        "Design ranking differs from eligible path scores",
    )
    require(
        [int(result["rank"]) for result in design_ranking]
        == list(range(1, len(design_ranking) + 1)),
        "Design ranking is not contiguous",
    )

    require(
        [
            len(proposal_by_id),
            len(screen_passes),
            len(audited_designs),
            len(collision),
            len(path_scores),
            len(design_ranking),
            len(eligible_paths),
        ]
        == [
            summary["candidate_designs"],
            summary["screen_passes"],
            summary["evaluation_counts"]["audited_path_found"],
            summary["audited_paths"],
            summary["collision_accepted_paths"],
            summary["ranked_designs"],
            ranking_manifest["eligible_paths"],
        ],
        "Campaign summary counts differ from detailed artifacts",
    )

    task = TaskEnvelope(manifest["task_envelope"])
    task_json = Path(manifest["task_envelope"]) / "task.json"
    require(
        sha256(task_json) == ranking_manifest["task_json_sha256"],
        "Task JSON hash mismatch",
    )

    task_data = read_json(task_json)
    phases = task_data["phases"]

    urdf_path = next(
        Path(path)
        for path in manifest["files"]
        if Path(path).name == "med14_runtime.urdf"
    )
    require(
        sha256(urdf_path) == ranking_manifest["urdf_sha256"],
        "URDF hash mismatch",
    )

    urdf = ET.parse(urdf_path).getroot()
    joint_limits = {
        joint.attrib["name"]: joint.find("limit").attrib
        for joint in urdf.findall("joint")
        if joint.attrib["name"]
        in {f"lbr_A{index}" for index in range(1, 8)}
    }

    lower = np.array([
        float(joint_limits[f"lbr_A{index}"]["lower"])
        for index in range(1, 8)
    ])
    upper = np.array([
        float(joint_limits[f"lbr_A{index}"]["upper"])
        for index in range(1, 8)
    ])
    velocity_limits = np.array([
        float(joint_limits[f"lbr_A{index}"]["velocity"])
        for index in range(1, 8)
    ])

    output.mkdir(parents=True)

    robot = RobotModel()
    checker = TibiaCollisionChecker(robot)
    meshes = [
        read_stl(geometry.meshPath)
        * np.asarray(geometry.meshScale, dtype=float)
        for geometry in checker.geom.geometryObjects
    ]
    cylinder = make_cylinder()

    def robot_geometry(configuration):
        pin.updateGeometryPlacements(
            robot._model,
            robot._data,
            checker.geom,
            checker.geom_data,
            robot._configuration(configuration),
        )
        return [
            move_points(mesh, pose.homogeneous)
            for mesh, pose in zip(
                meshes,
                checker.geom_data.oMg,
            )
        ]

    def render_scene(
        axis,
        configuration,
        requested_tibia,
        actual_tibia,
        actual_ee,
        caption,
        close=False,
        camera=(24, -55),
        mesh_alpha=0.55,
        show_task=True,
        fixed_limits=None,
        attachment=None,
        placement=None,
        requested_ee=None,
        requested_ee_path=None,
        actual_ee_path=None,
        requested_tibia_path=None,
        actual_tibia_path=None,
    ):
        axis.clear()

        for geometry, triangles in zip(
            checker.geom.geometryObjects,
            robot_geometry(configuration),
        ):
            axis.add_collection3d(Poly3DCollection(
                triangles,
                facecolor=(
                    "darkorange"
                    if geometry.name == "lbr_link_7_0"
                    else "silver"
                ),
                edgecolor="none",
                alpha=0.15 if close else mesh_alpha,
            ))

        poses = robot.frame_poses(configuration)
        skeleton = np.array([
            matrix(poses[name])[:3, 3]
            for name in robot.chain_frames
        ])
        axis.plot(
            *skeleton.T,
            "o-",
            color="midnightblue",
            linewidth=2,
            markersize=3,
        )

        axis.add_collection3d(Poly3DCollection(
            move_points(cylinder, actual_tibia),
            facecolor="steelblue",
            edgecolor="none",
            alpha=0.32,
        ))

        for axial_position in (0.0, 0.155, 0.31):
            theta = np.linspace(0.0, 2.0 * np.pi, 49)
            ring = np.stack([
                np.full_like(theta, axial_position),
                0.03 * np.cos(theta),
                0.03 * np.sin(theta),
            ], axis=1)
            axis.plot(
                *move_points(ring, requested_tibia).T,
                color="magenta",
                linewidth=1,
                linestyle="--",
            )

        axis.plot(
            *move_points(
                np.array([[0, 0, 0], [0.31, 0, 0]]),
                requested_tibia,
            ).T,
            color="magenta",
            linestyle="--",
        )

        draw_frame(axis, actual_tibia, "T actual", 0.045)
        draw_frame(
            axis,
            requested_tibia,
            "T requested",
            0.035,
            dashed=True,
        )
        draw_frame(axis, actual_ee, "E actual", 0.04)
        draw_frame(
            axis,
            requested_tibia @ np.linalg.inv(attachment),
            "E required",
            0.035,
            dashed=True,
        )

        if not close:
            draw_frame(axis, np.eye(4), "B", 0.10)
            draw_frame(axis, placement, "F", 0.08)

            if show_task:
                axis.plot(
                    *requested_ee_path[:, :3, 3].T,
                    color="darkorange",
                    linewidth=1,
                    label="Required EE path",
                )
                axis.plot(
                    *actual_ee_path[:, :3, 3].T,
                    color="black",
                    linestyle="--",
                    linewidth=1,
                    label="FK EE path",
                )
                axis.plot(
                    *requested_tibia_path[:, :3, 3].T,
                    color="purple",
                    linewidth=1,
                    label="Requested tibia path",
                )
                axis.plot(
                    *actual_tibia_path[:, :3, 3].T,
                    color="steelblue",
                    linestyle="--",
                    linewidth=1,
                    label="Actual tibia path",
                )

        if fixed_limits is not None:
            equal_limits(axis, fixed_limits)
        else:
            equal_limits(
                axis,
                np.vstack([
                    move_points(cylinder, actual_tibia).reshape(-1, 3),
                    move_points(cylinder, requested_tibia).reshape(-1, 3),
                    actual_ee[:3, 3][None],
                ]),
            )

        axis.view_init(*camera)
        axis.set_title(caption, fontsize=9)

    selected = []

    for ranked in design_ranking[: arguments.top]:
        path_directory = Path(ranked["path"])
        trajectory_path = path_directory / "whole_trajectory_path.npz"

        require(
            trajectory_path.resolve().is_relative_to(campaign),
            "Selected trajectory lies outside campaign directory",
        )
        require(
            sha256(trajectory_path) == ranked["path_sha256"],
            "Selected trajectory hash mismatch",
        )

        with np.load(trajectory_path, allow_pickle=False) as saved:
            time = np.asarray(saved["time_s"], dtype=float)
            joints = np.asarray(saved["q"], dtype=float)
            attachment = matrix(saved["A"])
            placement = matrix(saved["X_BF"])

        require(
            time.ndim == 1
            and joints.shape == (len(time), 7)
            and np.isfinite(time).all()
            and np.isfinite(joints).all()
            and np.all(np.diff(time) > 0),
            "Invalid selected trajectory arrays",
        )

        room = np.rad2deg(
            np.minimum(
                joints - lower,
                upper - joints,
            )
        )
        require(
            np.isclose(
                room.min(),
                float(ranked["minimum_hard_room_deg"]),
                atol=1e-6,
                rtol=0,
            ),
            "Selected trajectory joint room differs from ranking",
        )

        capability_cases = read_csv(
            path_directory / "capability_cases.csv"
        )
        capability = {
            (float(case["time_s"]), int(case["case_index"])):
            float(case["lambda_max"])
            for case in capability_cases
        }

        capability_curves = []

        for phase in phases:
            active_cases = phase["active_case_indices"]

            if not active_cases:
                continue

            start = phase["start_time_s"]
            end = phase["end_time_s"]
            times = sorted({
                sample_time
                for sample_time, case_index in capability
                if start <= sample_time <= end
                and case_index in active_cases
            })
            values = [
                min(
                    capability[sample_time, case_index]
                    for case_index in active_cases
                )
                for sample_time in times
            ]

            require(
                times[0] == start and times[-1] == end,
                "Loaded phase endpoints missing from capability samples",
            )
            capability_curves.append((times, values))

        integral = sum(
            np.dot(
                np.diff(times),
                (np.asarray(values[:-1]) + np.asarray(values[1:])) / 2,
            )
            for times, values in capability_curves
        )
        duration = sum(
            times[-1] - times[0]
            for times, values in capability_curves
        )

        require(
            np.allclose(
                [
                    min(
                        min(values)
                        for _, values in capability_curves
                    ),
                    integral / duration,
                ],
                [
                    float(ranked["min_lambda"]),
                    float(ranked["time_mean_active_lambda"]),
                ],
                rtol=1e-7,
            ),
            "Selected capability metrics differ from ranking",
        )

        selected.append({
            "ranked": ranked,
            "time": time,
            "joints": joints,
            "room": room,
            "capability_curves": capability_curves,
            "attachment": attachment,
            "placement": placement,
            "path_directory": path_directory,
            "trajectory_path": trajectory_path,
        })

    plt.rcParams.update({
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.dpi": 120,
    })

    # 01: Stage yield.
    figure, axes = plt.subplots(
        1,
        2,
        figsize=(12, 4),
        layout="constrained",
    )
    horizontal_bars(
        axes[0],
        [
            "Generated",
            "Screen pass",
            "Audited >=1 path",
            "Accepted >=1 path",
            "Ranked",
        ],
        [
            len(proposal_by_id),
            len(screen_passes),
            len(audited_designs),
            len({result["design_id"] for result in path_scores}),
            len(design_ranking),
        ],
        "Design counts",
    )
    horizontal_bars(
        axes[1],
        ["Audited", "Collision accepted", "Eligible"],
        [len(collision), len(path_scores), len(eligible_paths)],
        "Path counts",
    )
    save_figure(figure, output, "01_stage_yield")

    # 02: Score decomposition.
    top_twenty = list(reversed(design_ranking[:20]))
    figure, axis = plt.subplots(figsize=(10, 8))
    left = np.zeros(len(top_twenty))

    for field, label in (
        ("component_min", "Minimum capability"),
        ("component_mean", "Active-time mean"),
        ("component_joint", "Joint comfort"),
    ):
        values = np.array([
            float(result[field])
            for result in top_twenty
        ])
        axis.barh(
            [result["design_id"] for result in top_twenty],
            values,
            left=left,
            label=label,
        )
        left += values

    axis.set(
        xlabel="Weighted score",
        title="Top 20 designs: score decomposition",
    )
    axis.legend()
    save_figure(figure, output, "02_scores")

    # 03: Capability versus joint room.
    figure, axis = plt.subplots(figsize=(8, 5))

    for eligible, label in (
        ("False", "Buffer not met"),
        ("True", "Eligible"),
    ):
        results = [
            result
            for result in path_scores
            if result["eligible"] == eligible
        ]
        axis.scatter(
            [float(result["extra_room_deg"]) for result in results],
            [float(result["min_lambda"]) for result in results],
            label=label,
            alpha=0.6,
        )

    axis.axvline(
        ranking_manifest["min_extra_deg"],
        linestyle="--",
        color="black",
    )
    axis.set(
        xlabel="Extra joint room beyond reserve [deg]",
        ylabel="Minimum capability multiplier",
        title="All collision-accepted paths",
    )
    axis.legend()
    save_figure(figure, output, "03_tradeoff")

    # 04: Placement coverage.
    placements = {}

    for design in proposals["designs"]:
        key = tuple(
            np.ravel(design["placement"]["X_BF"])
        )
        placements.setdefault(key, []).append(design["id"])

    ranked_ids = set(best_path_by_design)
    stage_colors = [
        ("No screen pass", "#aaaaaa", 0),
        ("Screen pass", "#5585bd", 1),
        ("Audited", "#d28732", 2),
        ("Ranked", "#208158", 3),
    ]

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(15, 5),
        layout="constrained",
    )

    for axis, (first, second) in zip(
        axes,
        ((0, 1), (0, 2), (1, 2)),
    ):
        for rectangle in proposals["placement_domain"]["rectangles"]:
            lower_point = np.array([
                rectangle["x_min_m"],
                rectangle["y_min_m"],
                rectangle["z_min_exclusive_m"],
            ])
            upper_point = np.array([
                rectangle["x_max_m"],
                rectangle["y_max_m"],
                rectangle["z_max_m"],
            ])
            axis.add_patch(Rectangle(
                (lower_point[first], lower_point[second]),
                upper_point[first] - lower_point[first],
                upper_point[second] - lower_point[second],
                fill=False,
                edgecolor="0.75",
            ))

        for label, color, stage in stage_colors:
            points = []

            for key, design_ids in placements.items():
                reached = 0

                for stage_index, stage_ids in enumerate(
                    (screen_passes, audited_designs, ranked_ids),
                    start=1,
                ):
                    if stage_ids.intersection(design_ids):
                        reached = stage_index

                if reached == stage:
                    points.append(
                        np.reshape(key, (4, 4))[:3, 3]
                    )

            if points:
                points = np.asarray(points)
                axis.scatter(
                    points[:, first],
                    points[:, second],
                    color=color,
                    label=label,
                    s=20,
                )

        axis.autoscale_view()
        axis.set_aspect("equal")
        axis.set(
            xlabel=f"Base {'xyz'[first]} [m]",
            ylabel=f"Base {'xyz'[second]} [m]",
        )
        axis.legend(fontsize=7)

    figure.suptitle(
        "Unique femur placements: furthest stage reached by any attachment"
    )
    save_figure(figure, output, "04_placement_coverage")

    # 05: Arm and attachment yield.
    attachment_ids = sorted({
        design["attachment"]["attachment_id"]
        for design in proposals["designs"]
    })
    arms = ["+X", "-X", "+Y", "-Y"]

    def yield_matrix(stage_ids):
        matrix = np.zeros((len(arms), len(attachment_ids)))

        for arm_index, arm in enumerate(arms):
            for mount_index, mount in enumerate(attachment_ids):
                generated = sum(
                    design["placement"]["arm"] == arm
                    and design["attachment"]["attachment_id"] == mount
                    for design in proposals["designs"]
                )
                retained = sum(
                    proposal_by_id[design_id]["placement"]["arm"] == arm
                    and proposal_by_id[design_id]["attachment"]["attachment_id"]
                    == mount
                    for design_id in stage_ids
                )
                matrix[arm_index, mount_index] = (
                    retained / generated if generated else 0
                )

        return matrix

    figure, axes = plt.subplots(
        1,
        2,
        figsize=(12, 5),
        layout="constrained",
    )

    for axis, stage_ids, title in zip(
        axes,
        (screen_passes, ranked_ids),
        ("Screen-passing designs", "Ranked designs"),
    ):
        image = axis.imshow(
            yield_matrix(stage_ids),
            vmin=0,
            vmax=1,
            cmap="Blues",
            aspect="auto",
        )
        axis.set_xticks(
            range(len(attachment_ids)),
            attachment_ids,
            rotation=60,
            ha="right",
        )
        axis.set_yticks(range(len(arms)), arms)
        axis.set_title(title)

    figure.colorbar(image, ax=axes, label="Fraction of generated designs")
    save_figure(figure, output, "05_attachment_yield")

    # 06: Selected path profiles.
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 9),
        sharex=True,
        layout="constrained",
    )

    for index, result in enumerate(selected):
        ranked = result["ranked"]
        label = f"#{ranked['rank']} {ranked['design_id']}"
        color = f"C{index % 10}"

        for times, values in result["capability_curves"]:
            axes[0].plot(
                times,
                values,
                color=color,
                label=label if times == result["capability_curves"][0][0] else None,
            )

        axes[1].plot(
            result["time"],
            result["room"].min(axis=1),
            color=color,
        )
        axes[2].stairs(
            np.max(
                np.abs(np.diff(result["joints"], axis=0))
                / np.diff(result["time"])[:, None]
                / velocity_limits,
                axis=1,
            ),
            result["time"],
            color=color,
            baseline=None,
        )

    for axis in axes:
        for phase in phases:
            if not phase["active_case_indices"]:
                axis.axvspan(
                    phase["start_time_s"],
                    phase["end_time_s"],
                    color="0.9",
                    zorder=-1,
                )

    reserve = manifest["evaluation"]["reserve_deg"]
    axes[1].axhline(reserve, color="0.4", linestyle=":")
    axes[1].axhline(
        reserve + ranking_manifest["min_extra_deg"],
        color="0.4",
        linestyle="--",
    )
    axes[2].axhline(1, color="0.4", linestyle="--")
    axes[0].legend(fontsize=8, ncol=2)
    axes[0].set_ylabel("Worst active multiplier")
    axes[1].set_ylabel("Hard-limit room [deg]")
    axes[2].set(
        ylabel="Max joint speed / limit",
        xlabel="Task time [s]",
    )
    save_figure(figure, output, "06_selected_path_profiles")

    # 07: Collision rejections.
    rejections = []

    for result in collision:
        result_data = result["result"]

        if result_data["sampled_pass"]:
            continue

        first = result_data.get("first_violation") or {}
        worst = result_data.get("worst_sample") or {}
        rejections.append({
            "design": result["design_id"],
            "path": Path(result["path"]).parent.name,
            "minimum_m": result_data["minimum_distance_m"],
            "worst_geometry": worst.get(
                "minimum_distance_geometry",
                "unreported",
            ),
            "first_violating": ";".join(
                first.get("violating_geometries", [])
            )
            or "unreported",
            "first_sample": json.dumps(first),
        })

    if rejections:
        with (output / "collision_rejections.csv").open(
            "w",
            newline="",
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=list(rejections[0]),
            )
            writer.writeheader()
            writer.writerows(rejections)

        figure, axis = plt.subplots(figsize=(10, 8))
        sorted_rejections = sorted(
            rejections,
            key=lambda result: result["minimum_m"],
        )
        axis.barh(
            [
                f"{result['design']}/{result['path']}"
                for result in sorted_rejections
            ],
            [1000 * result["minimum_m"] for result in sorted_rejections],
            color="#aa5548",
        )
        axis.invert_yaxis()
        axis.set(
            xlabel="Minimum reported signed distance [mm]",
            title=(
                "Rejected paths: worst distance, not necessarily "
                "first policy violation"
            ),
        )
        save_figure(figure, output, "07_collision_rejections")

    # 08-10: Per-selected-design review.
    trajectory_reviews = []

    for index, result in enumerate(selected):
        ranked = result["ranked"]
        time = result["time"]
        joints = result["joints"]
        attachment = result["attachment"]
        placement = result["placement"]

        actual_ee = np.array([
            matrix(robot.fk_ee(configuration))
            for configuration in joints
        ])
        requested_tibia = np.array([
            placement @ matrix(task.pose_at(float(sample_time)))
            for sample_time in time
        ])
        actual_tibia = actual_ee @ attachment
        requested_ee = requested_tibia @ np.linalg.inv(attachment)

        ee_position_error = 1000 * np.linalg.norm(
            actual_ee[:, :3, 3] - requested_ee[:, :3, 3],
            axis=1,
        )
        ee_orientation_error = np.rad2deg(
            Rotation.from_matrix(
                np.swapaxes(requested_ee[:, :3, :3], 1, 2)
                @ actual_ee[:, :3, :3]
            ).magnitude()
        )
        tibia_position_error = 1000 * np.linalg.norm(
            actual_tibia[:, :3, 3] - requested_tibia[:, :3, 3],
            axis=1,
        )
        tibia_orientation_error = np.rad2deg(
            Rotation.from_matrix(
                np.swapaxes(requested_tibia[:, :3, :3], 1, 2)
                @ actual_tibia[:, :3, :3]
            ).magnitude()
        )

        fixed_points = np.vstack([
            np.zeros((1, 3)),
            placement[:3, 3][None],
            requested_tibia[:, :3, 3],
            actual_tibia[:, :3, 3],
            actual_ee[:, :3, 3],
            requested_ee[:, :3, 3],
        ] + [
            mesh.reshape(-1, 3)
            for sample_index in (0, len(time) // 2, len(time) - 1)
            for mesh in robot_geometry(joints[sample_index])
        ])

        label = (
            f"rank_{int(ranked['rank']):02d}_"
            f"{ranked['design_id']}"
        )

        figure = plt.figure(figsize=(16, 4.5 * 4))

        for row, (sample_time, close) in enumerate([
            (time[0], False),
            (time[len(time) // 2], False),
            (time[-1], False),
            (time[np.argmax(ee_position_error)], True),
        ]):
            configuration = np.array([
                np.interp(sample_time, time, joints[:, joint])
                for joint in range(7)
            ])
            actual_ee_pose = matrix(robot.fk_ee(configuration))
            actual_tibia_pose = actual_ee_pose @ attachment
            requested_tibia_pose = placement @ matrix(
                task.pose_at(float(sample_time))
            )

            for column, camera in enumerate(
                ((24, -55), (90, -90), (0, -90))
            ):
                axis = figure.add_subplot(
                    4,
                    3,
                    3 * row + column + 1,
                    projection="3d",
                )
                render_scene(
                    axis,
                    configuration,
                    requested_tibia_pose,
                    actual_tibia_pose,
                    actual_ee_pose,
                    (
                        f"{label}\n"
                        f"{'Interface' if close else 'Full robot'} | "
                        f"t={sample_time:.2f}s | "
                        f"EE error "
                        f"{np.interp(sample_time, time, ee_position_error):.3f}"
                        "mm / "
                        f"{np.interp(sample_time, time, ee_orientation_error):.3f}"
                        "deg"
                    ),
                    close=close,
                    camera=camera,
                    show_task=not close,
                    fixed_limits=None if close else fixed_points,
                    attachment=attachment,
                    placement=placement,
                    requested_ee_path=requested_ee,
                    actual_ee_path=actual_ee,
                    requested_tibia_path=requested_tibia,
                    actual_tibia_path=actual_tibia,
                )

        figure.suptitle(
            f"{label}\n"
            "Silver: URDF collision meshes; orange: link 7; "
            "blue: attached tibia; magenta dashed: requested tibia\n"
            "RGB axes = x/y/z. Geometry review, not a safety certificate.",
            fontsize=12,
        )
        figure.tight_layout(rect=(0, 0, 1, 0.94))
        save_figure(
            figure,
            output,
            f"08_{label}_scenes",
        )

        figure, axes = plt.subplots(
            3,
            1,
            figsize=(12, 9),
            sharex=True,
        )

        for joint in range(joints.shape[1]):
            axes[0].plot(
                time,
                np.rad2deg(joints[:, joint]),
                label=f"J{joint + 1}",
            )

        axes[0].set_ylabel("Joint angle [deg]")
        axes[0].legend(ncol=7)
        axes[1].plot(time, ee_position_error, label="EE")
        axes[1].plot(
            time,
            tibia_position_error,
            label="Tibia origin",
        )
        axes[1].set_ylabel("Position error [mm]")
        axes[1].legend()
        axes[2].plot(time, ee_orientation_error, label="EE")
        axes[2].plot(
            time,
            tibia_orientation_error,
            label="Tibia",
        )
        axes[2].set_ylabel("Orientation error [deg]")
        axes[2].set_xlabel("Task time [s]")
        axes[2].legend()

        for axis in axes:
            axis.grid(alpha=0.25)

        figure.suptitle(
            f"{label} - saved-node FK versus independently requested task"
        )
        figure.tight_layout()
        save_figure(figure, output, f"09_{label}_errors")

        if not arguments.no_gif:
            animation_times = np.linspace(
                time[0],
                time[-1],
                arguments.frames,
            )
            animation_joints = np.column_stack([
                np.interp(
                    animation_times,
                    time,
                    joints[:, joint],
                )
                for joint in range(7)
            ])

            figure = plt.figure(figsize=(13, 10))
            views = [
                (False, (24, -55), "Whole robot - fixed base/axes"),
                (False, (15, 45), "Whole robot - fixed base/axes"),
                (True, (90, -90), "Interface - top"),
                (True, (0, -90), "Interface - side"),
            ]
            axes = [
                figure.add_subplot(2, 2, index, projection="3d")
                for index in range(1, 5)
            ]

            def update(frame):
                configuration = animation_joints[frame]
                actual_ee_pose = matrix(robot.fk_ee(configuration))
                actual_tibia_pose = actual_ee_pose @ attachment
                requested_tibia_pose = placement @ matrix(
                    task.pose_at(float(animation_times[frame]))
                )

                for axis, (close, camera, view_label) in zip(
                    axes,
                    views,
                ):
                    render_scene(
                        axis,
                        configuration,
                        requested_tibia_pose,
                        actual_tibia_pose,
                        actual_ee_pose,
                        view_label,
                        close=close,
                        camera=camera,
                        show_task=not close,
                        fixed_limits=None if close else fixed_points,
                        attachment=attachment,
                        placement=placement,
                        requested_ee_path=requested_ee,
                        actual_ee_path=actual_ee,
                        requested_tibia_path=requested_tibia,
                        actual_tibia_path=actual_tibia,
                    )

                figure.suptitle(
                    f"{label} | t={animation_times[frame]:.2f}s | "
                    f"{(time[-1] - time[0]) / ((arguments.frames - 1) / arguments.fps):.1f}"
                    "x playback\n"
                    "Blue: attached tibia; magenta: requested; "
                    "RGB: x/y/z; joint-linear interpolation",
                    fontsize=10,
                )
                return []

            animation = FuncAnimation(
                figure,
                update,
                frames=arguments.frames,
                interval=1000 / arguments.fps,
                blit=False,
            )
            animation.save(
                output / f"10_{label}_motion.gif",
                writer=PillowWriter(fps=arguments.fps),
                dpi=85,
            )
            plt.close(figure)

        capability_times = np.concatenate([
            times
            for times, _ in result["capability_curves"]
        ])
        capability_values = np.concatenate([
            values
            for _, values in result["capability_curves"]
        ])
        minimum_index = int(np.argmin(capability_values))
        maximum_index = int(np.argmax(capability_values))

        trajectory_reviews.append({
            "ranked": ranked,
            "time": time,
            "joints": joints,
            "room": result["room"],
            "attachment": attachment,
            "placement": placement,
            "actual_ee": actual_ee,
            "requested_tibia": requested_tibia,
            "actual_tibia": actual_tibia,
            "requested_ee": requested_ee,
            "ee_position_error": ee_position_error,
            "ee_orientation_error": ee_orientation_error,
            "capability_times": capability_times,
            "capability_values": capability_values,
            "minimum_index": minimum_index,
            "maximum_index": maximum_index,
            "fixed_points": fixed_points,
        })

    # 11: Capability extrema.
    figure = plt.figure(figsize=(18, 12))

    for column, review in enumerate(trajectory_reviews[:2]):
        ranked = review["ranked"]
        time = review["time"]
        joints = review["joints"]
        attachment = review["attachment"]
        placement = review["placement"]

        for row, kind in enumerate(("minimum", "maximum")):
            index = (
                review["minimum_index"]
                if kind == "minimum"
                else review["maximum_index"]
            )
            sample_time = review["capability_times"][index]
            value = review["capability_values"][index]
            configuration = np.array([
                np.interp(sample_time, time, joints[:, joint])
                for joint in range(7)
            ])
            actual_ee = matrix(robot.fk_ee(configuration))
            actual_tibia = actual_ee @ attachment
            requested_tibia = placement @ matrix(
                task.pose_at(float(sample_time))
            )

            axis = figure.add_subplot(
                2,
                2,
                2 * row + column + 1,
                projection="3d",
            )
            render_scene(
                axis,
                configuration,
                requested_tibia,
                actual_tibia,
                actual_ee,
                (
                    f"#{ranked['rank']} {ranked['design_id']}\n"
                    f"{kind.capitalize()} capability: {value:.3f}\n"
                    f"t={sample_time:.2f}s\n"
                    "Modeled zero-bias multiplier, not hardware limit"
                ),
                close=False,
                camera=(24, -55),
                show_task=True,
                fixed_limits=review["fixed_points"],
                attachment=attachment,
                placement=placement,
                requested_ee_path=review["requested_ee"],
                actual_ee_path=review["actual_ee"],
                requested_tibia_path=review["requested_tibia"],
                actual_tibia_path=review["actual_tibia"],
            )

    figure.suptitle(
        "Best versus worst capability posture: rank 1 and rank 5\n"
        "Each design is evaluated at its own capability extrema"
    )
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    save_figure(
        figure,
        output,
        "11_best_worst_capability_postures",
    )

    # 12: Cross-design bottleneck comparison.
    if len(trajectory_reviews) >= 2:
        first = trajectory_reviews[0]
        last = trajectory_reviews[-1]

        figure = plt.figure(figsize=(16, 8))

        for column, (review, heading) in enumerate(
            (
                (first, "Rank 1"),
                (last, f"Rank {arguments.top}"),
            )
        ):
            ranked = review["ranked"]
            index = review["minimum_index"]
            sample_time = review["capability_times"][index]
            value = review["capability_values"][index]
            configuration = np.array([
                np.interp(
                    sample_time,
                    review["time"],
                    review["joints"][:, joint],
                )
                for joint in range(7)
            ])
            actual_ee = matrix(robot.fk_ee(configuration))
            actual_tibia = actual_ee @ review["attachment"]
            requested_tibia = review["placement"] @ matrix(
                task.pose_at(float(sample_time))
            )

            axis = figure.add_subplot(
                1,
                2,
                column + 1,
                projection="3d",
            )
            render_scene(
                axis,
                configuration,
                requested_tibia,
                actual_tibia,
                actual_ee,
                (
                    f"{heading}: {ranked['design_id']}\n"
                    f"Own capability bottleneck: {value:.3f}\n"
                    f"t={sample_time:.2f}s\n"
                    "Different task times; not a same-posture comparison"
                ),
                close=False,
                camera=(24, -55),
                show_task=True,
                fixed_limits=review["fixed_points"],
                attachment=review["attachment"],
                placement=review["placement"],
                requested_ee_path=review["requested_ee"],
                actual_ee_path=review["actual_ee"],
                requested_tibia_path=review["requested_tibia"],
                actual_tibia_path=review["actual_tibia"],
            )

        figure.suptitle(
            "Cross-design comparison at each design's own "
            "capability bottleneck"
        )
        figure.tight_layout(rect=(0, 0, 1, 0.92))
        save_figure(
            figure,
            output,
            "12_cross_design_bottlenecks",
        )

    # 13: Ranking sensitivity.
    policies = [
        ("campaign_0.6_0.1_0.3", (0.6, 0.1, 0.3)),
        ("capability_0.8_0.1_0.1", (0.8, 0.1, 0.1)),
        ("comfort_0.4_0.1_0.5", (0.4, 0.1, 0.5)),
        ("capability_only", (1.0, 0.0, 0.0)),
        ("joint_room_only", (0.0, 0.0, 1.0)),
    ]

    sensitivity = {}

    for policy, weights in policies:
        scores = {}

        for result in eligible_paths:
            if policy == "campaign_0.6_0.1_0.3":
                score = float(result["score"])
            else:
                score = (
                    weights[0] * float(result["min_lambda"])
                    + weights[1] * float(result["time_mean_active_lambda"])
                    + weights[2] * float(result["extra_room_deg"])
                )

            design_id = result["design_id"]

            if (
                design_id not in scores
                or score > scores[design_id][0]
            ):
                scores[design_id] = (score, result["path"])

        sensitivity[policy] = sorted(
            scores.items(),
            key=lambda item: (-item[1][0], item[0]),
        )

    figure, axes = plt.subplots(
        1,
        len(policies),
        figsize=(4.5 * len(policies), 8),
        sharey=True,
        layout="constrained",
    )

    for axis, (policy, ranking_result) in zip(axes, sensitivity.items()):
        top = ranking_result[:15]
        axis.barh(
            [design_id for design_id, _ in top],
            [score for _, (score, _) in top],
            color="#39768c",
        )
        axis.invert_yaxis()
        axis.set_title(policy.replace("_", " "))

    figure.suptitle(
        "Ranking sensitivity: best path reselected within each design\n"
        "Original campaign ranking remains unchanged"
    )
    save_figure(figure, output, "13_ranking_sensitivity")

    # 14: Limiting joints.
    limiting_joints = {}

    for review in trajectory_reviews:
        path_directory = Path(review["ranked"]["path"])
        capability_cases = read_csv(
            path_directory / "capability_cases.csv"
        )
        for case in capability_cases:
            limiting_joints[case["limiting_joint"]] = (
                limiting_joints.get(case["limiting_joint"], 0) + 1
            )

    if limiting_joints:
        figure, axis = plt.subplots(figsize=(10, 5))
        labels, values = zip(*sorted(
            limiting_joints.items(),
            key=lambda item: -item[1],
        ))
        axis.bar(labels, values, color="#aa5548")
        axis.set(
            ylabel="Capability-limiting cases",
            xlabel="Limiting joint",
            title="Limiting joints across selected designs",
        )
        save_figure(figure, output, "14_limiting_joints")

    # 15: Per-joint room profiles.
    figure, axes = plt.subplots(
        7,
        1,
        figsize=(12, 16),
        sharex=True,
        layout="constrained",
    )

    for index, review in enumerate(trajectory_reviews):
        ranked = review["ranked"]
        color = f"C{index % 10}"

        for joint in range(7):
            axes[joint].plot(
                review["time"],
                review["room"][:, joint],
                color=color,
                label=(
                    f"#{ranked['rank']} {ranked['design_id']}"
                    if joint == 0
                    else None
                ),
            )

    for axis in axes:
        axis.grid(alpha=0.25)

    axes[0].legend(fontsize=8, ncol=2)
    axes[-1].set_xlabel("Task time [s]")
    figure.suptitle(
        "Per-joint hard-limit room for selected designs\n"
        "Positive room is distance to the nearer joint limit"
    )
    save_figure(figure, output, "15_per_joint_room")

    # 16: EE error and velocity profiles.
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 9),
        sharex=True,
        layout="constrained",
    )

    for index, review in enumerate(trajectory_reviews):
        ranked = review["ranked"]
        color = f"C{index % 10}"
        label = f"#{ranked['rank']} {ranked['design_id']}"

        axes[0].plot(
            review["time"],
            review["ee_position_error"],
            color=color,
            label=label,
        )
        axes[1].plot(
            review["time"],
            review["ee_orientation_error"],
            color=color,
            label=label,
        )
        axes[2].stairs(
            np.max(
                np.abs(np.diff(review["joints"], axis=0))
                / np.diff(review["time"])[:, None]
                / velocity_limits,
                axis=1,
            ),
            review["time"],
            color=color,
            baseline=None,
            label=label,
        )

    axes[2].axhline(1, color="0.4", linestyle="--")
    axes[0].set_ylabel("EE position error [mm]")
    axes[1].set_ylabel("EE orientation error [deg]")
    axes[2].set(
        ylabel="Max joint speed / limit",
        xlabel="Task time [s]",
    )
    axes[0].legend(fontsize=8, ncol=2)
    save_figure(figure, output, "16_ee_error_and_velocity")

    # 17: Controller manifest.
    controller_manifest = []

    for review in trajectory_reviews:
        ranked = review["ranked"]
        design = proposal_by_id[ranked["design_id"]]
        path_directory = Path(ranked["path"])

        controller_manifest.append({
            "rank": int(ranked["rank"]),
            "design_id": ranked["design_id"],
            "arm": design["placement"]["arm"],
            "attachment_id": design["attachment"]["attachment_id"],
            "selected_path": str(path_directory),
            "selected_path_sha256": ranked["path_sha256"],
            "audit_report": evaluation_by_id[
                ranked["design_id"]
            ]["independent_audit"],
            "collision_status": "accepted",
            "minimum_capability": float(ranked["min_lambda"]),
            "active_time_mean_capability": float(
                ranked["time_mean_active_lambda"]
            ),
            "minimum_hard_limit_room_deg": float(
                ranked["minimum_hard_room_deg"]
            ),
            "extra_room_beyond_reserve_deg": float(
                ranked["extra_room_deg"]
            ),
            "weighted_score": float(ranked["score"]),
            "controller_ready_ee_reference": str(
                path_directory / "whole_trajectory_path.npz"
            ),
        })

    (output / "controller_test_manifest.json").write_text(
        json.dumps(controller_manifest, indent=2) + "\n"
    )

    with (output / "controller_test_manifest.csv").open(
        "w",
        newline="",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(controller_manifest[0]),
        )
        writer.writeheader()
        writer.writerows(controller_manifest)

    # Final summary.
    (output / "summary.json").write_text(json.dumps({
        "campaign": summary,
        "eligible_paths": len(eligible_paths),
        "accepted_designs": len({
            result["design_id"] for result in path_scores
        }),
        "selected": design_ranking[: arguments.top],
        "unreported_first_violations": sum(
            result["first_violating"] == "unreported"
            for result in rejections
        ),
    }, indent=2) + "\n")

    print(output)


if __name__ == "__main__":
    main()