#!/usr/bin/env python3
"""Sampled collision checks for Med14 URDF geometry versus an attached tibia."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import coal
import numpy as np
import pinocchio as pin

from robot_model import RobotModel
from transforms import validate_transform


HERE = Path(__file__).resolve().parent
DEFAULT_URDF = HERE / "models" / "med14_runtime.urdf"
DEFAULT_PACKAGE_DIR = (
    Path.home() / "ros2_ws/install/lbr_med14_r820_description/share"
)

TIBIA_RADIUS_M = 0.03
TIBIA_LENGTH_M = 0.31

# X_TS maps cylinder-local coordinates into tibia coordinates.
# Cylinder +z maps to tibia +x; its ends lie at x_T = 0 and 0.31 m.
X_TS = np.array([
    [0.0, 0.0, 1.0, TIBIA_LENGTH_M / 2.0],
    [0.0, 1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
])


def _transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    validate_transform(matrix)
    return matrix


def _nonnegative(value, name):
    value = float(value)
    if not np.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return value


class TibiaCollisionChecker:
    """Reuse one robot geometry model across checks in a single worker."""

    def __init__(self, robot, *, urdf=DEFAULT_URDF,
                 package_dirs=(DEFAULT_PACKAGE_DIR,)):
        self.robot = robot
        self.urdf = Path(urdf).resolve(strict=True)
        self.geom = pin.buildGeomFromUrdf(
            robot._model,
            str(self.urdf),
            pin.GeometryType.COLLISION,
            package_dirs=[str(Path(p).expanduser().resolve()) for p in package_dirs],
        )
        self.interface_geometry_names = {
            "lbr_link_7_0",
        }
        self.interface_allowance_m = 0.0001
        if self.geom.ngeoms == 0:
            raise RuntimeError("No robot collision geometries loaded")

        self.geom_data = pin.GeometryData(self.geom)
        self.cylinder = coal.Cylinder(TIBIA_RADIUS_M, TIBIA_LENGTH_M)
        if (
            not np.isclose(self.cylinder.radius, TIBIA_RADIUS_M)
            or not np.isclose(self.cylinder.halfLength, TIBIA_LENGTH_M / 2.0)
        ):
            raise RuntimeError("Unexpected Coal cylinder dimensions")

        self.collision_request = coal.CollisionRequest()
        self.distance_request = coal.DistanceRequest()
        self._check_primitives()

    def _check_primitives(self):
        sphere = coal.Sphere(0.005)
        rotation = np.eye(3)
        cylinder_tf = coal.Transform3s(rotation, np.zeros(3))
        for x, expected in ((0.0, True), (1.0, False)):
            sphere_tf = coal.Transform3s(rotation, np.array([x, 0.0, 0.0]))
            result = coal.CollisionResult()
            coal.collide(
                sphere, sphere_tf, self.cylinder, cylinder_tf,
                self.collision_request, result,
            )
            if bool(result.isCollision()) != expected:
                raise RuntimeError(f"Coal primitive sanity check failed at x={x}")

    def check_configuration(self, q, A, *, clearance_m=0.0):
        """Check one optimizer joint vector; A maps tibia coordinates into E."""
        q = np.asarray(q, dtype=float)
        if q.ndim != 1 or q.size == 0 or not np.all(np.isfinite(q)):
            raise ValueError("q must be a nonempty finite joint vector")
        A = _transform(A, "A")
        clearance_m = _nonnegative(clearance_m, "clearance_m")
        return self._check_configuration(q, A, clearance_m)

    def _check_configuration(self, q, A, clearance_m):
        X_BE = _transform(self.robot.fk_ee(q), "X_BE")
        X_BS = X_BE @ A @ X_TS
        cylinder_tf = coal.Transform3s(X_BS[:3, :3], X_BS[:3, 3])

        pin.updateGeometryPlacements(
            self.robot._model,
            self.robot._data,
            self.geom,
            self.geom_data,
            self.robot._configuration(q),
        )

        minimum = float("inf")
        minimum_geometry = None
        hits = []
        violations = []

        for i, obj in enumerate(self.geom.geometryObjects):
            pose = self.geom_data.oMg[i]
            robot_tf = coal.Transform3s(pose.rotation, pose.translation)

            collision_result = coal.CollisionResult()
            coal.collide(
                obj.geometry, robot_tf, self.cylinder, cylinder_tf,
                self.collision_request, collision_result,
            )
            hit = bool(collision_result.isCollision())

            distance_result = coal.DistanceResult()
            distance = float(coal.distance(
                obj.geometry, robot_tf, self.cylinder, cylinder_tf,
                self.distance_request, distance_result,
            ))
            if not np.isfinite(distance):
                raise RuntimeError(f"Nonfinite Coal distance for {obj.name}")

            if distance < minimum:
                minimum = distance
                minimum_geometry = obj.name

            if hit:
                hits.append(obj.name)

            if obj.name in self.interface_geometry_names:
                effective_clearance = (
                    clearance_m - self.interface_allowance_m
                )
                is_violation = distance < effective_clearance
            else:
                is_violation = hit or distance < clearance_m

            if is_violation:
                violations.append(obj.name)
                

        return {
            "minimum_distance_m": minimum,
            "minimum_distance_geometry": minimum_geometry,
            "colliding_geometries": hits,
            "violating_geometries": violations,
        }

    def check_path(self, Q, A, *, clearance_m=0.0, max_joint_step_rad=None):
        """Check saved q rows and optionally linear joint-space subdivisions."""
        Q = np.asarray(Q, dtype=float)
        if (
            Q.ndim != 2 or Q.shape[0] == 0 or Q.shape[1] == 0
            or not np.all(np.isfinite(Q))
        ):
            raise ValueError("Q must be a nonempty finite (nodes, joints) array")
        A = _transform(A, "A")
        clearance_m = _nonnegative(clearance_m, "clearance_m")
        if max_joint_step_rad is not None:
            max_joint_step_rad = _nonnegative(
                max_joint_step_rad, "max_joint_step_rad"
            )
            if max_joint_step_rad == 0.0:
                raise ValueError("max_joint_step_rad must be positive")

        def samples():
            yield Q[0], {"node_index": 0}
            for i in range(len(Q) - 1):
                if max_joint_step_rad is not None:
                    delta = Q[i + 1] - Q[i]
                    n = max(1, int(np.ceil(
                        np.max(np.abs(delta)) / max_joint_step_rad
                    )))
                    for j in range(1, n):
                        alpha = j / n
                        yield Q[i] + alpha * delta, {
                            "edge_start_index": i,
                            "edge_end_index": i + 1,
                            "alpha": alpha,
                        }
                yield Q[i + 1], {"node_index": i + 1}

        worst = None
        first_violation = None
        violation_count = 0
        collision_count = 0
        sample_count = 0

        for q, location in samples():
            result = {**location, **self._check_configuration(q, A, clearance_m)}
            sample_count += 1
            if worst is None or result["minimum_distance_m"] < worst["minimum_distance_m"]:
                worst = result
            if result["colliding_geometries"]:
                collision_count += 1
            if result["violating_geometries"]:
                violation_count += 1
                if first_violation is None:
                    first_violation = result

        return {
            "sampled_pass": violation_count == 0,
            "saved_nodes_checked": int(Q.shape[0]),
            "samples_checked": sample_count,
            "interpolated_samples_checked": sample_count - int(Q.shape[0]),
            "collision_samples": collision_count,
            "violation_samples": violation_count,
            "clearance_m": clearance_m,
            "max_joint_step_rad": max_joint_step_rad,
            "minimum_distance_m": worst["minimum_distance_m"],
            "worst_sample": worst,
            "first_violation": first_violation,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="whole_trajectory_path.npz")
    parser.add_argument("--clearance-m", type=float, default=0.0)
    parser.add_argument("--max-joint-step-rad", type=float, default=None)
    args = parser.parse_args()

    with np.load(args.path, allow_pickle=False) as saved:
        Q = saved["q"]
        A = saved["A"]

    checker = TibiaCollisionChecker(RobotModel())
    report = checker.check_path(
        Q, A,
        clearance_m=args.clearance_m,
        max_joint_step_rad=args.max_joint_step_rad,
    )
    report.update({
        "path": str(args.path.resolve()),
        "robot_urdf_sha256": checker.robot.urdf_sha256,
        "collision_geometry_count": int(checker.geom.ngeoms),
        "scope": "Sampled robot URDF geometry versus attached tibia cylinder only",
    })
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["sampled_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())