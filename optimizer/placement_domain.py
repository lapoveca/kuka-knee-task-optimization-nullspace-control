"""Configurable placement-domain geometry for robot-task placement sampling.

The placement point is the fixed femur reference origin expressed in the
robot base frame. The cross-shaped domain consists of a central exclusion
plate and optional rectangular arms extending outward from its edges.

All lengths are metres. The frame and placement point are explicit in the
configuration; this module does not assume a particular robot model.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np

PlacementPoint = Literal["femur_reference_origin"]
ArmName = Literal["+X", "-X", "+Y", "-Y"]

VALID_PLACEMENT_POINTS = {"femur_reference_origin"}
VALID_ARMS = {"+X", "-X", "+Y", "-Y"}


@dataclass(frozen=True)
class PlacementRectangle:
    """An axis-aligned rectangular arm in the robot-base XY plane."""

    arm: str
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_exclusive_m: float
    z_max_m: float

    def contains(self, point: np.ndarray) -> bool:
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("placement point must be a finite length-3 vector")

        x, y, z = (float(value) for value in point)

        if not (self.z_min_exclusive_m < z <= self.z_max_m):
            return False

        if self.arm == "+X":
            return (
                self.x_min_m < x <= self.x_max_m
                and self.y_min_m <= y <= self.y_max_m
            )

        if self.arm == "-X":
            return (
                self.x_min_m <= x < self.x_max_m
                and self.y_min_m <= y <= self.y_max_m
            )

        if self.arm == "+Y":
            return (
                self.x_min_m <= x <= self.x_max_m
                and self.y_min_m < y <= self.y_max_m
            )

        if self.arm == "-Y":
            return (
                self.x_min_m <= x <= self.x_max_m
                and self.y_min_m <= y < self.y_max_m
            )

        return False

@dataclass(frozen=True)
class PlacementDomain:
    """A cross-shaped placement domain around a robot base."""

    frame: str
    placement_point: str
    base_plate_side_m: float
    arm_extension_m: float
    enabled_arms: tuple[str, ...]
    z_min_exclusive_m: float
    z_max_m: float
    placements_per_arm: int
    source_path: Path | None = None
    source_sha256: str | None = None

    def __post_init__(self) -> None:
        if self.frame != "lbr_link_0":
            raise ValueError("placement domain frame must be lbr_link_0")

        if self.placement_point not in VALID_PLACEMENT_POINTS:
            raise ValueError(
                "placement_point must be femur_reference_origin"
            )

        if self.base_plate_side_m <= 0.0:
            raise ValueError("base_plate_side_m must be positive")

        if self.arm_extension_m <= 0.0:
            raise ValueError("arm_extension_m must be positive")

        if not self.enabled_arms:
            raise ValueError("at least one arm must be enabled")

        unknown_arms = set(self.enabled_arms) - VALID_ARMS
        if unknown_arms:
            raise ValueError(f"unknown arms: {sorted(unknown_arms)}")

        if len(set(self.enabled_arms)) != len(self.enabled_arms):
            raise ValueError("enabled_arms contains duplicates")

        if self.z_min_exclusive_m < 0.0:
            raise ValueError("z_min_exclusive_m cannot be negative")

        if self.z_max_m <= self.z_min_exclusive_m:
            raise ValueError("z_max_m must exceed z_min_exclusive_m")

        if not isinstance(self.placements_per_arm, int) or self.placements_per_arm <= 0:
            raise ValueError("placements_per_arm must be a positive integer")

    @classmethod
    def from_json(cls, path: str | Path) -> "PlacementDomain":
        source_path = Path(path).expanduser().resolve()
        raw = source_path.read_bytes()
        data = json.loads(raw)

        if data.get("schema") != "placement_domain":
            raise ValueError("placement-domain schema must be placement_domain")

        if data.get("shape") != "cross":
            raise ValueError("placement-domain shape must be cross")

        return cls(
            frame=data["frame"],
            placement_point=data["placement_point"],
            base_plate_side_m=float(data["base_plate_side_m"]),
            arm_extension_m=float(data["arm_extension_m"]),
            enabled_arms=tuple(data["enabled_arms"]),
            z_min_exclusive_m=float(data["z_min_exclusive_m"]),
            z_max_m=float(data["z_max_m"]),
            placements_per_arm=int(data["placements_per_arm"]),
            source_path=source_path,
            source_sha256=hashlib.sha256(raw).hexdigest(),
        )

    @property
    def half_plate_side_m(self) -> float:
        return self.base_plate_side_m / 2.0

    @property
    def inner_edge_m(self) -> float:
        return self.half_plate_side_m

    @property
    def outer_edge_m(self) -> float:
        return self.inner_edge_m + self.arm_extension_m

    @property
    def requested_placement_count(self) -> int:
        return len(self.enabled_arms) * self.placements_per_arm

    def rectangles(self) -> tuple[PlacementRectangle, ...]:
        inner = self.inner_edge_m
        outer = self.outer_edge_m
        half = self.half_plate_side_m

        definitions = {
            "+X": (inner, outer, -half, half),
            "-X": (-outer, -inner, -half, half),
            "+Y": (-half, half, inner, outer),
            "-Y": (-half, half, -outer, -inner),
        }

        return tuple(
            PlacementRectangle(
                arm=arm,
                x_min_m=x_min,
                x_max_m=x_max,
                y_min_m=y_min,
                y_max_m=y_max,
                z_min_exclusive_m=self.z_min_exclusive_m,
                z_max_m=self.z_max_m,
            )
            for arm, (x_min, x_max, y_min, y_max) in definitions.items()
            if arm in self.enabled_arms
        )

    def arm_of(self, point: np.ndarray) -> str | None:
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("placement point must be a finite length-3 vector")

        for rectangle in self.rectangles():
            if rectangle.contains(point):
                return rectangle.arm
        return None

    def contains(self, point: np.ndarray) -> bool:
        return self.arm_of(point) is not None

    def metadata(self) -> dict:
        return {
            "schema": "placement_domain",
            "shape": "cross",
            "frame": self.frame,
            "placement_point": self.placement_point,
            "units": "m",
            "base_plate_side_m": self.base_plate_side_m,
            "arm_extension_m": self.arm_extension_m,
            "enabled_arms": list(self.enabled_arms),
            "z_min_exclusive_m": self.z_min_exclusive_m,
            "z_max_m": self.z_max_m,
            "placements_per_arm": self.placements_per_arm,
            "requested_placement_count": self.requested_placement_count,
            "inner_edge_m": self.inner_edge_m,
            "outer_edge_m": self.outer_edge_m,
            "source_path": (
                str(self.source_path) if self.source_path is not None else None
            ),
            "source_sha256": self.source_sha256,
            "rectangles": [
                {
                    "arm": rectangle.arm,
                    "x_min_m": rectangle.x_min_m,
                    "x_max_m": rectangle.x_max_m,
                    "y_min_m": rectangle.y_min_m,
                    "y_max_m": rectangle.y_max_m,
                    "z_min_exclusive_m": rectangle.z_min_exclusive_m,
                    "z_max_m": rectangle.z_max_m,
                }
                for rectangle in self.rectangles()
            ],
            "interpretation": (
                "Geometric placement-domain restriction for the fixed femur "
                "reference origin in the robot base frame. It is not a reach, "
                "collision, mounting, clinical, or connected-path feasibility "
                "guarantee."
            ),
        }

