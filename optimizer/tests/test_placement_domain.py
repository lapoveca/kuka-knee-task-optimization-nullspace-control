import json
from pathlib import Path

import numpy as np
import pytest

from placement_domain import PlacementDomain


@pytest.fixture
def domain(tmp_path: Path) -> PlacementDomain:
    config = {
        "schema": "placement_domain",
        "shape": "cross",
        "frame": "lbr_link_0",
        "placement_point": "femur_reference_origin",
        "base_plate_side_m": 0.30,
        "arm_extension_m": 0.60,
        "enabled_arms": ["+X", "-X", "+Y", "-Y"],
        "placements_per_arm": 10,
        "placements_per_arm": 10,
        "z_min_exclusive_m": 0.0,
        "z_max_m": 1.40,
    }
    path = tmp_path / "placement_domain.json"
    path.write_text(json.dumps(config))
    return PlacementDomain.from_json(path)


def test_positive_x_arm(domain: PlacementDomain) -> None:
    point = np.array([0.45, 0.0, 0.50])
    assert domain.contains(point)
    assert domain.arm_of(point) == "+X"


def test_negative_x_arm(domain: PlacementDomain) -> None:
    point = np.array([-0.45, 0.0, 0.50])
    assert domain.contains(point)
    assert domain.arm_of(point) == "-X"


def test_positive_y_arm(domain: PlacementDomain) -> None:
    point = np.array([0.0, 0.45, 0.50])
    assert domain.contains(point)
    assert domain.arm_of(point) == "+Y"


def test_negative_y_arm(domain: PlacementDomain) -> None:
    point = np.array([0.0, -0.45, 0.50])
    assert domain.contains(point)
    assert domain.arm_of(point) == "-Y"


def test_central_plate_is_excluded_at_all_heights(
    domain: PlacementDomain,
) -> None:
    for z in (0.01, 0.50, 1.00, 1.40):
        assert not domain.contains(np.array([0.0, 0.0, z]))
        assert not domain.contains(np.array([0.10, 0.10, z]))


def test_corners_are_excluded(domain: PlacementDomain) -> None:
    for point in (
        [0.45, 0.45, 0.50],
        [-0.45, 0.45, 0.50],
        [0.45, -0.45, 0.50],
        [-0.45, -0.45, 0.50],
    ):
        assert not domain.contains(np.array(point))


def test_zero_height_is_excluded(domain: PlacementDomain) -> None:
    assert not domain.contains(np.array([0.45, 0.0, 0.0]))


def test_height_ceiling_is_inclusive(domain: PlacementDomain) -> None:
    assert domain.contains(np.array([0.45, 0.0, 1.40]))
    assert not domain.contains(np.array([0.45, 0.0, 1.400001]))


def test_outer_edge_is_inclusive(domain: PlacementDomain) -> None:
    assert domain.contains(np.array([0.75, 0.0, 0.50]))
    assert not domain.contains(np.array([0.750001, 0.0, 0.50]))


def test_inner_edge_is_exclusive(domain: PlacementDomain) -> None:
    assert not domain.contains(np.array([0.15, 0.0, 0.50]))
    assert domain.contains(np.array([0.150001, 0.0, 0.50]))


def test_disabled_arm_is_excluded(tmp_path: Path) -> None:
    config = {
        "schema": "placement_domain",
        "shape": "cross",
        "frame": "lbr_link_0",
        "placement_point": "femur_reference_origin",
        "base_plate_side_m": 0.30,
        "arm_extension_m": 0.60,
        "enabled_arms": ["+X"],
        "placements_per_arm": 10,
        "placements_per_arm": 10,
        "z_min_exclusive_m": 0.0,
        "z_max_m": 1.40,
    }
    path = tmp_path / "placement_domain.json"
    path.write_text(json.dumps(config))
    domain = PlacementDomain.from_json(path)

    assert domain.contains(np.array([0.45, 0.0, 0.50]))
    assert not domain.contains(np.array([-0.45, 0.0, 0.50]))
    assert not domain.contains(np.array([0.0, 0.45, 0.50]))
    assert not domain.contains(np.array([0.0, -0.45, 0.50]))


def test_plate_size_changes_arm_width(tmp_path: Path) -> None:
    config = {
        "schema": "placement_domain",
        "shape": "cross",
        "frame": "lbr_link_0",
        "placement_point": "femur_reference_origin",
        "base_plate_side_m": 0.40,
        "arm_extension_m": 0.60,
        "enabled_arms": ["+X", "-X", "+Y", "-Y"],
        "placements_per_arm": 10,
        "placements_per_arm": 10,
        "z_min_exclusive_m": 0.0,
        "z_max_m": 1.40,
    }
    path = tmp_path / "placement_domain.json"
    path.write_text(json.dumps(config))
    domain = PlacementDomain.from_json(path)

    assert domain.inner_edge_m == pytest.approx(0.20)
    assert domain.outer_edge_m == pytest.approx(0.80)

    rectangles = {rectangle.arm: rectangle for rectangle in domain.rectangles()}
    assert rectangles["+X"].y_min_m == pytest.approx(-0.20)
    assert rectangles["+X"].y_max_m == pytest.approx(0.20)


def test_arm_extension_changes_outer_edge(tmp_path: Path) -> None:
    config = {
        "schema": "placement_domain",
        "shape": "cross",
        "frame": "lbr_link_0",
        "placement_point": "femur_reference_origin",
        "base_plate_side_m": 0.30,
        "arm_extension_m": 0.80,
        "enabled_arms": ["+X", "-X", "+Y", "-Y"],
        "placements_per_arm": 10,
        "placements_per_arm": 10,
        "z_min_exclusive_m": 0.0,
        "z_max_m": 1.40,
    }
    path = tmp_path / "placement_domain.json"
    path.write_text(json.dumps(config))
    domain = PlacementDomain.from_json(path)

    assert domain.outer_edge_m == pytest.approx(0.95)
    assert domain.contains(np.array([0.90, 0.0, 0.50]))
    assert not domain.contains(np.array([0.950001, 0.0, 0.50]))


def test_metadata_records_resolved_geometry(
    domain: PlacementDomain,
) -> None:
    metadata = domain.metadata()

    assert metadata["placement_point"] == "femur_reference_origin"
    assert metadata["inner_edge_m"] == pytest.approx(0.15)
    assert metadata["outer_edge_m"] == pytest.approx(0.75)
    assert metadata["z_max_m"] == pytest.approx(1.40)
    assert len(metadata["rectangles"]) == 4



def test_requested_placement_count(domain: PlacementDomain) -> None:
    assert domain.requested_placement_count == 40
