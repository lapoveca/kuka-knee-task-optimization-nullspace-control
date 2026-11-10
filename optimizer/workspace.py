"""An explicit, fixed OpenSim-ground-to-base placement. No ROS imports."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from transforms import validate_transform


def checked_transform(value):
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError("rigid transform must be a finite (4,4) array")
    validate_transform(result)
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-10, rtol=0):
        raise ValueError("invalid homogeneous last row")
    if not np.allclose(result[:3,:3].T @ result[:3,:3], np.eye(3), atol=1e-8, rtol=0):
        raise ValueError("rotation must be orthogonal")
    if not np.isclose(np.linalg.det(result[:3,:3]), 1., atol=1e-8, rtol=0):
        raise ValueError("rotation must have determinant +1")
    return result


@dataclass(frozen=True)
class FixedWorkspace:
    X_BG: np.ndarray
    provenance: str
    description: str

    def __post_init__(self):
        if self.provenance not in {"synthetic", "simulation", "measured"}:
            raise ValueError("provenance must be synthetic, simulation, or measured")
        if not self.description.strip():
            raise ValueError("workspace description is required")
        matrix = checked_transform(self.X_BG)
        matrix.setflags(write=False)
        object.__setattr__(self, "X_BG", matrix)

    @classmethod
    def from_json(cls, path):
        data = json.loads(Path(path).read_text())
        return cls(data["X_BG"], data["provenance"], data["description"])