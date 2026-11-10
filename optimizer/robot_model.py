"""Kinematics-only Pinocchio adapter for the audited fixed-base Med14.

Public ordering: lbr_A1..lbr_A7. Poses map frame coordinates into B.
Jacobian ordering: [linear velocity at E origin; angular velocity], B axes.
No ROS, mesh loading, IK, dynamics, collision or robot commands.
An instance owns mutable Pinocchio Data: use serially, not concurrently.
"""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin

AUDITED_SHA256 = "ae0c657636ec294f098c240557f3337dd6314002571c2c9833f510e26d825112"
DEFAULT_URDF = Path(__file__).resolve().parent / "models" / "med14_runtime.urdf"
JOINT_NAMES = tuple(f"lbr_A{i}" for i in range(1, 8))
BASE_FRAME = "lbr_link_0"
EE_FRAME = "lbr_link_ee"
CHAIN_FRAMES = (BASE_FRAME,) + tuple(f"lbr_link_{i}" for i in range(1, 8)) + (EE_FRAME,)


def _read_only(values):
    a = np.array(values, dtype=float, copy=True)
    a.setflags(write=False)
    return a


class RobotModel:
    def __init__(self, urdf_path=DEFAULT_URDF, *, expected_sha256=AUDITED_SHA256):
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        raw = self.urdf_path.read_bytes()
        self.urdf_sha256 = hashlib.sha256(raw).hexdigest()
        if expected_sha256 is not None and self.urdf_sha256 != expected_sha256:
            raise ValueError(
                f"URDF differs from audited snapshot: {self.urdf_sha256}. "
                "Review/version the model rather than bypassing this check for a study run."
            )
        root = ET.fromstring(raw)
        if root.tag != "robot":
            raise ValueError("expected a URDF robot document")
        joints = {j.attrib["name"]:j for j in root.findall("joint")}
        lower, upper, velocity, effort = [], [], [], []
        for index, name in enumerate(JOINT_NAMES, 1):
            if name not in joints:
                raise ValueError(f"missing audited joint {name}")
            j = joints[name]
            if j.attrib["type"] != "revolute" or j.find("mimic") is not None:
                raise ValueError("this adapter requires seven independent bounded revolute joints")
            if j.find("parent").attrib["link"] != f"lbr_link_{index-1}" or j.find("child").attrib["link"] != f"lbr_link_{index}":
                raise ValueError(f"unexpected topology at {name}")
            limit = j.find("limit")
            if limit is None:
                raise ValueError(f"missing limits for {name}")
            lower.append(float(limit.attrib["lower"]))
            upper.append(float(limit.attrib["upper"]))
            velocity.append(float(limit.attrib["velocity"]))
            effort.append(float(limit.attrib["effort"]))
        if not np.all(np.isfinite([lower,upper,velocity,effort])) or not np.all(np.array(lower)<np.array(upper)):
            raise ValueError("invalid joint limits")
        self.joint_names = JOINT_NAMES
        self.chain_frames = CHAIN_FRAMES
        self.position_lower = _read_only(lower)
        self.position_upper = _read_only(upper)
        self.velocity_limits = _read_only(velocity)
        self.urdf_effort_limits = _read_only(effort)
        self._model = pin.buildModelFromUrdf(str(self.urdf_path))
        self._data = self._model.createData()
        if self._model.nq != 7 or self._model.nv != 7:
            raise ValueError("full model must have nq=nv=7 for this adapter")
        self._frame_ids = {name:self._model.getFrameId(name) for name in CHAIN_FRAMES}
        if any(i >= self._model.nframes for i in self._frame_ids.values()):
            raise ValueError("a required link frame is missing")
        if int(self._model.frames[self._frame_ids[BASE_FRAME]].parentJoint) != 0:
            raise ValueError("base must be fixed to the model root")
        q_indices, v_indices = [], []
        for name in JOINT_NAMES:
            if name not in list(self._model.names):
                raise ValueError(f"Pinocchio joint not found: {name}")
            joint = self._model.joints[self._model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f"unsupported configuration representation at {name}")
            q_indices.append(int(joint.idx_q))
            v_indices.append(int(joint.idx_v))
        self._q_indices = np.array(q_indices, dtype=int)
        self._v_indices = np.array(v_indices, dtype=int)

    def _configuration(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("q must be a finite length-7 vector ordered lbr_A1..lbr_A7")
        if np.any(q < self.position_lower-1e-12) or np.any(q > self.position_upper+1e-12):
            raise ValueError("q is outside the URDF position limits")
        qp = pin.neutral(self._model)
        qp[self._q_indices] = q
        return qp

    def frame_poses(self, q):
        qp = self._configuration(q)
        pin.framesForwardKinematics(self._model, self._data, qp)
        xob_inverse = self._data.oMf[self._frame_ids[BASE_FRAME]].inverse()
        return {name:(xob_inverse*self._data.oMf[index]).homogeneous.copy()
                for name,index in self._frame_ids.items()}

    def frame_pose(self, q, name):
        if name not in self._frame_ids:
            raise ValueError(f"supported frame names: {self.chain_frames}")
        return self.frame_poses(q)[name]

    def fk_ee(self, q):
        return self.frame_pose(q, EE_FRAME)

    def jacobian_ee(self, q):
        qp = self._configuration(q)
        pin.computeJointJacobians(self._model, self._data, qp)
        pin.updateFramePlacements(self._model, self._data)
        jw = np.array(pin.getFrameJacobian(
            self._model, self._data, self._frame_ids[EE_FRAME],
            pin.LOCAL_WORLD_ALIGNED,
        ), copy=True)[:,self._v_indices]
        rbo = self._data.oMf[self._frame_ids[BASE_FRAME]].rotation.T
        # The reference point remains E; only the vector axes change O -> B.
        return np.vstack((rbo@jw[:3], rbo@jw[3:])).copy()

    def validation_configurations(self, count=10, seed=42):
        if count < 1:
            raise ValueError("count must be positive")
        rng = np.random.default_rng(seed)
        lo,hi = self.position_lower,self.position_upper
        return np.vstack((np.zeros(7),lo+(hi-lo)*rng.uniform(0.2,0.8,(count,7))))
