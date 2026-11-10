#!/usr/bin/env python3
"""
Knee model interface for mechanical_knee_testbed_cbl.osim
Converts knee coordinate values (angles) -> Cartesian pose of tibia in ground frame.
"""
import numpy as np
from geometry_msgs.msg import PoseStamped
import opensim

def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert 3x3 rotation matrix to quaternion [w, x, y, z].
    Robust implementation using trace method.
    """
    trace = np.trace(R)
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    
    return np.array([w, x, y, z])


class KneeModelInterface:
    def __init__(self, model_path: str):
        """Load and initialize the OpenSim knee model."""
        self.model = opensim.Model(model_path)
        self.state = self.model.initSystem()
        
        # Cache coordinate references
        self.flexion_coord = self.model.getCoordinateSet().get("flexion_extension_rz")
        self.varus_coord = self.model.getCoordinateSet().get("varus_valgus_rz")
        self.rotation_coord = self.model.getCoordinateSet().get("internal_external_rotation_rz")
        
        # Cache tibia body reference (the output frame we care about)
        self.tibia_body = self.model.getBodySet().get("tibia")
        
    def set_knee_pose(self, flexion_deg: float = 0.0, 
                      varus_deg: float = 0.0, 
                      rotation_deg: float = 0.0):
        """
        Set knee coordinates and recompute forward kinematics.
        
        Args:
            flexion_deg: Knee flexion angle (positive = flexion, 0 to ~80 deg)
            varus_deg: Varus/valgus angle (positive = varus, +-25 deg)
            rotation_deg: Internal/external rotation (positive = internal, +-25 deg)
        """
        self.flexion_coord.setValue(self.state, np.deg2rad(flexion_deg))
        self.varus_coord.setValue(self.state, np.deg2rad(varus_deg))
        self.rotation_coord.setValue(self.state, np.deg2rad(rotation_deg))
        self.model.realizePosition(self.state)
        
    def get_tibia_pose_in_ground(self):
        """
        Get tibia position and rotation in ground frame.
        
        Returns:
            pos: numpy array [x, y, z] in meters
            rot: numpy 3x3 rotation matrix
        """
        X_GT = self.tibia_body.getTransformInGround(self.state)
        
        # Position
        p = X_GT.p()
        pos = np.array([p.get(0), p.get(1), p.get(2)])
        
        # Rotation as 3x3 matrix
        R = X_GT.R()
        Rmat = R.toMat33()
        rot = np.array([
            [Rmat.get(0,0), Rmat.get(0,1), Rmat.get(0,2)],
            [Rmat.get(1,0), Rmat.get(1,1), Rmat.get(1,2)],
            [Rmat.get(2,0), Rmat.get(2,1), Rmat.get(2,2)],
        ])
        
        return pos, rot
    
    def get_tibia_pose_as_geometry_msgs(self, frame_id: str = "lbr_link_0",
                                        base_offset: np.ndarray = None) -> PoseStamped:
        """
        Get tibia pose as PoseStamped message for ROS2.
        
        Args:
            frame_id: ROS2 frame ID (typically "lbr_link_0" for robot base)
            base_offset: Optional 4x4 transform from OpenSim "ground" to robot base frame
                        (calibration transform: where is the femur relative to the robot?)
                        If None, assumes OpenSim ground = robot base frame.
        
        Returns:
            PoseStamped message ready to publish to CIC controller
        """
        # Get tibia pose in OpenSim ground frame
        pos, rot = self.get_tibia_pose_in_ground()
        
        # Apply calibration offset if provided
        if base_offset is not None:
            # pos is 3x1, rot is 3x3
            # base_offset is 4x4 homogeneous transform
            X_GT = np.eye(4)
            X_GT[:3, :3] = rot
            X_GT[:3, 3] = pos
            X_GT_calibrated = base_offset @ X_GT
            pos = X_GT_calibrated[:3, 3]
            rot = X_GT_calibrated[:3, :3]
        
        # Convert rotation matrix to quaternion (w, x, y, z)
        quaternion = rotation_matrix_to_quaternion(rot)  # returns [w, x, y, z]
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        # msg.header.stamp will be set by caller
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = float(quaternion[0])
        msg.pose.orientation.x = float(quaternion[1])
        msg.pose.orientation.y = float(quaternion[2])
        msg.pose.orientation.z = float(quaternion[3])
        
        return msg