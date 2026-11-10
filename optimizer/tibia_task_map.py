"""Fixed-L tibia-point kinematics and dual wrench mapping.

A = {}^E T_T. X_BT = X_BE @ A. L is fixed at 0.35 m in T.
Twists: [point linear velocity; angular velocity]. Wrenches: [force; moment].
All moments are about the declared origin. No dynamics or robot commands.

W_T denotes the desired ROBOT-ON-TIBIA wrench for actuator-demand analysis.
Then the task contribution is tau_task = J_T.T @ W_T. A measured
ENVIRONMENT-ON-ROBOT wrench has the opposite action/reaction sign and must
not be substituted without conversion. A wrench mapping itself only changes
its representation; it never flips physical action/reaction automatically.
"""
import numpy as np
from workspace import checked_transform
from attachment import attachment_to_xte

FIXED_MOUNT_LENGTH_M=0.35


def _six(values,name):
    values=np.asarray(values,dtype=float)
    if values.shape!=(6,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be a finite length-6 vector")
    return values


def _cross_matrix(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


class TibiaTaskMap:
    """One immutable mounting, evaluated at arbitrary robot configurations.

    This enforces the project's fixed mounting point, but is not a collision
    or orientation-admissibility checker. Candidate generation owns that role.
    T is the declared OpenSim tibia frame, not automatically a measured knee center.
    """
    def __init__(self,robot,attachment):
        self.robot=robot
        a=checked_transform(attachment)
        xte=attachment_to_xte(a)
        if not np.allclose(xte[:3,3],[FIXED_MOUNT_LENGTH_M,0.,0.],atol=1e-10,rtol=0):
            raise ValueError("study mounting point must be tibia [0.35,0,0] m")
        self._attachment=a.copy()
        self._attachment.setflags(write=False)

    @property
    def attachment(self):
        result=self._attachment.copy()
        result.setflags(write=False)
        return result

    def _geometry(self,q):
        xbe=self.robot.fk_ee(q)
        xbt=xbe@self._attachment
        r_ee_to_tibia=xbe[:3,:3]@self._attachment[:3,3]
        return xbt,r_ee_to_tibia

    def pose(self,q):
        """Return X_BT, tibia pose in base coordinates."""
        return self._geometry(q)[0].copy()

    def jacobian_base(self,q):
        """T-origin linear velocity and angular velocity, expressed in B."""
        _,r=self._geometry(q)
        je=self.robot.jacobian_ee(q)
        return np.vstack((je[:3]-_cross_matrix(r)@je[3:],je[3:]))

    def jacobian_tibia(self,q):
        """T-origin linear velocity and angular velocity, expressed in T."""
        xbt,_=self._geometry(q)
        jbase=self.jacobian_base(q)
        rtb=xbt[:3,:3].T
        return np.vstack((rtb@jbase[:3],rtb@jbase[3:]))

    def wrench_at_ee_base(self,q,wrench_tibia):
        """Same wrench moved T -> E and expressed in B, [force; moment]."""
        wrench_tibia=_six(wrench_tibia,"wrench_tibia")
        xbt,r=self._geometry(q)
        f=xbt[:3,:3]@wrench_tibia[:3]
        m=xbt[:3,:3]@wrench_tibia[3:]+np.cross(r,f)
        return np.r_[f,m]

    def torque_for_wrench(self,q,wrench_tibia):
        """Task-wrench joint torque only: excludes gravity/inertia/friction."""
        return self.jacobian_tibia(q).T@_six(wrench_tibia,"wrench_tibia")
