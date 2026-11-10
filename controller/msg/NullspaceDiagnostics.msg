builtin_interfaces/Time stamp
float64[7] joint_position
float64[7] joint_velocity

# =====================================================================
# FRAME-INVARIANT SCALARS  (reference point = EE origin, lbr_link_ee)
# A = J J^T. Rotating the twist into another frame (base <-> EE) changes A
# only by an orthogonal similarity, so these do not depend on the frame.
# =====================================================================
float64 w_v                 # sqrt(det(Jv Jv^T)), Yoshikawa on translational rows. [m^3]  (== old `manipulability`)
float64 w_omega             # sqrt(det(Jw Jw^T)), Yoshikawa on rotational rows.    [rad^3-equivalent, dimensionless]
float64 w_6                 # L^3 * sqrt(det(J J^T)), full 6D, rotational rows scaled by L = 0.35 m (tibia). [m^3]
float64 sigma_min_scaled    # smallest singular value of diag(I,L*I)*J. [m]  (distance-to-singularity)
float64 cond_scaled         # sigma_max/sigma_min of diag(I,L*I)*J. Dimensionless. +inf if singular.

# =====================================================================
# BASE FRAME (lbr_link_0), translational rows Jv only   (legacy metrics)
# Velocity value = rotation FREE (projection of the 6D ellipsoid onto xyz).
# =====================================================================
float64[3] velocity_manip_xyz_base   # 1/sqrt(((Jv Jv^T)^-1)_ii): max EE speed along base x,y,z per unit |qdot|
float64[3] force_manip_xyz_base      # 1/sqrt((Jv Jv^T)_ii): max EE force along base x,y,z per unit |tau|
float64[9] velocity_matrix_v_base    # Jv Jv^T (3x3, row-major), expressed in base axes

# =====================================================================
# BASE FRAME (lbr_link_0), full 6D  -- index 0..2 = x,y,z linear ; 3..5 = about x,y,z
# Velocity value = all OTHER 5 DoF held at zero (slice of the 6D ellipsoid).
# =====================================================================
float64[6] velocity_manip_6d_base    # 1/sqrt(((J J^T)^-1)_ii) : [m/s or rad/s] per unit |qdot|
float64[6] force_manip_6d_base       # 1/sqrt((J J^T)_ii)      : [N or N*m] per unit |tau| (others zero)

# =====================================================================
# END-EFFECTOR FRAME (lbr_link_ee axes), full 6D -- same index convention
# J_ee = blkdiag(R^T, R^T) J with R = R_base_ee (from FK). Tibia = +z of this frame.
# =====================================================================
float64[6] velocity_manip_6d_ee      # 1/sqrt(((J_ee J_ee^T)^-1)_ii)
float64[6] force_manip_6d_ee         # 1/sqrt((J_ee J_ee^T)_ii)
float64[36] velocity_matrix_6d_ee    # J_ee J_ee^T (6x6, row-major)



# Joint-velocity-limit-weighted 3D block directional radii in EE axes.
# Index order is x, y, z. Linear velocity [m/s].
float64[3] velocity_manip_linear_ee

# Joint-velocity-limit-weighted angular block directional radii in EE axes.
# Index order is about x, y, z. Angular velocity [rad/s].
float64[3] velocity_manip_angular_ee

# Joint-effort-limit-weighted translational block directional radii in EE axes.
# Index order is force along x, y, z. [N].
float64[3] force_manip_linear_ee

# Joint-effort-limit-weighted angular block directional radii in EE axes.
# Index order is moment about x, y, z. [N*m].
float64[3] moment_manip_angular_ee