#!/usr/bin/env python3
"""Read-only robot/tibia trajectory review. Run from optimizer/."""
from __future__ import annotations
import argparse
import hashlib
import json
import struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
import pinocchio as pin
from robot_model import RobotModel
from task_envelope import TaskEnvelope
from tibia_collision import TibiaCollisionChecker
from transforms import validate_transform


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def matrix(v):
    v = np.asarray(v, dtype=float)
    if v.shape != (4, 4) or not np.isfinite(v).all():
        raise ValueError("Expected finite 4x4 transform")
    validate_transform(v)
    return v


def stl_triangles(path):
    raw = Path(path).read_bytes()
    if len(raw) >= 84:
        n = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) == 84 + n * 50:
            dtype = np.dtype([("normal", "<f4", (3,)),
                              ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
            triangles = np.frombuffer(raw, dtype=dtype, count=n, offset=84)["vertices"].astype(float)
        else:
            triangles = ascii_stl(raw)
    else:
        triangles = ascii_stl(raw)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError(f"Invalid STL triangles: {path}")
    return triangles


def ascii_stl(raw):
    vertices = []
    for line in raw.decode("ascii").splitlines():
        words = line.split()
        if words and words[0].lower() == "vertex":
            if len(words) != 4:
                raise ValueError("Invalid ASCII STL vertex")
            vertices.append([float(x) for x in words[1:]])
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


def transform(points, T):
    return points @ T[:3, :3].T + T[:3, 3]


def cylinder():
    theta = np.linspace(0, 2*np.pi, 49)
    faces = []
    for a, b in zip(theta[:-1], theta[1:]):
        p = np.array([[0, .03*np.cos(a), .03*np.sin(a)],
                      [.31, .03*np.cos(a), .03*np.sin(a)],
                      [.31, .03*np.cos(b), .03*np.sin(b)],
                      [0, .03*np.cos(b), .03*np.sin(b)]])
        faces.extend([p[[0,1,2]], p[[0,2,3]],
                      np.array([[0,0,0],p[0],p[3]]),
                      np.array([[.31,0,0],p[2],p[1]])])
    return np.asarray(faces)


def frame(ax, T, label, length=.06, dashed=False):
    p = T[:3, 3]
    for i, color in enumerate(("red", "green", "blue")):
        end = p + length*T[:3,i]
        ax.plot(*np.stack([p,end]).T, color=color,
                ls="--" if dashed else "-", lw=1.7)
    ax.text(*p, label, fontsize=7)


def limits(ax, xyz):
    lo, hi = xyz.min(0), xyz.max(0)
    center = (lo+hi)/2
    half = max(float(np.max(hi-lo))*.57, .06)
    ax.set_xlim(center[0]-half, center[0]+half)
    ax.set_ylim(center[1]-half, center[1]+half)
    ax.set_zlim(center[2]-half, center[2]+half)
    ax.set_box_aspect((1,1,1))
    ax.set_proj_type("ortho")
    ax.set(xlabel="Base x [m]", ylabel="Base y [m]", zlabel="Base z [m]")
    ax.view_init(24,-55)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--eval-dir", type=Path, required=True)
    p.add_argument("--task-envelope", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--max-designs", type=int, default=3)
    p.add_argument("--frames", type=int, default=80)
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--no-gif", action="store_true")
    p.add_argument("--mesh-alpha", type=float, default=0.55)
    p.add_argument("--detail-mesh-alpha", type=float, default=0.15)
    a = p.parse_args()
    if min(a.max_designs, a.fps) < 1 or a.frames < 2:
        p.error("Positive max-designs/fps and frames >= 2 required")
    if not 0 <= a.mesh_alpha <= 1 or not 0 <= a.detail_mesh_alpha <= 1:
        p.error("Mesh alpha must lie in [0, 1]")
    task = TaskEnvelope(a.task_envelope)
    robot = RobotModel()
    checker = TibiaCollisionChecker(robot)
    meshes = []
    mesh_info = []
    for obj in checker.geom.geometryObjects:
        path = Path(obj.meshPath)
        triangles = stl_triangles(path)
        scale = np.asarray(obj.meshScale, float)
        meshes.append(triangles * scale)
        mesh_info.append(dict(name=obj.name, path=str(path), sha256=digest(path), triangles=len(triangles)))
    paths = sorted(a.eval_dir.rglob("whole_trajectory_path.npz"))
    selected = {}
    for path in paths:
        selected.setdefault(path.parent.parent, path)
    paths = list(selected.values())[:a.max_designs]
    if not paths:
        raise FileNotFoundError(f"No saved trajectories in {a.eval_dir}")
    a.output_dir.mkdir(parents=True, exist_ok=False)
    report = dict(schema="robot_tibia_visual_review_v1", task_json_sha256=digest(a.task_envelope/"task.json"),
                  task_arrays_sha256=digest(a.task_envelope/"task_arrays.npz"),
                  urdf_sha256=robot.urdf_sha256, meshes=mesh_info, paths=[],
                  scope="Read-only offline visualization; not a new audit or hardware safety certificate")
    local_cylinder = cylinder()
    for number, path in enumerate(paths,1):
        with np.load(path, allow_pickle=False) as z:
            t, Q = np.asarray(z["time_s"],float), np.asarray(z["q"],float)
            A, BF = matrix(z["A"]), matrix(z["X_BF"])
        if t.ndim != 1 or Q.ndim != 2 or len(t) != len(Q) or len(t)<2 or not np.isfinite(t).all() or not np.isfinite(Q).all() or np.any(np.diff(t)<=0):
            raise ValueError(f"Invalid path arrays: {path}")
        FK = np.array([matrix(robot.fk_ee(q)) for q in Q])
        BT = np.array([BF @ matrix(task.pose_at(float(v))) for v in t])
        actual_BT = FK @ A
        target_E = BT @ np.linalg.inv(A)
        ep = np.linalg.norm(FK[:,:3,3]-target_E[:,:3,3],axis=1)*1000
        er = np.rad2deg(Rotation.from_matrix(np.swapaxes(target_E[:,:3,:3],1,2)@FK[:,:3,:3]).magnitude())
        tp = np.linalg.norm(actual_BT[:,:3,3]-BT[:,:3,3],axis=1)*1000
        tr = np.rad2deg(Rotation.from_matrix(np.swapaxes(BT[:,:3,:3],1,2)@actual_BT[:,:3,:3]).magnitude())
        prefix = f"design_{number:02d}"
        title = path.parent.parent.name
        def geometry(q):
            pin.updateGeometryPlacements(robot._model,robot._data,checker.geom,checker.geom_data,robot._configuration(q))
            return [transform(m,pose.homogeneous) for m,pose in zip(meshes,checker.geom_data.oMg)]
        world_points = [np.zeros((1,3)),BF[:3,3][None],BT[:,:3,3],FK[:,:3,3]]
        for k in (0,len(t)//2,len(t)-1):
            world_points.extend([m.reshape(-1,3) for m in geometry(Q[k])])
            world_points.append(transform(local_cylinder,BT[k]).reshape(-1,3))
        world_points = np.vstack(world_points)
        close_points = np.vstack([transform(local_cylinder,BT[k]).reshape(-1,3) for k in range(len(t))]+[FK[:,:3,3]])
        def scene(ax,q,desired,actual,ee,caption,close=False,camera=(24,-55)):
            ax.clear()
            for obj,triangles in zip(checker.geom.geometryObjects,geometry(q)):
                ax.add_collection3d(Poly3DCollection(triangles,facecolor="darkorange" if obj.name=="lbr_link_7_0" else "silver",edgecolor="none",alpha=a.detail_mesh_alpha if close else a.mesh_alpha))
            poses = robot.frame_poses(q)
            skeleton = np.array([matrix(poses[name])[:3,3] for name in robot.chain_frames])
            ax.plot(*skeleton.T, "o-", color="midnightblue", lw=2.0,
                    ms=3, label="Kinematic frame chain (not body surface)")
            ax.add_collection3d(Poly3DCollection(transform(local_cylinder,actual),facecolor="steelblue",edgecolor="none",alpha=.32))
            # Three centerline rings plus axis expose the independently requested pose.
            for x in (0.,.155,.31):
                theta=np.linspace(0,2*np.pi,49)
                ring=np.stack([np.full_like(theta,x),.03*np.cos(theta),.03*np.sin(theta)],axis=1)
                ax.plot(*transform(ring,desired).T,color="magenta",lw=1,ls="--")
            ax.plot(*transform(np.array([[0,0,0],[.31,0,0]]),desired).T,color="magenta",ls="--")
            ax.plot(*target_E[:,:3,3].T,color="darkorange",lw=1,label="Required EE path")
            ax.plot(*FK[:,:3,3].T,color="black",ls="--",lw=1,label="FK EE path")
            frame(ax,actual,"T actual",.045)
            frame(ax,desired,"T requested",.035,True)
            frame(ax,ee,"E actual",.04)
            frame(ax,desired @ np.linalg.inv(A),"E required",.035,True)
            if not close:
                frame(ax,np.eye(4),"B",.10)
                frame(ax,BF,"F",.08)
            limits(ax,close_points if close else world_points)
            ax.view_init(elev=camera[0], azim=camera[1])
            ax.set_title(caption,fontsize=9)
        indices=sorted(set([0,len(t)//2,len(t)-1,int(np.argmax(ep))]))
        fig=plt.figure(figsize=(16,4.5*len(indices)))
        for row,k in enumerate(indices):
            for col,close in enumerate((False,True)):
                ax=fig.add_subplot(len(indices),2,2*row+col+1,projection="3d")
                scene(ax,Q[k],BT[k],actual_BT[k],FK[k],f"{'Interface detail' if close else 'Full robot'} | t={t[k]:.2f} s\nEE error {ep[k]:.3f} mm / {er[k]:.3f} deg",close)
        fig.suptitle(title+"\nSilver: URDF collision meshes; orange: link 7; blue: attached tibia; magenta dashed: requested tibia\nRGB axes = x/y/z. Geometry review, not a safety certificate.",fontsize=12)
        fig.tight_layout(rect=(0,0,1,.94))
        fig.savefig(a.output_dir/f"{prefix}_scenes.png",dpi=140)
        plt.close(fig)
        fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        for j in range(Q.shape[1]):
            axes[0].plot(t,np.rad2deg(Q[:,j]),label=f"J{j+1}")
        axes[0].set_ylabel("Joint angle [deg]"); axes[0].legend(ncol=7)
        axes[1].plot(t,ep,label="EE"); axes[1].plot(t,tp,label="Tibia origin"); axes[1].set_ylabel("Position error [mm]"); axes[1].legend()
        axes[2].plot(t,er,label="EE"); axes[2].plot(t,tr,label="Tibia"); axes[2].set_ylabel("Orientation error [deg]"); axes[2].set_xlabel("Task time [s]"); axes[2].legend()
        for ax in axes: ax.grid(alpha=.25)
        fig.suptitle(title+" — saved-node FK versus independently requested task")
        fig.tight_layout(); fig.savefig(a.output_dir/f"{prefix}_errors.png",dpi=150); plt.close(fig)
        fig = plt.figure(figsize=(15,5))
        for col,(elev,azim,label) in enumerate(((24,-55,"Oblique"),(90,-90,"Top"),(0,-90,"Side")),1):
            ax = fig.add_subplot(1,3,col,projection="3d")
            scene(ax,Q[0],BT[0],actual_BT[0],FK[0],label+" — initial configuration",True,(elev,azim))
        fig.suptitle(title+" — frame-focused review\nFaint meshes; navy frame-chain skeleton; RGB axes x/y/z; blue actual tibia; magenta requested")
        fig.tight_layout(rect=(0,0,1,.88))
        fig.savefig(a.output_dir/f"{prefix}_frames_multiview.png",dpi=160)
        plt.close(fig)
        if not a.no_gif:
            times=np.linspace(t[0],t[-1],a.frames)
            qi=np.column_stack([np.interp(times,t,Q[:,j]) for j in range(Q.shape[1])])
            fig=plt.figure(figsize=(13,10))
            views = [(False,(24,-55),"Whole robot — oblique"),
                     (False,(15,45),"Whole robot — opposite side"),
                     (True,(90,-90),"Interface — top"),
                     (True,(0,-90),"Interface — side")]
            axs=[fig.add_subplot(2,2,i+1,projection="3d") for i in range(len(views))]
            def update(k):
                ee=matrix(robot.fk_ee(qi[k])); actual=ee@A; desired=BF@matrix(task.pose_at(float(times[k])))
                for ax,(close,camera,label) in zip(axs,views):
                    scene(ax,qi[k],desired,actual,ee,label,close,camera)
                fig.suptitle(f"{title} | t={times[k]:.2f} s | {(t[-1]-t[0])/((a.frames-1)/a.fps):.1f}x playback\nBlue: attached tibia; magenta: requested; RGB: x/y/z; joint-linear interpolation",fontsize=10)
                return []
            ani=FuncAnimation(fig,update,frames=a.frames,interval=1000/a.fps,blit=False)
            ani.save(a.output_dir/f"{prefix}_motion.gif",writer=PillowWriter(fps=a.fps),dpi=85)
            plt.close(fig)
        report["paths"].append(dict(label=prefix,path=str(path.resolve()),path_sha256=digest(path),
             max_ee_position_error_mm=float(ep.max()),max_ee_orientation_error_deg=float(er.max()),
             max_tibia_position_error_mm=float(tp.max()),max_tibia_orientation_error_deg=float(tr.max()),
             selection="First saved path per candidate, sorted order; not capability ranking"))
        print("Completed",title,flush=True)
        (a.output_dir/"checks.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    (a.output_dir/"README.md").write_text("# Robot and tibia trajectory review\n\nScenes show actual URDF collision meshes, not clinical anatomy or visual CAD. Blue is the rigidly attached cylinder; magenta is the independently requested task pose. RGB axes denote x/y/z. Errors compare saved robot FK with the requested task. GIFs sample uniformly in physical task time using joint-linear interpolation; they are not new audits.\n\nThese historical paths are selected by candidate, not capability. No hardware capacity, collision-free continuous motion, loading normalization, or clinical safety certification is implied. See checks.json for input identities and numerical errors.\n")
    print("Review written to",a.output_dir)


if __name__ == "__main__":
    main()
