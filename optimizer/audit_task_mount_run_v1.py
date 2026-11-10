#!/usr/bin/env python3
"""Separate saved-output audit for envelope-based evaluator; no IK.
Independently recompute sampled tracking, bounds/rates, group capacity, and FD
mapping at3postures. Shared model/envelope assumptions remain explicit.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from task_envelope import TaskEnvelope
from robot_model import RobotModel
from attachment import make_attachment_candidate
from tibia_task_map import TibiaTaskMap

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--task-envelope',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--check-dt',type=float,default=.01)
 a=p.parse_args();root=Path(a.run_dir);out=Path(a.output_dir)
 if out.exists():raise FileExistsError('Use NEW output directory')
 if not np.isfinite(a.check_dt) or a.check_dt<=0:raise ValueError('Invalid dt')
 inp=json.loads((root/'inputs.json').read_text());rank=json.loads((root/'ranking.json').read_text());task=TaskEnvelope(a.task_envelope);robot=RobotModel()
 if inp['task_json_sha256']!=hashlib.sha256((Path(a.task_envelope)/'task.json').read_bytes()).hexdigest() or inp['urdf_sha256']!=robot.urdf_sha256:raise ValueError('Task/model identity changed')
 if rank['budget']!=inp['budget']:raise ValueError('Budget mismatch')
 limits=np.array(inp['budget']['limits_Nm'],float);reserve=float(inp['settings']['reserve_deg'])
 if reserve<5 or not np.isfinite(reserve):raise ValueError('New study requires at least5deg reserve')
 BF=np.array(inp['placement']['X_BF'],float);candidates={c['id']:c for c in inp['candidates']};results=[]
 # Separate interpolation implementation from generator's target cache.
 rotation=Slerp(np.array(task.time_s,dtype=float,copy=True),Rotation.from_matrix(np.array(task.X_FT[:,:3,:3],dtype=float,order='C',copy=True)))
 for file in sorted(root.glob('*/path_*/whole_trajectory_path.npz')):
  try:
   report=json.loads((file.parent/'report.json').read_text());cid=file.parent.parent.name;c=candidates[cid]
   with np.load(file,allow_pickle=False) as z:t,Q,A,savedBF=[np.array(z[k],float) for k in ('time_s','q','A','X_BF')]
   if t.ndim!=1 or len(t)<2 or Q.shape!=(len(t),7) or np.any(np.diff(t)<=0) or not np.isfinite(Q).all() or not np.isfinite(t).all():raise ValueError('Bad saved arrays')
   if abs(t[0]-task.time_s[0])>1e-9 or abs(t[-1]-task.time_s[-1])>1e-9:raise ValueError('Incomplete task')
   expected=make_attachment_candidate(roll_angle_rad=np.deg2rad(c['roll_deg']),pitch_angle_rad=np.deg2rad(c['pitch_deg']),yaw_angle_rad=np.deg2rad(c['yaw_deg']),tibia_offset_m=.35)
   if not np.allclose(A,expected,atol=1e-10,rtol=0) or not np.allclose(savedBF,BF,atol=1e-10,rtol=0):raise ValueError('Design identity mismatch')
   if float(report['reserve_deg'])!=reserve or report['budget_id']!=inp['budget']['id']:raise ValueError('Report policy differs')
   hard=float(np.rad2deg(np.minimum(Q-robot.position_lower,robot.position_upper-Q).min()))
   if hard<reserve-1e-8:raise ValueError('Reserved bounds fail')
   rate=float(np.max(np.abs(np.diff(Q,axis=0))/np.diff(t)[:,None]/robot.velocity_limits))
   boundaries=[ph[k] for ph in task.phases for k in ('start_time_s','end_time_s')]
   ts=np.unique(np.r_[np.linspace(t[0],t[-1],int(np.ceil((t[-1]-t[0])/a.check_dt))+1),task.time_s,t,boundaries]);bad=0;mp=mo=0.
   for v in ts:
    qi=np.array([np.interp(v,t,Q[:,j]) for j in range(7)]);F=np.eye(4);F[:3,:3]=rotation(float(v)).as_matrix();F[:3,3]=[np.interp(v,task.time_s,task.X_FT[:,j,3]) for j in range(3)];target=BF@F@np.linalg.inv(A);E=robot.fk_ee(qi)
    pe=float(np.linalg.norm(E[:3,3]-target[:3,3]));oe=float(Rotation.from_matrix(E[:3,:3]@target[:3,:3].T).magnitude());pt,ot=task.tolerances(v);bad+=int(pe>pt+1e-12 or oe>ot+1e-10);mp=max(mp,pe);mo=max(mo,oe)
   mapper=TibiaTaskMap(robot,A);values={g:[] for g in task.groups};fd=0.
   for i in np.unique(np.linspace(0,len(Q)-1,3,dtype=int)):
    qi=Q[i];T=robot.fk_ee(qi)@A;J=np.empty((6,7));h=1e-6
    for j in range(7):
     dq=np.zeros(7);dq[j]=h;P=robot.fk_ee(qi+dq)@A;M=robot.fk_ee(qi-dq)@A;J[:3,j]=T[:3,:3].T@(P[:3,3]-M[:3,3])/(2*h);J[3:,j]=T[:3,:3].T@Rotation.from_matrix(P[:3,:3]@M[:3,:3].T).as_rotvec()/(2*h)
    direct=np.column_stack([mapper.torque_for_wrench(qi,w) for w in np.eye(6)]);fd=max(fd,float(np.max(np.abs(direct-J.T))))
   for v in task.time_s:
    cases=task.case_indices(v)
    if not cases:continue
    qi=np.array([np.interp(v,t,Q[:,j]) for j in range(7)])
    for case in cases:
     ray=np.asarray(mapper.torque_for_wrench(qi,task.wrenches[case]),float);u=float(np.max(np.abs(ray)/limits))
     if not np.isfinite(u) or u<=0:raise ValueError('Invalid/unbounded capacity ray')
     for g in task.groups_at(v):values[g].append(1/u)
   scores={g:min(vals) for g,vals in values.items()};stored={v['group_id']:v['min_lambda'] for v in report['group_capability']}
   if set(stored)!=set(scores):raise ValueError('Group coverage differs')
   delta=max(abs(scores[g]-stored[g]) for g in scores);margin_diff=abs(hard-reserve-report['worst_track_margin_deg'])
   passed=bad==0 and rate<=1+1e-10 and fd<=1e-5 and delta<=1e-6 and margin_diff<=1e-6
   r=dict(path=str(file.parent.relative_to(root)),candidate=cid,pass_all=bool(passed),nodes=len(t),bad_tracking_samples=bad,max_ee_error_mm=mp*1000,max_ee_error_deg=float(np.rad2deg(mo)),minimum_hard_margin_deg=hard,extra_margin_beyond_reserve_deg=hard-reserve,max_velocity_utilization=rate,torque_map_fd_max_abs_error=fd,group_capacity_max_abs_difference=delta,group_min_lambda=scores)
  except Exception as exc:r=dict(path=str(file.parent.relative_to(root)),pass_all=False,error=f'{type(exc).__name__}: {exc}')
  results.append(r);print(json.dumps(r,indent=2),flush=True)
 expected=sum(v['complete_paths'] for v in rank['all_results']);status='PASS' if results and len(results)==expected and all(r['pass_all'] for r in results) else ('NO_PATHS_TO_AUDIT' if not results and expected==0 else 'FAIL')
 out.mkdir(parents=True);payload=dict(status=status,expected_paths=expected,checked_paths=len(results),results=results,scope='Separate sampled saved-path audit, same robot/task model; no collision,dynamics or continuous certification');(out/'audit_report.json').write_text(json.dumps(payload,indent=2,allow_nan=False));print('Audit status:',status)
 if status=='FAIL':raise SystemExit(1)
if __name__=='__main__':main()
