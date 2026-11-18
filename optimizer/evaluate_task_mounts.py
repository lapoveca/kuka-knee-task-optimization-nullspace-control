#!/usr/bin/env python3
"""Bounded fixed-placement mount evaluation using TaskEnvelope for VV or IE.
No task-name/coordinate-specific solver branches. Default/new minimum reserve5deg.
No collisions,dynamics,force commands or global-optimality claim.
"""
import argparse,json,csv,time,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from task_envelope import TaskEnvelope,transform
from robot_model import RobotModel
from attachment import make_attachment_candidate
from tibia_task_map import TibiaTaskMap
from wrench_margin import checked_margin

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--task-envelope',required=True);p.add_argument('--placement',required=True,help='JSON with frame=X_BF and X_BF matrix')
 p.add_argument('--candidates',required=True);p.add_argument('--budget',required=True);p.add_argument('--output-dir',required=True)
 p.add_argument('--reserve-deg',type=float,default=5.);p.add_argument('--stride-s',type=float,default=2.)
 p.add_argument('--edge-dt',type=float,default=.1);p.add_argument('--audit-dt',type=float,default=.01)
 p.add_argument('--seeds',type=int,default=12);p.add_argument('--max-tracks',type=int,default=5)
 p.add_argument('--max-nfev',type=int,default=200);p.add_argument('--rng-seed',type=int,default=42)
 a=p.parse_args()
 if not np.isfinite([a.reserve_deg,a.stride_s,a.edge_dt,a.audit_dt]).all() or a.reserve_deg<5 or min(a.stride_s,a.edge_dt,a.audit_dt)<=0 or min(a.seeds,a.max_tracks,a.max_nfev)<1:raise ValueError('Invalid options; this study requires reserve >=5deg')
 out=Path(a.output_dir)
 if out.exists():raise FileExistsError('Use a NEW output directory')
 task=TaskEnvelope(a.task_envelope);placement=json.loads(Path(a.placement).read_text())
 if placement.get('frame')!='X_BF':raise ValueError('Placement must explicitly declare frame X_BF')
 BF=transform(placement['X_BF']);candidates=json.loads(Path(a.candidates).read_text());budget=json.loads(Path(a.budget).read_text());limits=np.array(budget['limits_Nm'],float)
 ids=[c['id'] for c in candidates]
 if not candidates or len(ids)!=len(set(ids)) or any(not isinstance(i,str) or Path(i).name!=i or i in ('.','..') for i in ids):raise ValueError('Invalid candidate IDs')
 if limits.shape!=(7,) or np.any(limits<=0) or not np.isfinite(limits).all():raise ValueError('Invalid budget')
 robot=RobotModel();lo=robot.position_lower+np.deg2rad(a.reserve_deg);hi=robot.position_upper-np.deg2rad(a.reserve_deg)
 if np.any(lo>=hi):raise ValueError('Reserve empties joint range')
 plan=task.planning_times(a.stride_s);boundaries=np.unique([ph[k] for ph in task.phases for k in ('start_time_s','end_time_s')])
 out.mkdir(parents=True)
 def dump(path,obj):
  tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(obj,indent=2,allow_nan=False));tmp.replace(path)
 manifest=dict(schema='envelope_mount_run_v1',settings=vars(a),task_id=task.meta['task_id'],task_json_sha256=hashlib.sha256((Path(a.task_envelope)/'task.json').read_bytes()).hexdigest(),task_arrays_sha256=task.meta['task_arrays_sha256'],urdf_sha256=robot.urdf_sha256,placement=placement,candidates=candidates,budget=budget,scope='Sampled linear-joint kinematics and zero-bias task torque only; collision and dynamics excluded')
 dump(out/'inputs.json',manifest);results=[]
 def margin(q):return float(np.minimum(q-lo,hi-q).min())
 def grid(t0,t1,dt):return np.unique(np.r_[np.linspace(t0,t1,int(np.ceil((t1-t0)/dt))+1),task.time_s[(task.time_s>=t0)&(task.time_s<=t1)],boundaries[(boundaries>=t0)&(boundaries<=t1)]])
 for c in candidates:
  start=time.time();dest=out/c['id'];dest.mkdir();rng=np.random.default_rng(a.rng_seed);diagnostics=[]
  try:A=make_attachment_candidate(roll_angle_rad=np.deg2rad(c['roll_deg']),pitch_angle_rad=np.deg2rad(c['pitch_deg']),yaw_angle_rad=np.deg2rad(c['yaw_deg']),tibia_offset_m=.35)
  except ValueError as exc:
   r=dict(candidate=c['id'],status='geometry_rejected',reason=str(exc),complete_paths=0,best=None);results.append(r);dump(dest/'candidate_report.json',r);continue
  targets={}
  def target(v):
   key=float(v)
   if key not in targets:targets[key]=task.required_ee(v,BF,A)
   return targets[key]
  def errors(q,v):
   E=robot.fk_ee(q);T=target(v)
   return float(np.linalg.norm(E[:3,3]-T[:3,3])),float(Rotation.from_matrix(T[:3,:3].T@E[:3,:3]).magnitude())
  def solve(v,seed):
   T=target(v)
   def residual(q):
    E=robot.fk_ee(q);return np.r_[(E[:3,3]-T[:3,3])/.1,Rotation.from_matrix(T[:3,:3].T@E[:3,:3]).as_rotvec()]
   fit=least_squares(residual,np.clip(seed,lo+1e-10,hi-1e-10),bounds=(lo,hi),max_nfev=a.max_nfev,ftol=1e-10,xtol=1e-10,gtol=1e-10)
   pe,oe=errors(fit.x,v);pt,ot=task.tolerances(v)
   return fit.x.copy() if pe<=pt and oe<=ot and margin(fit.x)>=-1e-10 else None
  def edge(t0,q0,t1,q1,dt):
   rate=float(np.max(np.abs(q1-q0)/(t1-t0)/robot.velocity_limits));mp=mo=0.;bp=bo=0
   for v in grid(t0,t1,dt):
    pe,oe=errors(q0+(q1-q0)*(v-t0)/(t1-t0),v);pt,ot=task.tolerances(v);mp=max(mp,pe);mo=max(mo,oe);bp+=int(pe>pt+1e-12);bo+=int(oe>ot+1e-10)
   return dict(passed=bp==0 and bo==0 and rate<=1+1e-10,max_position_m=mp,max_orientation_rad=mo,velocity_utilization=rate,position_failed_samples=bp,orientation_failed_samples=bo)
  seeds=np.vstack([.5*(lo+hi),lo+(hi-lo)*rng.uniform(.05,.95,(a.seeds-1,7))]);beam=[];initial_ok=0
  for seed in seeds:
   q=solve(plan[0],seed)
   if q is not None:
    initial_ok+=1
    if all(np.max(np.abs(q-b['q'][-1]))>np.deg2rad(5) for b in beam):beam.append(dict(q=[q],margin=margin(q)))
  beam=sorted(beam,key=lambda b:-b['margin'])[:a.max_tracks];failed=None
  if not beam:failed=dict(stage='initial_ik',time_s=float(plan[0]))
  for k in range(1,len(plan)):
   if not beam:break
   nxt=[];diag=dict(layer=k,time_s=float(plan[k]),parents=len(beam),endpoint_ik_failures=0,edge_position_failures=0,edge_orientation_failures=0,edge_velocity_failures=0,accepted_edges=0)
   for parent in beam:
    q=solve(plan[k],parent['q'][-1])
    if q is None:diag['endpoint_ik_failures']+=1;continue
    check=edge(plan[k-1],parent['q'][-1],plan[k],q,a.edge_dt)
    diag['edge_position_failures']+=int(check['position_failed_samples']>0);diag['edge_orientation_failures']+=int(check['orientation_failed_samples']>0);diag['edge_velocity_failures']+=int(check['velocity_utilization']>1+1e-10)
    if check['passed']:diag['accepted_edges']+=1;nxt.append(dict(q=parent['q']+[q],margin=min(parent['margin'],margin(q))))
   beam=sorted(nxt,key=lambda b:-b['margin'])[:a.max_tracks];diagnostics.append(diag)
   if not beam:failed=dict(stage='continuation',time_s=float(plan[k]),detail=diag);break
   print(task.meta['task_id'],c['id'],f'{k}/{len(plan)-1}',f'{len(beam)} paths',flush=True)
  dump(dest/'search_diagnostics.json',dict(initial_attempts=a.seeds,initial_endpoint_acceptances=initial_ok,layers=diagnostics,failure=failed))
  complete=[];fine_failed=0
  if beam and failed is None:
   mapper=TibiaTaskMap(robot,A)
   for n,b in enumerate(beam):
    Q=np.array(b['q']);checks=[edge(plan[k],Q[k],plan[k+1],Q[k+1],a.audit_dt) for k in range(len(plan)-1)]
    if len(Q)!=len(plan) or not all(x['passed'] for x in checks):fine_failed+=1;continue
    rows=[];caps=[];group_values={g:[] for g in task.groups}
    for v in task.time_s:
     cases=task.case_indices(v)
     if not cases:continue
     groups=task.groups_at(v)
     if not groups:raise ValueError('Active load has no objective group')
     qi=np.array([np.interp(v,plan,Q[:,j]) for j in range(7)]);vals=[]
     for case in cases:
      ray=np.asarray(mapper.torque_for_wrench(qi,task.wrenches[case]),float);u=np.abs(ray)/limits
      if ray.shape!=(7,) or not np.isfinite(u).all() or np.max(u)<=0:raise ValueError('Nonfinite or unbounded ray requires explicit handling; pilot refuses ranking')
      lam=1/float(np.max(u));vals.append(lam)
      if not rows:
       cross=checked_margin(ray,-limits,limits,np.zeros(7))
       if cross['status']!='finite' or not np.isclose(lam,cross['lambda_max'],rtol=1e-7):raise ValueError('Capacity cross-check failed')
      for g in groups:group_values[g].append(lam)
      rows.append(dict(time_s=float(v),group_ids=';'.join(groups),case_index=int(case),lambda_max=lam,limiting_joint=robot.joint_names[int(np.argmax(u))]))
     caps.append(dict(time_s=float(v),lambda_min=min(vals)))
    if any(not vals for vals in group_values.values()):raise ValueError('Objective group has no active capacity samples')
    group_scores=[dict(group_id=g,min_lambda=min(vals)) for g,vals in group_values.items()]
    pd=dest/f'path_{n:02d}';pd.mkdir()
    np.savez_compressed(pd/'whole_trajectory_path.npz',time_s=plan,q=Q,A=A,X_BF=BF,X_BG=BF@np.linalg.inv(task.X_GF_reference),plan_times=plan)
    actual_margin=float(np.rad2deg(np.minimum(Q-lo,hi-Q).min()))
    rep=dict(schema='envelope_path_report_v1',status='sampled_path_pass',candidate=c['id'],task_id=task.meta['task_id'],reserve_deg=a.reserve_deg,minimum_hard_margin_deg=actual_margin+a.reserve_deg,worst_track_margin_deg=actual_margin,max_ee_error_mm=1000*max(x['max_position_m'] for x in checks),max_ee_error_deg=float(np.rad2deg(max(x['max_orientation_rad'] for x in checks))),max_velocity_utilization=max(x['velocity_utilization'] for x in checks),group_capability=group_scores,min_lambda=min(x['min_lambda'] for x in group_scores),mean_group_min_lambda=float(np.mean([x['min_lambda'] for x in group_scores])),budget_id=budget['id'],scope=manifest['scope'])
    dump(pd/'report.json',rep);dump(pd/'capability.json',caps)
    with (pd/'capability_cases.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    complete.append(dict(**rep,path=str(pd)))
  complete.sort(key=lambda r:(r['min_lambda'],r['mean_group_min_lambda'],r['worst_track_margin_deg']),reverse=True)
  r=dict(candidate=c['id'],status='sampled_path_found' if complete else 'no_path_found_with_budget',failure=failed,final_audit_rejected_paths=fine_failed,complete_paths=len(complete),best=complete[0] if complete else None,runtime_s=time.time()-start)
  dump(dest/'candidate_report.json',r);results.append(r);dump(out/'summary.json',results)
 ranked=sorted([r for r in results if r['best']],key=lambda r:(r['best']['min_lambda'],r['best']['mean_group_min_lambda'],r['best']['worst_track_margin_deg']),reverse=True)
 dump(out/'ranking.json',dict(schema='envelope_ranking_v1',rule='Provisional exact lexicographic min-lambda,mean-group-minima,margin; no near-tie policy/global optimum claim',budget=budget,ranked=ranked,all_results=results));print(json.dumps(results,indent=2))
if __name__=='__main__':main()
