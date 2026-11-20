#!/usr/bin/env python3
"""Guarded offline ranking of audited, collision-accepted saved trajectories."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from robot_model import RobotModel
from task_envelope import TaskEnvelope


def read(p):
    return json.loads(Path(p).read_text())


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_table(p,rows):
    if not rows:
        p.write_text('')
        return
    with p.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--eval-dir',type=Path,required=True)
    p.add_argument('--task-envelope',type=Path,required=True)
    p.add_argument('--collision-report',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--min-extra-deg',type=float,default=2.)
    a=p.parse_args()
    if not np.isfinite(a.min_extra_deg) or a.min_extra_deg<0: p.error('Invalid eligibility buffer')
    if a.output_dir.exists(): raise FileExistsError('Use a new ranking directory')
    task=TaskEnvelope(a.task_envelope);robot=RobotModel();cfg=read(a.weights)
    if cfg.get('schema')!='joint_score_weights_v1':raise ValueError('Unexpected score schema')
    weights=np.array([cfg[k] for k in ('w_min','w_mean','w_joint')],float)
    if not np.isfinite(weights).all() or np.any(weights<0) or not np.isclose(weights.sum(),1.,atol=1e-12,rtol=0): raise ValueError('Weights must be nonnegative and sum to 1')
    ref=float(cfg['lambda_ref'][task.meta['task_id']]);comfort=float(cfg['comfort_extra_room_deg']);sat=float(cfg['utility_saturation'])
    if not np.isfinite([ref,comfort,sat]).all() or min(ref,comfort)<=0 or sat<=1:raise ValueError('Invalid normalization/utility')
    collision=read(a.collision_report)
    if collision.get('schema')!='audited_path_collision_checks_v1' or collision['robot_urdf_sha256']!=robot.urdf_sha256:raise ValueError('Collision report model/schema mismatch')
    if collision['checker_sha256']!=sha('tibia_collision.py'):raise ValueError('Collision checker changed since report')
    accepted={}
    for r in collision['results']:
        path=Path(r['path']).resolve()
        if path in accepted:raise ValueError('Duplicate collision path')
        accepted[path]=r
    jobs=read(a.eval_dir/'summary.json')['jobs']
    rows=[];excluded=[];seen=set();identities=[]
    for job in jobs:
        if job['status']!='audited_path_found':continue
        audit_path=Path(job['independent_audit']).resolve();audit=read(audit_path)
        run=audit_path.parent.parent/'result';inp=read(run/'inputs.json')
        for key,actual in (('task_json_sha256',sha(a.task_envelope/'task.json')),('task_arrays_sha256',task.meta['task_arrays_sha256']),('urdf_sha256',robot.urdf_sha256)):
            if inp[key]!=actual:raise ValueError(f'Input identity mismatch: {key} in {run}')
        entries=audit.get('results',[]);names=[r['path'] for r in entries]
        disk={p.parent.resolve() for p in run.glob('*/path_*/whole_trajectory_path.npz')}
        if audit.get('status')!='PASS' or not entries or len(names)!=len(set(names)) or audit.get('expected_paths')!=len(entries) or audit.get('checked_paths')!=len(entries) or not all(r.get('pass_all') is True for r in entries) or disk!={(run/name).resolve() for name in names}:
            raise ValueError('Incomplete audit coverage')
        identities.append(dict(path=str(audit_path),sha256=sha(audit_path)))
        for ae in entries:
            folder=(run/ae['path']).resolve();folder.relative_to(run.resolve())
            path=folder/'whole_trajectory_path.npz'
            if path in seen:raise ValueError('Duplicate saved path')
            seen.add(path)
            cr=accepted.get(path)
            if cr is None:raise ValueError(f'Missing collision check: {path}')
            if cr['path_sha256']!=sha(path) or cr['audit_report_sha256']!=sha(audit_path):raise ValueError('Collision path/audit identity mismatch')
            if not cr['result']['sampled_pass']:
                excluded.append(dict(path=str(path),reason='collision_rejected'));continue
            report=read(folder/'report.json')
            if report['candidate']!=job['design_id']:raise ValueError('Candidate mismatch')
            reserve=float(report['reserve_deg'])
            # Match the recovered guarded policy explicitly; do not silently mix reserves.
            if reserve!=5.:raise ValueError('This guarded policy requires exactly 5 degree reserve')
            if report['budget_id']!=inp['budget']['id']:raise ValueError('Budget mismatch')
            with np.load(path,allow_pickle=False) as z:
                t=np.asarray(z['time_s'],float);Q=np.asarray(z['q'],float)
            if Q.shape!=(len(t),7) or len(t)<2 or not np.isfinite(Q).all() or not np.isfinite(t).all() or np.any(np.diff(t)<=0):raise ValueError('Invalid path arrays')
            if not np.isclose(t[0],task.time_s[0],atol=1e-9,rtol=0) or not np.isclose(t[-1],task.time_s[-1],atol=1e-9,rtol=0):raise ValueError('Incomplete task duration')
            room=float(np.rad2deg(np.minimum(Q-robot.position_lower,robot.position_upper-Q).min()))
            np.testing.assert_allclose(room,ae['minimum_hard_margin_deg'],atol=1e-6,rtol=0)
            np.testing.assert_allclose(room,report['minimum_hard_margin_deg'],atol=1e-6,rtol=0)
            if room<reserve-1e-8:raise ValueError('Joint reserve violation')
            with (folder/'capability_cases.csv').open(newline='') as f:cases=list(csv.DictReader(f))
            caps={}
            for c in cases:
                key=(float(c['time_s']),int(c['case_index']));value=float(c['lambda_max'])
                if key in caps or not np.isfinite(value) or value<=0:raise ValueError('Invalid/duplicate capacity cache')
                caps[key]=value
            integral=duration=0.;minimum=float('inf')
            for ph in task.phases:
                active=ph['active_case_indices']
                if not active:continue
                lo,hi=ph['start_time_s'],ph['end_time_s']
                ts=np.unique(np.r_[lo,task.time_s[(task.time_s>lo)&(task.time_s<hi)],hi])
                vals=np.array([min(caps[(float(v),int(k))] for k in active) for v in ts])
                minimum=min(minimum,float(vals.min()))
                integral+=float(np.sum(np.diff(ts)*(vals[:-1]+vals[1:])/2))
                duration+=float(ts[-1]-ts[0])
            if duration<=0:raise ValueError('No active loading duration')
            np.testing.assert_allclose(minimum,report['min_lambda'],atol=1e-7,rtol=1e-7)
            for group,audited in ae['group_min_lambda'].items():
                values=[float(r['lambda_max']) for r in cases if group in r['group_ids'].split(';')]
                if not values:raise ValueError('Missing group samples')
                np.testing.assert_allclose(min(values),audited,atol=1e-7,rtol=1e-7)
            mean=integral/duration;extra=room-reserve
            utility=-math.expm1(-math.log(sat)*max(0.,extra)/comfort)
            components=weights*np.array([minimum/ref,mean/ref,utility])
            rows.append(dict(design_id=report['candidate'],path=str(folder),path_sha256=sha(path),
                min_lambda=minimum,time_mean_active_lambda=mean,legacy_mean_group_min_lambda=report['mean_group_min_lambda'],
                minimum_hard_room_deg=room,extra_room_deg=extra,joint_utility=utility,
                component_min=float(components[0]),component_mean=float(components[1]),component_joint=float(components[2]),
                score=float(components.sum()),eligible=bool(extra>=a.min_extra_deg-1e-8),budget_id=report['budget_id']))
    if not rows:raise ValueError('No audited collision-accepted paths')
    if len({r['budget_id'] for r in rows})!=1:raise ValueError('Mixed torque budgets')
    best={}
    for r in rows:
        if not r['eligible']:continue
        key=r['design_id']
        if key not in best or (r['score'],r['path'])>(best[key]['score'],best[key]['path']):best[key]=r
    ranked=sorted(best.values(),key=lambda r:(-r['score'],r['design_id']))
    a.output_dir.mkdir(parents=True)
    write_table(a.output_dir/'path_scores.csv',rows)
    write_table(a.output_dir/'design_ranking.csv',[dict(rank=i,**r) for i,r in enumerate(ranked,1)])
    write_table(a.output_dir/'collision_exclusions.csv',excluded)
    manifest=dict(schema='guarded_offline_ranking_v1',weights=cfg,weights_sha256=sha(a.weights),
        min_extra_deg=a.min_extra_deg,task_json_sha256=sha(a.task_envelope/'task.json'),
        task_arrays_sha256=task.meta['task_arrays_sha256'],urdf_sha256=robot.urdf_sha256,
        collision_report=str(a.collision_report.resolve()),collision_report_sha256=sha(a.collision_report),
        collision_policy={k:collision[k] for k in ('interface_geometry_names','interface_allowance_m','clearance_m','max_joint_step_rad')},
        audit_reports=identities,paths_scored=len(rows),eligible_paths=sum(r['eligible'] for r in rows),ranked_designs=len(ranked),
        definition='Active-phase trapezoidal mean of worst active case; unloaded transfers excluded; 5 degree reserve; existing saturating worst-room utility',
        source_sha256=sha(__file__),ranking=[dict(rank=i,**r) for i,r in enumerate(ranked,1)])
    (a.output_dir/'ranking_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(f"Scored {len(rows)} paths; {sum(r['eligible'] for r in rows)} eligible; {len(ranked)} ranked designs")
    for i,r in enumerate(ranked,1):print(i,r['design_id'],f"score={r['score']:.6f}",Path(r['path']).name)
    if not ranked:print('No eligible winner under this policy')


if __name__=='__main__':main()
