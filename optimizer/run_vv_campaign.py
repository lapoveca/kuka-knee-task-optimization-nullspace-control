#!/usr/bin/env python3
"""Resumable VV campaign: generate, screen, promote, evaluate, collide, rescore.
Run from optimizer/. Never edit frozen inputs while a campaign is active.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def atomic(p,obj):
    tmp=p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n');tmp.replace(p)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign-dir',type=Path,required=True)
    p.add_argument('--task-envelope',type=Path,default=Path('inputs/task_envelopes/varus_valgus_distal_force_attachment_origin'))
    p.add_argument('--placement-domain',type=Path,default=Path('configs/placement_domain.json'))
    p.add_argument('--budget',type=Path,default=Path('configs/screening_budget_historical.json'))
    p.add_argument('--weights',type=Path,default=Path('configs/joint_score_weights_guarded_v2.json'))
    p.add_argument('--placements-per-arm',type=int,default=17)
    p.add_argument('--workers',type=int,default=2)
    p.add_argument('--rng-seed',type=int,default=42)
    p.add_argument('--screen-seeds',type=int,default=4)
    p.add_argument('--screen-seconds',type=float,default=15.)
    p.add_argument('--timeout-s',type=int,default=1800)
    p.add_argument('--retry-failed',action='store_true')
    a=p.parse_args()
    if min(a.placements_per_arm,a.workers,a.timeout_s)<1 or a.screen_seeds<2 or a.screen_seconds<=0:p.error('Invalid campaign options')
    base=Path.cwd().resolve();root=a.campaign_dir.resolve()
    task=a.task_envelope.resolve();domain=a.placement_domain.resolve();budget=a.budget.resolve();weights=a.weights.resolve()
    sources=['generate_global_placements.py','screen_joint_proposals.py','promote_screened_designs.py',
             'run_parallel_design_eval.py','run_joint_design_eval.py','evaluate_task_mounts.py','audit_task_mount_run.py',
             'rescore_saved_designs.py','tibia_collision.py','task_envelope.py','attachment.py','robot_model.py',
             'transforms.py','tibia_task_map.py','workspace.py','wrench_margin.py','placement_domain.py','models/med14_runtime.urdf']
    files=[base/x for x in sources]+[Path(__file__).resolve(),task/'task.json',task/'task_arrays.npz',domain,budget,weights]
    hashes={str(f.resolve()):sha(f) for f in files}
    config=dict(schema='vv_campaign_v1',task_envelope=str(task),placement_domain=str(domain),budget=str(budget),weights=str(weights),
        placements_per_arm=a.placements_per_arm,workers=a.workers,rng_seed=a.rng_seed,screen_seeds=a.screen_seeds,
        screen_seconds=a.screen_seconds,timeout_s=a.timeout_s,files=hashes,
        evaluation=dict(reserve_deg=5.,stride_s=2.,edge_dt=.1,audit_dt=.01,seeds=12,max_tracks=5,max_nfev=200),
        collision=dict(clearance_m=0.,max_joint_step_rad=.01),promotion='all_screen_passes',min_extra_deg=2.)
    root.mkdir(parents=True,exist_ok=True)
    lock=(root/'.campaign.lock').open('a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('Campaign already running in this directory')
    frozen=root/'campaign_manifest.json'
    if frozen.exists():
        if read(frozen)!=config:raise ValueError('Campaign source/configuration changed; use a NEW campaign directory')
    else:atomic(frozen,config)
    logs=root/'logs';logs.mkdir(exist_ok=True)
    events=root/'campaign.log'
    stages=root/'stages';stages.mkdir(exist_ok=True)
    def event(message):
        line=time.strftime('%Y-%m-%dT%H:%M:%S')+' '+message
        print(line,flush=True)
        with events.open('a') as f:f.write(line+'\n')
    def verify():
        for name,value in hashes.items():
            if sha(name)!=value:raise RuntimeError('Frozen input/source changed: '+name)
    def call(stage,command):
        verify();event('START '+stage)
        atomic(stages/(stage+'.json'),dict(status='running',command=command,started=time.time()))
        with (logs/(stage+'.log')).open('a') as stream:
            stream.write('\nCOMMAND '+json.dumps(command)+'\n');stream.flush()
            env=os.environ.copy()
            for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):env[name]='1'
            proc=subprocess.Popen(command,cwd=base,stdout=stream,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            try:rc=proc.wait()
            except BaseException:
                if proc.poll() is None:
                    try:os.killpg(proc.pid,signal.SIGINT)
                    except ProcessLookupError:pass
                    try:proc.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        try:os.killpg(proc.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        proc.wait()
                atomic(stages/(stage+'.json'),dict(status='interrupted',command=command,updated=time.time()))
                raise
        if rc:
            atomic(stages/(stage+'.json'),dict(status='error',exit_code=rc,command=command,updated=time.time()))
            raise RuntimeError(f'{stage} exited {rc}; inspect {logs/(stage+".log")}')
        verify();event('FINISH '+stage)
        atomic(stages/(stage+'.json'),dict(status='subprocess_complete',command=command,updated=time.time()))
    py=sys.executable
    generation=root/'generation';proposals=generation/'proposals.json'
    screening=root/'screening';promoted=root/'promoted.json';evaluation=root/'evaluation'
    collisions=root/'collision_checks.json';ranking=root/'ranking_guarded'
    try:
        from task_envelope import TaskEnvelope
        from robot_model import RobotModel
        import numpy as np
        task_obj=TaskEnvelope(task)
        if task_obj.loading_model_id!='catalog_normalized_attachment_origin_distal_force_v1':raise ValueError('Expected final normalized catalog loading')
        if not np.isclose(task_obj.time_s[-1]-task_obj.time_s[0],320.):raise ValueError('Expected 320 s final VV task')
        if not proposals.exists():
            if generation.exists():raise RuntimeError('Incomplete generation directory exists; inspect it before retrying (not deleted automatically)')
            call('01_generation',[py,'generate_global_placements.py','--task-envelope',str(task),'--placement-domain',str(domain),
                '--rng-seed',str(a.rng_seed),'--placements-per-arm',str(a.placements_per_arm),'--allow-placement-count-override',
                '--max-proposal-attempts','100000','--output-dir',str(generation)])
        pool=read(proposals)
        if pool['task_json_sha256']!=sha(task/'task.json'):raise ValueError('Generated proposal task identity mismatch')
        ids=[d['id'] for d in pool['designs']]
        if len(ids)!=36*a.placements_per_arm or len(ids)!=len(set(ids)):raise ValueError('Unexpected proposal population')
        placements={}
        for d in pool['designs']:
            key=tuple(np.asarray(d['placement']['X_BF']).ravel())
            group=placements.setdefault(key,dict(arm=d['placement']['arm'],attachments=[]))
            group['attachments'].append(d['attachment']['attachment_id'])
        arms={arm:0 for arm in ('+X','-X','+Y','-Y')}
        expected={'axial_inline'}|{f'peripheral_{i:02d}' for i in range(8)}
        for group in placements.values():
            if len(group['attachments'])!=9 or set(group['attachments'])!=expected:raise ValueError('Attachment population differs')
            arms[group['arm']]+=1
        if any(n!=a.placements_per_arm for n in arms.values()):raise ValueError('Arm placement quotas differ')
        event(f'POPULATION {len(placements)} placements / {len(ids)} designs')
        # Always invoke the resumable screener: its own signature and records validate resume.
        call('02_screening',[py,'screen_joint_proposals.py','--proposals',str(proposals),'--output-dir',str(screening),
            '--seeds-per-pose',str(a.screen_seeds),'--max-nfev','200','--rng-seed',str(a.rng_seed),'--design-seconds',str(a.screen_seconds)])
        screen=read(screening/'summary.json');records=screen['results']
        if len(records)!=len(ids) or {r['design_id'] for r in records}!=set(ids):raise ValueError('Incomplete screening coverage')
        if any(r['status']=='error' for r in records):raise RuntimeError('Screening contains software errors; inspect records, do not treat as infeasibility')
        passing=sum(r['status']=='pointwise_screen_pass' for r in records)
        if not passing:
            atomic(root/'campaign_summary.json',dict(status='complete_no_screen_passes',candidate_designs=len(ids),screen_passes=0))
            event('COMPLETE: no screen-passing designs');return 0
        if not promoted.exists():
            call('03_promotion',[py,'promote_screened_designs.py','--proposals',str(proposals),'--screen-dir',str(screening),
                '--task-envelope',str(task),'--promote-count',str(len(ids)),'--output',str(promoted)])
        else:
            from promote_screened_designs import validate_promotion_inputs
            checked=validate_promotion_inputs(pool,screen,proposals,screening,task)
            m=read(promoted)
            if m.get('verified_screening')!=checked:raise ValueError('Existing promotion provenance differs')
        m=read(promoted)
        wanted={r['design_id'] for r in records if r['status']=='pointwise_screen_pass'}
        if {d['id'] for d in m['designs']}!=wanted or len(m['designs'])!=len(wanted):raise ValueError('Promotion omitted/duplicated passes')
        command=[py,'run_parallel_design_eval.py','--manifest',str(promoted),'--output-dir',str(evaluation),'--workers',str(a.workers),
                 '--budget',str(budget),'--rng-seed',str(a.rng_seed),'--timeout-s',str(a.timeout_s)]
        for key,value in config['evaluation'].items():command+=['--'+key.replace('_','-'),str(value)]
        if a.retry_failed:command.append('--retry-failed')
        call('04_evaluation',command)
        ev=read(evaluation/'summary.json');jobs=ev['jobs']
        if len(jobs)!=passing or {j['design_id'] for j in jobs}!=wanted:raise ValueError('Evaluation job coverage differs')
        if any(j['status'] not in ('audited_path_found','no_path_found_with_budget') for j in jobs):raise RuntimeError('Unresolved evaluation jobs')
        from tibia_collision import TibiaCollisionChecker
        robot=RobotModel();checker=TibiaCollisionChecker(robot)
        cached=read(collisions) if collisions.exists() else {}
        policy=dict(checker_sha256=sha('tibia_collision.py'),robot_urdf_sha256=robot.urdf_sha256,
                    interface_geometry_names=sorted(checker.interface_geometry_names),interface_allowance_m=checker.interface_allowance_m,
                    clearance_m=0.,max_joint_step_rad=.01)
        old={}
        if cached:
            if any(cached.get(k)!=v for k,v in policy.items()):raise ValueError('Collision policy changed')
            old={r['path']:r for r in cached['results']}
            if len(old)!=len(cached['results']):raise ValueError('Duplicate collision cache entries')
        rows=[];seen=set();event('START 05_collision')
        for job in jobs:
            if job['status']!='audited_path_found':continue
            ap=Path(job['independent_audit']).resolve();audit=read(ap);run=ap.parent.parent/'result'
            if audit['status']!='PASS' or not all(r['pass_all'] for r in audit['results']):raise ValueError('Audit not passed')
            declared={(run/r['path']).resolve() for r in audit['results']}
            disk={z.parent.resolve() for z in run.glob('*/path_*/whole_trajectory_path.npz')}
            if declared!=disk or len(declared)!=audit['expected_paths'] or len(declared)!=audit['checked_paths']:raise ValueError('Audit path coverage differs')
            for folder in sorted(declared):
                folder.relative_to(run.resolve());path=folder/'whole_trajectory_path.npz'
                if path in seen:raise ValueError('Duplicate audited trajectory')
                seen.add(path);key=str(path);ph=sha(path);ah=sha(ap)
                row=old.get(key)
                if row is None or row['path_sha256']!=ph or row['audit_report_sha256']!=ah:
                    with np.load(path,allow_pickle=False) as z:result=checker.check_path(z['q'],z['A'],clearance_m=0.,max_joint_step_rad=.01)
                    row=dict(design_id=job['design_id'],path=key,path_sha256=ph,audit_report=str(ap),audit_report_sha256=ah,result=result)
                rows.append(row)
                atomic(collisions,dict(schema='audited_path_collision_checks_v1',**policy,paths_checked=len(rows),
                    paths_passed=sum(r['result']['sampled_pass'] for r in rows),results=rows))
                event('COLLISION '+job['design_id']+'/'+folder.name+' '+('PASS' if row['result']['sampled_pass'] else 'REJECT'))
        passed=sum(r['result']['sampled_pass'] for r in rows)
        final=dict(candidate_designs=len(ids),screen_passes=passing,evaluation_counts=ev['counts'],audited_paths=len(rows),collision_accepted_paths=passed)
        if passed:
            rank_command=[py,'rescore_saved_designs.py','--eval-dir',str(evaluation),'--task-envelope',str(task),
                          '--collision-report',str(collisions),'--weights',str(weights),'--output-dir',str(ranking),'--min-extra-deg','2']
            if not ranking.exists():call('06_ranking',rank_command)
            else:
                rm=read(ranking/'ranking_manifest.json')
                if rm['collision_report_sha256']!=sha(collisions) or rm['weights_sha256']!=sha(weights) or rm['source_sha256']!=sha('rescore_saved_designs.py'):
                    raise ValueError('Existing ranking identity differs; inspect before replacing')
            final['ranked_designs']=read(ranking/'ranking_manifest.json')['ranked_designs']
            final['status']='complete' if final['ranked_designs'] else 'complete_no_eligible_winner'
        else:final['status']='complete_no_collision_accepted_paths'
        verify();atomic(root/'campaign_summary.json',final);event('COMPLETE '+json.dumps(final));return 0
    except KeyboardInterrupt:
        atomic(root/'campaign_summary.json',dict(status='interrupted',resumable=True,updated=time.time()))
        event('INTERRUPTED: rerun the identical command');return 130
    except Exception as exc:
        atomic(root/'campaign_summary.json',dict(status='error',error=f'{type(exc).__name__}: {exc}',updated=time.time()))
        event('ERROR '+str(exc));raise


if __name__=='__main__':raise SystemExit(main())
