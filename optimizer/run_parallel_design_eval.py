#!/usr/bin/env python3
"""Parallel evaluation coordinator; one isolated resumable batch per design."""
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


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic(path,obj):
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
    temp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--workers',type=int,default=2)
    p.add_argument('--budget',type=Path,default=Path('configs/screening_budget_historical.json'))
    p.add_argument('--reserve-deg',type=float,default=5.)
    p.add_argument('--stride-s',type=float,default=2.)
    p.add_argument('--edge-dt',type=float,default=.1)
    p.add_argument('--audit-dt',type=float,default=.01)
    p.add_argument('--seeds',type=int,default=12)
    p.add_argument('--max-tracks',type=int,default=5)
    p.add_argument('--max-nfev',type=int,default=200)
    p.add_argument('--rng-seed',type=int,default=42)
    p.add_argument('--timeout-s',type=int,default=1800)
    p.add_argument('--retry-failed',action='store_true')
    a=p.parse_args()
    if a.workers<1: p.error('workers must be positive')
    base=Path.cwd().resolve()
    manifest=json.loads(a.manifest.read_text())
    if manifest.get('schema')!='joint_promotion_manifest_v1': raise ValueError('Unexpected manifest schema')
    designs=manifest['designs'];ids=[d['id'] for d in designs]
    if not ids or len(ids)!=len(set(ids)) or any(Path(i).name!=i or i in ('.','..') for i in ids): raise ValueError('Invalid design IDs')
    envelope=Path(manifest['task_envelope']).resolve()
    files=[Path(__file__).resolve(),a.manifest.resolve(),a.budget.resolve(),
           envelope/'task.json',envelope/'task_arrays.npz']
    files += [base/name for name in ('run_joint_design_eval.py','evaluate_task_mounts.py','audit_task_mount_run.py',
              'attachment.py','robot_model.py','task_envelope.py','transforms.py','tibia_task_map.py','workspace.py','wrench_margin.py','models/med14_runtime.urdf')]
    options={key:getattr(a,key) for key in ('reserve_deg','stride_s','edge_dt','audit_dt','seeds','max_tracks','max_nfev','rng_seed','timeout_s')}
    signature=dict(schema='parallel_design_evaluation_v1',files={str(f.resolve()):sha(f) for f in files},
                   design_ids=ids,budget=str(a.budget.resolve()),options=options)
    root=a.output_dir.resolve();root.mkdir(parents=True,exist_ok=True)
    lock=(root/'.coordinator.lock').open('a+')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise RuntimeError('Coordinator already running for this output directory')
    frozen=root/'manifest.json'
    if frozen.exists():
        if json.loads(frozen.read_text())!=signature: raise ValueError('Evaluation inputs/source/options changed; use a new output directory')
    else: atomic(frozen,signature)
    manifests=root/'design_manifests';logs=root/'logs'
    manifests.mkdir(exist_ok=True);logs.mkdir(exist_ok=True)
    for d in designs:
        m=dict(manifest);m['designs']=[d]
        target=manifests/(d['id']+'.json')
        if target.exists():
            if json.loads(target.read_text())!=m: raise ValueError('Worker manifest changed')
        else: atomic(target,m)
    def checkpoint(d):
        return root/'evaluation'/d['id']/d['id']/f'seed_{a.rng_seed}'/'checkpoint.json'
    states=root/'worker_states'
    states.mkdir(exist_ok=True)
    def state_path(d):return states/(d['id']+'.json')
    def worker_state(d):
        path=state_path(d)
        return json.loads(path.read_text()) if path.exists() else {}
    def validate_completed(d,cp):
        job=json.loads(cp.read_text())
        if job.get('design_id')!=d['id'] or job.get('rng_seed')!=a.rng_seed:
            raise ValueError('Checkpoint design/seed mismatch')
        attempt=int(job['attempt'])
        work=cp.parent/f'attempt_{attempt:03d}'
        result=work/'result'
        inputs=json.loads((result/'inputs.json').read_text())
        checks={'task_json_sha256':sha(envelope/'task.json'),
                'task_arrays_sha256':sha(envelope/'task_arrays.npz'),
                'urdf_sha256':sha(base/'models/med14_runtime.urdf')}
        for key,value in checks.items():
            if inputs.get(key)!=value:raise ValueError('Completed input identity mismatch: '+key)
        if inputs.get('budget')!=json.loads(a.budget.read_text()):raise ValueError('Completed budget differs')
        if inputs.get('settings',{}).get('reserve_deg')!=a.reserve_deg:raise ValueError('Completed reserve differs')
        if inputs.get('placement')!=d['placement']:raise ValueError('Completed placement differs')
        expected_candidate=[{'id':d['id'],'attachment':d['attachment'],'A':d['A'],'X_TE':d['X_TE']}]
        if inputs.get('candidates')!=expected_candidate:raise ValueError('Completed candidate differs')
        ranking=json.loads((result/'ranking.json').read_text())
        results=ranking['all_results']
        if len(results)!=1 or results[0]['candidate']!=d['id']:raise ValueError('Completed ranking candidate differs')
        outcome=results[0]
        if job['status']=='no_path_found_with_budget':
            if outcome['status']!='no_path_found_with_budget' or outcome['complete_paths']!=0:
                raise ValueError('No-path checkpoint contradicts evaluator')
            if list(result.glob('*/path_*/whole_trajectory_path.npz')):raise ValueError('No-path checkpoint contains saved paths')
            return
        if outcome['status']!='sampled_path_found' or outcome['best']!=job.get('best'):
            raise ValueError('Completed best-path checkpoint differs')
        audit_path=Path(job['independent_audit']).resolve()
        if audit_path!=(work/'audit'/'audit_report.json').resolve():raise ValueError('Audit path outside expected attempt')
        audit=json.loads(audit_path.read_text());entries=audit.get('results',[])
        names=[r['path'] for r in entries]
        expected=int(outcome['complete_paths'])
        if audit.get('status')!='PASS' or expected<1 or audit.get('checked_paths')!=expected or audit.get('expected_paths')!=expected or len(entries)!=expected or len(names)!=len(set(names)) or not all(r.get('pass_all') is True for r in entries):
            raise ValueError('Completed audit coverage invalid')
        disk={path.parent.resolve() for path in result.glob('*/path_*/whole_trajectory_path.npz')}
        declared={(result/name).resolve() for name in names}
        if disk!=declared:raise ValueError('Completed audit/path sets differ')
        for folder in declared:
            folder.relative_to(result.resolve())
            with (folder/'whole_trajectory_path.npz').open('rb') as stream:
                if not stream.read(1):raise ValueError('Empty trajectory file')
            if not (folder/'report.json').is_file() or not (folder/'capability_cases.csv').is_file():raise ValueError('Missing completed metrics')
        if Path(job['best']['path']).resolve() not in declared:raise ValueError('Selected path absent from audit')
    def collect():
        jobs=[]
        for d in designs:
            cp=checkpoint(d)
            job=json.loads(cp.read_text()) if cp.exists() else dict(design_id=d['id'],status='pending_or_interrupted')
            state=worker_state(d)
            if state.get('status') in ('running','interrupted') and job['status'] not in ('audited_path_found','no_path_found_with_budget'):
                job=dict(design_id=d['id'],status=state['status'],previous_checkpoint=job)
            jobs.append(job)
        counts={}
        for job in jobs: counts[job['status']]=counts.get(job['status'],0)+1
        atomic(root/'summary.json',dict(scheduled=len(ids),counts=counts,jobs=jobs,
               scope='Trajectory evaluation and independent audit only; collision filtering and weighted rescoring are subsequent stages'))
        return counts
    def verify_frozen():
        if any(sha(path)!=value for path,value in signature['files'].items()): raise RuntimeError('Frozen source/input changed during execution')
    queue=[]
    interrupted_ids=set()
    for d in designs:
        cp=checkpoint(d)
        if cp.exists():
            status=json.loads(cp.read_text())['status']
            if status in ('audited_path_found','no_path_found_with_budget'):
                validate_completed(d,cp)
                continue
            state=worker_state(d)
            if state.get('status') in ('running','interrupted'):
                interrupted_ids.add(d['id'])
            if not a.retry_failed and d['id'] not in interrupted_ids: continue
        state=worker_state(d)
        if state.get('status')=='running' and state.get('pid'):
            try:os.kill(int(state['pid']),0)
            except ProcessLookupError:pass
            else:raise RuntimeError('A recorded worker may still be alive; inspect PID '+str(state['pid']))
        queue.append(d)
    running={};errors=[]
    def stop_workers():
        for proc,stream,d in running.values():
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        until=time.monotonic()+8
        for proc,stream,d in running.values():
            try:proc.wait(timeout=max(.1,until-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait()
            stream.close()
            atomic(state_path(d),dict(design_id=d['id'],status='interrupted',pid=proc.pid,time=time.time()))
    try:
        while queue or running:
            while queue and len(running)<a.workers:
                verify_frozen()
                d=queue.pop(0)
                command=[sys.executable,str(base/'run_joint_design_eval.py'),'--manifest',str(manifests/(d['id']+'.json')),
                         '--batch-dir',str(root/'evaluation'/d['id']),'--budget',str(a.budget.resolve())]
                for key,value in options.items(): command += ['--'+key.replace('_','-'),str(value)]
                if a.retry_failed or d['id'] in interrupted_ids:command.append('--retry-failed')
                stream=(logs/(d['id']+'.log')).open('a')
                stream.write('\nCOMMAND '+json.dumps(command)+'\n');stream.flush()
                env=os.environ.copy()
                for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):env[name]='1'
                proc=subprocess.Popen(command,cwd=base,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                running[proc.pid]=(proc,stream,d)
                atomic(state_path(d),dict(design_id=d['id'],status='running',pid=proc.pid,time=time.time()))
                print('START',d['id'],flush=True)
            for pid,(proc,stream,d) in list(running.items()):
                rc=proc.poll()
                if rc is None:continue
                stream.close();del running[pid]
                cp=checkpoint(d)
                if rc!=0 or not cp.exists(): errors.append(d['id'])
                status=json.loads(cp.read_text())['status'] if cp.exists() else 'worker_process_error'
                if rc==0 and status in ('audited_path_found','no_path_found_with_budget'):
                    validate_completed(d,cp)
                atomic(state_path(d),dict(design_id=d['id'],status=status,pid=proc.pid,exit_code=rc,time=time.time()))
                print('DONE',d['id'],status,'exit',rc,flush=True)
                collect()
            if running:time.sleep(.5)
        verify_frozen()
    except BaseException:
        stop_workers();collect();raise
    counts=collect();print(json.dumps(counts,indent=2))
    if errors or any(k in counts for k in ('error','timeout','pending_or_interrupted','interrupted','running')):return 1
    return 0


if __name__=='__main__':
    raise SystemExit(main())
