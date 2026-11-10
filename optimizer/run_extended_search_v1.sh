#!/usr/bin/env bash
# Run from knee_task_optimizer. No robot commands. All results in new folders.
set -Eeuo pipefail
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
ROOT="$PWD"
RUN="${RUN_ROOT:-output/extended_global_20261104_v1}"
MODE="${1:---preflight}"
mkdir -p "$RUN"
exec 9>"$RUN/.driver.lock"
flock -n 9 || { echo "Another driver owns $RUN"; exit 1; }

fingerprint() {
python3 - <<'PY'
from pathlib import Path
import hashlib,json
files=list(Path('.').glob('*.py'))+list(Path('models').glob('*.urdf'))+list(Path('configs').glob('*.json'))
for task in ('vv_supervisor_scaled_distal_force_v1','ie_table_v1'):
    files+=list((Path('inputs/task_envelopes')/task).glob('task*'))
print(hashlib.sha256(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files)) if p.is_file()},sort_keys=True).encode()).hexdigest())
PY
}

runner() {
    local manifest="$1" batch="$2" limit="$3"
    python3 -u run_joint_design_eval_v2.py --manifest "$manifest" --batch-dir "$batch" \
      --budget configs/screening_budget_historical_v1.json --rng-seed 42 \
      --reserve-deg 5 --stride-s 2 --edge-dt 0.1 --audit-dt 0.01 \
      --seeds 12 --max-tracks 5 --max-nfev 200 --timeout-s 1200 --max-jobs "$limit"
}

if [[ "$MODE" == --preflight ]]; then
    python3 - <<'PY'
import ast, importlib
from pathlib import Path
names=['generate_global_placements_v3.py','screen_joint_proposals_v2.py',
       'promote_screened_designs_v1.py','run_joint_design_eval_v2.py',
       'evaluate_task_mounts_v1.py','audit_task_mount_run_v1.py']
for name in names:
    text=Path(name).read_text();compile(text,name,'exec');tree=ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='magnitude' and (node.args or node.keywords):
            raise RuntimeError(f'{name}:{node.lineno}: invalid magnitude(arguments) call')
    print('Syntax/API-pattern check:',name)
for name in ('numpy','scipy','pinocchio','robot_model','attachment','task_envelope','tibia_task_map','wrench_margin'):
    importlib.import_module(name)
from task_envelope import TaskEnvelope
from robot_model import RobotModel
r=RobotModel()
for name in ('vv_supervisor_scaled_distal_force_v1','ie_table_v1'):
    t=TaskEnvelope('inputs/task_envelopes/'+name)
    assert len(t.time_s)>1
print('Model and both task envelopes loaded')
PY
    for script in generate_global_placements_v3.py screen_joint_proposals_v2.py promote_screened_designs_v1.py run_joint_design_eval_v2.py; do
        python3 "$script" --help > /dev/null
    done
    fingerprint > "$RUN/source_fingerprint.txt"
    runner inputs/global_promotion_VV_robotinformed_v1.json "$RUN/preflight_VV" 1 | tee "$RUN/preflight_VV.log"
    runner inputs/global_promotion_IE_robotinformed_v1.json "$RUN/preflight_IE" 1 | tee "$RUN/preflight_IE.log"
    python3 - "$RUN" <<'PY'
import json,sys
from pathlib import Path
r=Path(sys.argv[1])
for task in ('VV','IE'):
    s=json.loads((r/f'preflight_{task}'/'summary.json').read_text())
    jobs=s['jobs']
    assert jobs and all(j['status']=='audited_path_found' for j in jobs),(task,jobs)
    print(task,'live smoke test PASS:',len(jobs),'audited design(s)')
(r/'PREFLIGHT_PASSED').write_text('Both known-task live evaluation/audit tests passed.\n')
PY
    echo "PREFLIGHT PASSED. Launch --run only after reading this output."
    exit 0
fi
[[ "$MODE" == --run ]] || { echo 'Use --preflight or --run'; exit 2; }
[[ -f "$RUN/PREFLIGHT_PASSED" ]] || { echo 'Run --preflight first'; exit 1; }
[[ "$(fingerprint)" == "$(cat "$RUN/source_fingerprint.txt")" ]] || { echo 'Workspace source/config changed since preflight; use a new RUN_ROOT and preflight'; exit 1; }

run_task() {
    local task="$1" envelope="$2" seed="$3"
    local dir="$RUN/$task"
    mkdir -p "$dir"
    echo "=== $task started $(date -Is) ==="
    if [[ ! -f "$dir/proposals/proposals.json" ]]; then
        python3 -u generate_global_placements_v3.py --task-envelope "$envelope" \
          --placement-count 40 --extra-attachments-per-placement 9 --rng-seed "$seed" \
          --max-attempts 10000 --output-dir "$dir/proposals" || return 1
    fi
    python3 - "$dir/proposals/proposals.json" <<'PY' || return 1
import json,sys,collections
p=json.load(open(sys.argv[1]));ds=p['designs'];ids=[d['id'] for d in ds]
assert p['schema']=='joint_proposal_pool_v2'
assert len(ids)==len(set(ids)),'Duplicate IDs'
counts=collections.Counter(tuple(round(v,9) for row in d['placement']['X_BF'] for v in row) for d in ds)
assert len(counts)==40 and len(ds)==400 and set(counts.values())=={10},dict(counts)
print('Pool validated: 40 placements x 10 attachments = 400 pairs')
PY
    python3 -u screen_joint_proposals_v2.py --proposals "$dir/proposals/proposals.json" \
      --output-dir "$dir/screen" --max-designs 0 --seeds-per-pose 4 \
      --max-nfev 120 --rng-seed 42 --design-seconds 120 || return 1
    python3 - "$dir/screen/summary.json" <<'PY' || return 1
import json,sys,collections
s=json.load(open(sys.argv[1]));c=collections.Counter(r['status'] for r in s['results'])
print('Screen statuses:',dict(c))
assert s['recorded_designs']==s['scheduled_designs']==400
assert c['error']==0,'Unexpected screening errors: inspect checkpoints'
assert c['pointwise_screen_pass']>0,'No passes; do not run empty evaluation'
PY
    if [[ ! -f "$dir/promotion.json" ]]; then
        python3 -u promote_screened_designs_v1.py --proposals "$dir/proposals/proposals.json" \
          --screen-dir "$dir/screen" --task-envelope "$envelope" --promote-count 400 \
          --output "$dir/promotion.json" || return 1
    fi
    # Promote ALL screen passes: no diversity-based truncation in this study.
    runner "$dir/promotion.json" "$dir/eval" 0 || return 1
    python3 - "$dir/eval/summary.json" <<'PY' || return 1
import json,sys,collections
s=json.load(open(sys.argv[1]));c=collections.Counter(j['status'] for j in s['jobs'])
print('Whole-task statuses:',dict(c))
assert s['recorded_designs']==s['scheduled_designs']
assert c['error']==0,'Inspect error checkpoints; no automatic expensive retries'
print('Timeouts, if present, remain unfinished search outcomes; no success claim for them.')
PY
    echo "=== $task completed $(date -Is) ==="
}
failed=0
for task in VV IE; do
    if [[ "$task" == VV ]]; then
        env_path=inputs/task_envelopes/vv_supervisor_scaled_distal_force_v1; seed=6101
    else
        env_path=inputs/task_envelopes/ie_table_v1; seed=6102
    fi
    [[ "$(fingerprint)" == "$(cat "$RUN/source_fingerprint.txt")" ]] || { echo 'Source/config drift: stopping'; exit 1; }
    if run_task "$task" "$env_path" "$seed" > "$RUN/${task}_pipeline.log" 2>&1; then
        echo "$task stages completed; see $RUN/${task}_pipeline.log"
    else
        echo "$task stopped on a validation/stage error; evidence preserved. See $RUN/${task}_pipeline.log"
        failed=1
    fi
done
echo "Finished $(date -Is). Raw trajectories/audits saved. Guarded rescoring and figures follow separately."
exit "$failed"
