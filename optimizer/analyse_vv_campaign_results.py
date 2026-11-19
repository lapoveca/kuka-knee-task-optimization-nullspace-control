#!/usr/bin/env python3
"""Read-only result-atlas analysis for a completed VV campaign.

Run from optimizer/:

    python3 analyse_vv_campaign_results.py "$FULL_CAMPAIGN_DIR"

It prints:
- campaign funnel;
- ranked-design summary;
- score-component ranges and correlations;
- arm/attachment coverage;
- collision rejection details using the actual schema;
- no-path failure summary.

It does not modify campaign artifacts.
"""
import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: python3 analyse_vv_campaign_results.py CAMPAIGN_DIR")
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        raise SystemExit(f"Campaign directory does not exist: {root}")

    summary = read_json(root / "campaign_summary.json")
    ranking = read_json(root / "ranking_guarded" / "ranking_manifest.json")
    paths = read_csv(root / "ranking_guarded" / "path_scores.csv")
    designs = read_csv(root / "ranking_guarded" / "design_ranking.csv")
    evaluation = read_json(root / "evaluation" / "summary.json")
    collision = read_json(root / "collision_checks.json")

    print("CAMPAIGN FUNNEL")
    print(f"  generated designs: {summary['candidate_designs']}")
    print(f"  screening passes: {summary['screen_passes']}")
    print(f"  audited-path designs: {summary['evaluation_counts']['audited_path_found']}")
    print(f"  no-path designs: {summary['evaluation_counts']['no_path_found_with_budget']}")
    print(f"  audited paths: {summary['audited_paths']}")
    print(f"  collision-accepted paths: {summary['collision_accepted_paths']}")
    print(f"  eligible ranked designs: {summary['ranked_designs']}")
    print(f"  status: {summary['status']}")
    print()

    numeric = [
        "min_lambda", "time_mean_active_lambda", "minimum_hard_room_deg",
        "extra_room_deg", "joint_utility", "component_min",
        "component_mean", "component_joint", "score",
    ]
    print("RANKED-DESIGN METRICS")
    print(f"{'metric':28s} {'min':>10s} {'median':>10s} {'mean':>10s} {'max':>10s}")
    for name in numeric:
        values = np.array([float(row[name]) for row in designs])
        print(
            f"{name:28s} {values.min():10.3f} {np.median(values):10.3f} "
            f"{values.mean():10.3f} {values.max():10.3f}"
        )
    print()

    print("TOP 15 DESIGNS")
    print(
        f"{'rank':>4s} {'design_id':38s} {'min_lam':>8s} "
        f"{'active_mean':>11s} {'room_deg':>9s} {'utility':>8s} {'score':>8s}"
    )
    for row in designs[:15]:
        print(
            f"{int(row['rank']):4d} {row['design_id']:38s} "
            f"{float(row['min_lambda']):8.3f} "
            f"{float(row['time_mean_active_lambda']):11.3f} "
            f"{float(row['minimum_hard_room_deg']):9.3f} "
            f"{float(row['joint_utility']):8.3f} "
            f"{float(row['score']):8.3f}"
        )
    print()

    print("SCORE-COMPONENT CONTRIBUTIONS")
    components = {
        "capability minimum": "component_min",
        "active-time capability": "component_mean",
        "joint comfort": "component_joint",
    }
    score = np.array([float(row["score"]) for row in designs])
    for label, field in components.items():
        values = np.array([float(row[field]) for row in designs])
        share = np.divide(
            values, score,
            out=np.zeros_like(values),
            where=score != 0,
        )
        print(
            f"  {label:24s} mean={values.mean():7.3f} "
            f"median_share={np.median(share):6.1%} "
            f"min={values.min():7.3f} max={values.max():7.3f}"
        )
    print()

    print("ARM AND ATTACHMENT COVERAGE")
    arm_counts = Counter(row["design_id"].split("__")[0].split("_")[-2] for row in designs)
    attachment_counts = Counter(row["design_id"].split("__")[1] for row in designs)
    print("  eligible designs by arm:")
    for arm in sorted(arm_counts):
        print(f"    {arm}: {arm_counts[arm]}")
    print("  eligible designs by attachment family:")
    for attachment, count in sorted(attachment_counts.items()):
        print(f"    {attachment}: {count}")
    print()

    print("PATH-LEVEL SCORE DISTRIBUTION")
    path_scores = np.array([float(row["score"]) for row in paths])
    eligible = np.array([row["eligible"] == "True" for row in paths])
    print(f"  scored paths: {len(paths)}")
    print(f"  eligible paths: {eligible.sum()}")
    print(f"  ineligible collision-accepted paths: {len(paths) - eligible.sum()}")
    print(f"  eligible score range: {path_scores[eligible].min():.3f} to {path_scores[eligible].max():.3f}")
    print()

    print("COLLISION REJECTIONS")
    rejected = [
        row for row in collision["results"]
        if not row["result"]["sampled_pass"]
    ]
    print(f"  rejected paths: {len(rejected)}")
    for row in rejected:
        result = row["result"]
        violating = result.get("violating_geometries", result.get("colliding_geometries", []))
        print(
            f"  {row['design_id']} / {Path(row['path']).name}: "
            f"minimum_distance_m={result['minimum_distance_m']:.6f}; "
            f"violating={violating}"
        )
    print()

    print("NO-PATH DESIGNS")
    for job in evaluation["jobs"]:
        if job["status"] != "no_path_found_with_budget":
            continue
        failure = job.get("failure", {})
        detail = failure.get("detail", {})
        print(
            f"  {job['design_id']}: stage={failure.get('stage')} "
            f"time_s={failure.get('time_s')} "
            f"endpoint_ik_failures={detail.get('endpoint_ik_failures')} "
            f"edge_position_failures={detail.get('edge_position_failures')} "
            f"edge_orientation_failures={detail.get('edge_orientation_failures')}"
        )


if __name__ == "__main__":
    main()
