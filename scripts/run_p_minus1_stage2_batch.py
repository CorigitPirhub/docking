#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.dockbench import FAMILIES, load_manifest, load_representatives, dataset_root, family_label
from docking.p_minus1_baselines import CAPABILITY_METHOD_IDS, CORE_ABLATION_METHOD_IDS, METHOD_SPECS, STRONG_METHOD_IDS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"
BATCH_FULL_METHOD_IDS = ["co_bcfd", "T_hard_switch", "T_lattice_pbvs", "T_parking_hierarchical", "T_coop_dist_blend"]


@dataclass(frozen=True)
class SceneMethodResult:
    scene_id: str
    split: str
    family_code: str
    difficulty: str
    subset_tag: str
    seed: int
    style: str
    method_id: str
    family: str
    success: bool
    reason: str
    failure_category: str
    done_time_s: float
    collision: bool
    min_clearance_m: float
    trajectory_cost: float
    path_length_m: float
    fallback_count: int
    visual_loss_count: int
    replan_count: int
    leader_relocation_distance_m: float
    leader_relocation_time_s: float
    follower_detour_ratio: float
    accel_smoothness_max: float
    steer_rate_smoothness_max: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MethodAggregate:
    method_id: str
    family: str
    subset: str
    n: int
    success_rate: float
    collision_rate: float
    done_time_mean_success: float | None
    done_time_p90_success: float | None
    trajectory_cost_mean_success: float | None
    min_clearance_min: float
    fallback_mean: float
    visual_loss_mean: float
    replan_mean: float
    accel_smoothness_p90: float | None
    steer_smoothness_p90: float | None
    fail_reasons: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["fail_reasons"] = dict(self.fail_reasons)
        return data


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="P-1 Stage-2 DockBench batch evaluation.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--out-dir", type=str, default=str(ROOT / "artifacts"), help="Output directory.")
    p.add_argument("--max-time-s", type=float, default=40.0, help="Global time cap per scene.")
    p.add_argument("--skip-gifs", action="store_true", help="Skip representative GIFs.")
    p.add_argument("--workers", type=int, default=4, help="Parallel scene workers.")
    p.add_argument("--bootstrap-samples", type=int, default=2000, help="Bootstrap samples for paired CI.")
    return p.parse_args()


def _run_stage1_once(*, scene_json: str, methods: Iterable[str], out_dir: Path, max_time_s: float, skip_gifs: bool) -> dict[str, Any]:
    payload = json.loads(Path(scene_json).read_text(encoding="utf-8"))
    seed = int(payload["scenario"]["seed"])
    result_path = out_dir / f"p_minus1_stage1_results_seed{seed}.json"
    if result_path.exists():
        cached = json.loads(result_path.read_text(encoding="utf-8"))
        got = {str(run["metrics"]["method_id"]) for run in cached.get("runs", [])}
        need = set(methods)
        if need.issubset(got):
            return cached
    cmd = [
        sys.executable,
        str(STAGE1_SCRIPT),
        "--scenario-json",
        str(scene_json),
        "--methods",
        ",".join(methods),
        "--out-dir",
        str(out_dir),
        "--max-time-s",
        str(float(max_time_s)),
    ]
    if skip_gifs:
        cmd.append("--skip-gifs")
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    return json.loads(result_path.read_text(encoding="utf-8"))


def _extract_rows(result: dict[str, Any], spec) -> list[SceneMethodResult]:
    rows: list[SceneMethodResult] = []
    for run in result["runs"]:
        metrics = run["metrics"]
        method_id = str(metrics["method_id"])
        rows.append(
            SceneMethodResult(
                scene_id=spec.scene_id,
                split=spec.split,
                family_code=spec.family,
                difficulty=spec.difficulty,
                subset_tag=spec.subset_tag,
                seed=int(spec.seed),
                style=str(spec.style),
                method_id=method_id,
                family=METHOD_SPECS.get(method_id, METHOD_SPECS["co_bcfd"]).family,
                success=bool(metrics["success"]),
                reason=str(metrics["reason"]),
                failure_category=str(metrics.get("failure_category", metrics["reason"])),
                done_time_s=float(metrics["done_time_s"]),
                collision=bool(metrics["collision"]),
                min_clearance_m=float(metrics["min_clearance_m"]),
                trajectory_cost=float(metrics["trajectory_cost"]),
                path_length_m=float(metrics["path_length_m"]),
                fallback_count=int(metrics.get("fallback_count", 0)),
                visual_loss_count=int(metrics.get("visual_loss_count", 0)),
                replan_count=int(metrics.get("replan_count", 0)),
                leader_relocation_distance_m=float(metrics.get("leader_relocation_distance_m", 0.0)),
                leader_relocation_time_s=float(metrics.get("leader_relocation_time_s", 0.0)),
                follower_detour_ratio=float(metrics.get("follower_detour_ratio", 1.0)),
                accel_smoothness_max=float(metrics.get("accel_smoothness_max", 0.0)),
                steer_rate_smoothness_max=float(metrics.get("steer_rate_smoothness_max", 0.0)),
            )
        )
    return rows


def _run_scene_bundle(spec, *, methods: list[str], out_dir: Path, max_time_s: float, skip_gifs: bool) -> list[SceneMethodResult]:
    result = _run_stage1_once(scene_json=spec.scenario_json, methods=methods, out_dir=out_dir, max_time_s=min(float(max_time_s), float(spec.max_time_s)), skip_gifs=skip_gifs)
    return _extract_rows(result, spec)


def _mean_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.mean(np.asarray(values, dtype=float)))


def _quantile_or_none(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=float), float(q)))


def _rows_for_subset(rows: list[SceneMethodResult], subset: str) -> list[SceneMethodResult]:
    if subset == "Overall":
        return rows
    return [row for row in rows if row.family_code == subset]


def _aggregate(rows: list[SceneMethodResult], method_id: str, subset: str) -> MethodAggregate:
    rel = [row for row in _rows_for_subset(rows, subset) if row.method_id == method_id]
    succ = [row for row in rel if row.success]
    fail_reasons = Counter(row.failure_category for row in rel if not row.success)
    return MethodAggregate(
        method_id=method_id,
        family=METHOD_SPECS[method_id].family,
        subset=subset,
        n=len(rel),
        success_rate=float(sum(1 for row in rel if row.success) / max(len(rel), 1)),
        collision_rate=float(sum(1 for row in rel if row.collision) / max(len(rel), 1)),
        done_time_mean_success=_mean_or_none([row.done_time_s for row in succ]),
        done_time_p90_success=_quantile_or_none([row.done_time_s for row in succ], 0.90),
        trajectory_cost_mean_success=_mean_or_none([row.trajectory_cost for row in succ]),
        min_clearance_min=float(min((row.min_clearance_m for row in rel), default=float("inf"))),
        fallback_mean=float(np.mean([row.fallback_count for row in rel])) if rel else 0.0,
        visual_loss_mean=float(np.mean([row.visual_loss_count for row in rel])) if rel else 0.0,
        replan_mean=float(np.mean([row.replan_count for row in rel])) if rel else 0.0,
        accel_smoothness_p90=_quantile_or_none([row.accel_smoothness_max for row in rel], 0.90),
        steer_smoothness_p90=_quantile_or_none([row.steer_rate_smoothness_max for row in rel], 0.90),
        fail_reasons=dict(fail_reasons),
    )


def _paired(rows: list[SceneMethodResult], subset: str, method_a: str, method_b: str) -> list[tuple[SceneMethodResult, SceneMethodResult]]:
    rel = _rows_for_subset(rows, subset)
    lut_a = {row.scene_id: row for row in rel if row.method_id == method_a}
    lut_b = {row.scene_id: row for row in rel if row.method_id == method_b}
    keys = sorted(set(lut_a) & set(lut_b))
    return [(lut_a[key], lut_b[key]) for key in keys]


def _bootstrap_ci(values: np.ndarray, *, seed: int, samples: int) -> dict[str, float]:
    if values.size == 0:
        return {"mean": math.nan, "ci95_low": math.nan, "ci95_high": math.nan, "n": 0}
    rng = np.random.default_rng(int(seed))
    draws = rng.integers(0, values.size, size=(int(samples), values.size))
    means = np.mean(values[draws], axis=1)
    return {
        "mean": float(np.mean(values)),
        "ci95_low": float(np.quantile(means, 0.025)),
        "ci95_high": float(np.quantile(means, 0.975)),
        "n": int(values.size),
    }


def _paired_success_ci(rows: list[SceneMethodResult], subset: str, method_a: str, method_b: str, *, samples: int) -> dict[str, float]:
    diffs = np.asarray([float(int(a.success) - int(b.success)) for a, b in _paired(rows, subset, method_a, method_b)], dtype=float)
    return _bootstrap_ci(diffs, seed=11, samples=samples)


def _paired_metric_ci(rows: list[SceneMethodResult], subset: str, method_a: str, method_b: str, attr: str, *, samples: int) -> dict[str, float]:
    values: list[float] = []
    for a, b in _paired(rows, subset, method_a, method_b):
        if a.success and b.success:
            values.append(float(getattr(a, attr) - getattr(b, attr)))
    return _bootstrap_ci(np.asarray(values, dtype=float), seed=17, samples=samples)


def _select_best_strong(rows: list[SceneMethodResult], subset: str) -> str:
    best_mid = STRONG_METHOD_IDS[0]
    best_key = (1.0, math.inf)
    for mid in STRONG_METHOD_IDS:
        agg = _aggregate(rows, mid, subset)
        key = (-agg.success_rate, math.inf if agg.done_time_mean_success is None else agg.done_time_mean_success)
        if key < best_key:
            best_mid = mid
            best_key = key
    return best_mid


def _save_success_heatmap(rows: list[SceneMethodResult], scene_specs, methods: list[str], out_path: Path) -> None:
    mat = np.zeros((len(methods), len(scene_specs)), dtype=float)
    lut = {(row.method_id, row.scene_id): float(1.0 if row.success else 0.0) for row in rows}
    for i, mid in enumerate(methods):
        for j, spec in enumerate(scene_specs):
            mat[i, j] = lut.get((mid, spec.scene_id), np.nan)
    fig, ax = plt.subplots(figsize=(18.0, 3.4 + 0.35 * len(methods)), dpi=170)
    im = ax.imshow(mat, cmap="RdYlGn", vmin=0.0, vmax=1.0, aspect="auto")
    ax.set_yticks(np.arange(len(methods)))
    ax.set_yticklabels(methods)
    ax.set_xticks(np.arange(len(scene_specs)))
    ax.set_xticklabels([f"{spec.family}-{spec.difficulty}:{spec.scene_id.split('-')[-1]}" for spec in scene_specs], rotation=55, ha="right", fontsize=7)
    ax.set_title("P-1 Stage-2 DockBench success heatmap")
    fig.colorbar(im, ax=ax, fraction=0.025, pad=0.02)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _save_subset_bars(aggregates: list[MethodAggregate], methods: list[str], out_path: Path) -> None:
    subsets = ["Overall", *FAMILIES]
    fig, axes = plt.subplots(1, len(subsets), figsize=(20.5, 4.8), dpi=170, sharey=True)
    for ax, subset in zip(axes, subsets):
        vals = [next((agg.success_rate for agg in aggregates if agg.method_id == mid and agg.subset == subset), 0.0) for mid in methods]
        ax.bar(np.arange(len(methods)), vals, color=["tab:blue" if mid == "co_bcfd" else "tab:gray" for mid in methods])
        ax.set_ylim(0.0, 1.05)
        ax.set_title(subset if subset == "Overall" else f"{subset}:{family_label(subset)}")
        ax.set_xticks(np.arange(len(methods)))
        ax.set_xticklabels(methods, rotation=35, ha="right", fontsize=8)
        ax.grid(alpha=0.25, axis="y")
    axes[0].set_ylabel("success")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _save_failure_bars(rows: list[SceneMethodResult], methods: list[str], out_path: Path) -> None:
    categories = sorted({row.failure_category for row in rows if not row.success})
    if not categories:
        categories = ["none"]
    x = np.arange(len(categories))
    width = 0.8 / max(len(methods), 1)
    fig, ax = plt.subplots(figsize=(16.2, 5.0), dpi=170)
    for idx, mid in enumerate(methods):
        counts = [sum(1 for row in rows if row.method_id == mid and (not row.success) and row.failure_category == cat) for cat in categories]
        ax.bar(x + (idx - 0.5 * (len(methods) - 1)) * width, counts, width=width, label=mid)
    ax.set_xticks(x)
    ax.set_xticklabels(categories, rotation=35, ha="right", fontsize=8)
    ax.set_title("P-1 Stage-2 failure clusters")
    ax.grid(alpha=0.25, axis="y")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _save_ablation_bars(rows: list[SceneMethodResult], out_path: Path) -> None:
    methods = ["co_bcfd", *CORE_ABLATION_METHOD_IDS]
    subsets = ["Overall", "SC", "FC", "EC"]
    fig, axes = plt.subplots(1, len(subsets), figsize=(18.0, 4.8), dpi=170, sharey=True)
    for ax, subset in zip(axes, subsets):
        vals = [_aggregate(rows, mid, subset).success_rate for mid in methods]
        ax.bar(np.arange(len(methods)), vals, color=["tab:blue" if mid == "co_bcfd" else "tab:orange" for mid in methods])
        ax.set_title(subset)
        ax.set_ylim(0.0, 1.05)
        ax.set_xticks(np.arange(len(methods)))
        ax.set_xticklabels(methods, rotation=35, ha="right", fontsize=8)
        ax.grid(alpha=0.25, axis="y")
    axes[0].set_ylabel("success")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _write_protocol(out_path: Path, *, tuning, test, challenge) -> None:
    lines = [
        "# P-1 Stage-2 DockBench Protocol\n\n",
        "## Frozen Dataset\n",
        f"- tuning: `{len(tuning)}`\n",
        f"- test: `{len(test)}`\n",
        f"- challenge: `{len(challenge)}`\n",
        "\n## Tuning\n",
    ]
    for spec in tuning:
        lines.append(f"- `{spec.scene_id}` / `{spec.family}` / `{spec.difficulty}` / `{spec.split}`\n")
    lines.append("\n## Test\n")
    for spec in test:
        lines.append(f"- `{spec.scene_id}` / `{spec.family}` / `{spec.difficulty}` / `{spec.split}`\n")
    lines.append("\n## Challenge\n")
    for spec in challenge:
        lines.append(f"- `{spec.scene_id}` / `{spec.family}` / `{spec.difficulty}` / `{spec.split}`\n")
    out_path.write_text("".join(lines), encoding="utf-8")


def _write_report(out_path: Path, *, tuning, test, challenge, stage0_audit: dict[str, Any], aggregates: list[MethodAggregate], comparisons: dict[str, Any], plots: dict[str, str], rep_gifs: list[str]) -> None:
    agg_map = {(agg.method_id, agg.subset): agg for agg in aggregates}
    lines = [
        "# P-1 Stage-2 DockBench Batch Report\n\n",
        "## Dataset\n",
        f"- tuning: `{len(tuning)}` / test: `{len(test)}` / challenge: `{len(challenge)}`\n",
        f"- stage0_pass: `{stage0_audit.get('pass', False)}`\n",
        "\n## Overall\n\n",
        "| method | family | success_rate | collision_rate | T_done_mean[s] | cost_mean | min_clear_min[m] |\n",
        "|---|---|---:|---:|---:|---:|---:|\n",
    ]
    for mid in [*BATCH_FULL_METHOD_IDS, *CORE_ABLATION_METHOD_IDS]:
        agg = agg_map[(mid, "Overall")]
        td = "nan" if agg.done_time_mean_success is None else f"{agg.done_time_mean_success:.2f}"
        cost = "nan" if agg.trajectory_cost_mean_success is None else f"{agg.trajectory_cost_mean_success:.2f}"
        lines.append(f"| `{mid}` | `{agg.family}` | {agg.success_rate:.3f} | {agg.collision_rate:.3f} | {td} | {cost} | {agg.min_clearance_min:.3f} |\n")
    lines.append("\n## Family Breakdown\n")
    for subset in FAMILIES:
        lines.append(f"\n### {subset} / {family_label(subset)}\n\n")
        lines.append("| method | success_rate | collision_rate | T_done_mean[s] | cost_mean | fallback_mean | visual_loss_mean | replan_mean |\n")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for mid in [*BATCH_FULL_METHOD_IDS, *CORE_ABLATION_METHOD_IDS]:
            agg = agg_map[(mid, subset)]
            td = "nan" if agg.done_time_mean_success is None else f"{agg.done_time_mean_success:.2f}"
            cost = "nan" if agg.trajectory_cost_mean_success is None else f"{agg.trajectory_cost_mean_success:.2f}"
            lines.append(f"| `{mid}` | {agg.success_rate:.3f} | {agg.collision_rate:.3f} | {td} | {cost} | {agg.fallback_mean:.2f} | {agg.visual_loss_mean:.2f} | {agg.replan_mean:.2f} |\n")
    lines.append("\n## Compatibility View\n")
    for subset in ("CF", "EC"):
        agg = agg_map[("co_bcfd", subset)]
        lines.append(f"- `{subset}` / `{family_label(subset)}` co_bcfd success=`{agg.success_rate:.3f}` collision=`{agg.collision_rate:.3f}`\n")
    lines.append("\n## Comparisons\n")
    for key, value in comparisons.items():
        lines.append(f"- `{key}`: `{json.dumps(value, ensure_ascii=False)}`\n")
    lines.append("\n## Plots\n")
    for key, value in plots.items():
        lines.append(f"- `{key}`: `{value}`\n")
    lines.append("\n## Representative GIFs\n")
    for value in rep_gifs:
        lines.append(f"- `{value}`\n")
    out_path.write_text("".join(lines), encoding="utf-8")


def _load_stage0_audit(ds_root: Path) -> dict[str, Any]:
    audit_path = ds_root / "dockbench_v1_stage0_audit.json"
    if audit_path.exists():
        return json.loads(audit_path.read_text(encoding="utf-8"))
    return {"pass": False, "family_admission": {}, "rows": []}


def _parallel_run(specs, *, methods: list[str], out_dir: Path, max_time_s: float, workers: int) -> list[SceneMethodResult]:
    rows: list[SceneMethodResult] = []
    with ThreadPoolExecutor(max_workers=max(1, int(workers))) as executor:
        futs = {
            executor.submit(_run_scene_bundle, spec, methods=methods, out_dir=out_dir, max_time_s=max_time_s, skip_gifs=True): spec.scene_id
            for spec in specs
        }
        for fut in as_completed(futs):
            rows.extend(fut.result())
    return rows


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    stage1_tmp = out_dir / "p_minus1_stage2_stage1_runs"
    stage1_tmp.mkdir(parents=True, exist_ok=True)

    tuning_manifest = load_manifest(ds_root, split="tuning")
    test_manifest = load_manifest(ds_root, split="test")
    challenge_manifest = load_manifest(ds_root, split="challenge")
    protocol_path = out_dir / "P_MINUS1_BASELINE_PROTOCOL.md"
    _write_protocol(protocol_path, tuning=tuning_manifest, test=test_manifest, challenge=challenge_manifest)

    stage0_audit = _load_stage0_audit(ds_root)
    methods = [*BATCH_FULL_METHOD_IDS, *CORE_ABLATION_METHOD_IDS]
    test_rows = _parallel_run(test_manifest, methods=methods, out_dir=stage1_tmp, max_time_s=float(args.max_time_s), workers=int(args.workers))
    challenge_rows = _parallel_run(challenge_manifest, methods=BATCH_FULL_METHOD_IDS, out_dir=stage1_tmp, max_time_s=float(args.max_time_s), workers=int(args.workers)) if challenge_manifest else []

    aggregates: list[MethodAggregate] = []
    for subset in ["Overall", *FAMILIES]:
        for mid in methods:
            aggregates.append(_aggregate(test_rows, mid, subset))

    best_strong_cf = _select_best_strong(test_rows, "CF")
    cf_success = _paired_success_ci(test_rows, "CF", "co_bcfd", best_strong_cf, samples=int(args.bootstrap_samples))
    cf_time = _paired_metric_ci(test_rows, "CF", "co_bcfd", best_strong_cf, "done_time_s", samples=int(args.bootstrap_samples))
    cf_cost = _paired_metric_ci(test_rows, "CF", "co_bcfd", best_strong_cf, "trajectory_cost", samples=int(args.bootstrap_samples))
    ec_success = _paired_success_ci(test_rows, "EC", "co_bcfd", best_strong_cf, samples=int(args.bootstrap_samples))

    agg_map = {(agg.method_id, agg.subset): agg for agg in aggregates}
    overall = agg_map[("co_bcfd", "Overall")]
    full_cf = agg_map[("co_bcfd", "CF")]
    full_ec = agg_map[("co_bcfd", "EC")]
    strong_cf = agg_map[(best_strong_cf, "CF")]
    strong_ec = agg_map[(best_strong_cf, "EC")]
    comparisons = {
        "best_strong_cf": best_strong_cf,
        "cf_success_bootstrap": cf_success,
        "cf_time_bootstrap": cf_time,
        "cf_cost_bootstrap": cf_cost,
        "ec_success_bootstrap": ec_success,
        "stage2_gate": {
            "co_success_ge_095": bool(overall.success_rate >= 0.95 - 1e-12),
            "co_collision_zero": bool(overall.collision_rate <= 1e-12),
            "co_avg_time_le_15": bool((overall.done_time_mean_success or 1e9) <= 15.0 + 1e-12),
            "strong_cf_nonzero": bool(strong_cf.success_rate > 0.0),
        },
        "p13_common_competitive": {
            "success_noninferior": bool(full_cf.success_rate >= strong_cf.success_rate - 0.05 - 1e-12),
            "time_gain_ge_5pct": bool(strong_cf.done_time_mean_success is not None and full_cf.done_time_mean_success is not None and full_cf.done_time_mean_success <= 0.95 * strong_cf.done_time_mean_success),
            "cost_gain_ge_5pct": bool(strong_cf.trajectory_cost_mean_success is not None and full_cf.trajectory_cost_mean_success is not None and full_cf.trajectory_cost_mean_success <= 0.95 * strong_cf.trajectory_cost_mean_success),
            "success_gain_ge_10pt": bool(full_cf.success_rate >= strong_cf.success_rate + 0.10 - 1e-12),
        },
        "p13_extension_gain_ge_15pt": bool(full_ec.success_rate >= strong_ec.success_rate + 0.15 - 1e-12),
        "challenge_summary": {
            "num_rows": len(challenge_rows),
            "co_success_rate": float(sum(1 for row in challenge_rows if row.method_id == "co_bcfd" and row.success) / max(1, sum(1 for row in challenge_rows if row.method_id == "co_bcfd"))),
        },
    }

    heatmap_path = out_dir / "p_minus1_stage2_success_heatmap.png"
    subset_bar_path = out_dir / "p_minus1_stage2_subset_compare.png"
    failure_bar_path = out_dir / "p_minus1_stage2_failure_clusters.png"
    ablation_bar_path = out_dir / "p_minus1_stage2_ablation_summary.png"
    _save_success_heatmap(test_rows, test_manifest, BATCH_FULL_METHOD_IDS, heatmap_path)
    _save_subset_bars(aggregates, BATCH_FULL_METHOD_IDS, subset_bar_path)
    _save_failure_bars(test_rows, BATCH_FULL_METHOD_IDS, failure_bar_path)
    _save_ablation_bars(test_rows, ablation_bar_path)

    rep_gifs: list[str] = []
    if not bool(args.skip_gifs):
        reps = load_representatives(ds_root)
        rep_specs = [reps["CF_L2"], reps["SC_L2"], reps["FC_L2"], reps["EC_L2"]]
        for spec in rep_specs:
            family_methods = ["co_bcfd", best_strong_cf]
            if spec.family == "SC":
                family_methods.append("A_no_belief_gate")
            elif spec.family == "FC":
                family_methods.append("A_no_funnel_gate")
            elif spec.family == "EC":
                family_methods.append("A_no_stage")
            _run_stage1_once(scene_json=spec.scenario_json, methods=family_methods, out_dir=out_dir, max_time_s=min(float(args.max_time_s), float(spec.max_time_s)), skip_gifs=False)
            payload = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
            seed = int(payload["scenario"]["seed"])
            for mid in family_methods:
                rep_gifs.append(str((out_dir / f"p_minus1_stage1_{mid}_seed{seed}.gif").resolve()))

    summary = {
        "dataset_root": str(ds_root),
        "config": {
            "max_time_s": float(args.max_time_s),
            "workers": int(args.workers),
            "bootstrap_samples": int(args.bootstrap_samples),
            "methods": methods,
            "full_methods": BATCH_FULL_METHOD_IDS,
            "core_ablations": CORE_ABLATION_METHOD_IDS,
            "strong_methods": STRONG_METHOD_IDS,
            "capability_methods": CAPABILITY_METHOD_IDS,
        },
        "manifests": {
            "tuning": [spec.to_dict() for spec in tuning_manifest],
            "test": [spec.to_dict() for spec in test_manifest],
            "challenge": [spec.to_dict() for spec in challenge_manifest],
        },
        "stage0_audit": stage0_audit,
        "rows": [row.to_dict() for row in test_rows],
        "aggregates": [agg.to_dict() for agg in aggregates],
        "comparisons": comparisons,
        "plots": {
            "protocol": str(protocol_path.resolve()),
            "success_heatmap": str(heatmap_path.resolve()),
            "subset_compare": str(subset_bar_path.resolve()),
            "failure_clusters": str(failure_bar_path.resolve()),
            "ablation_summary": str(ablation_bar_path.resolve()),
        },
        "representative_gifs": rep_gifs,
    }
    json_path = out_dir / "p_minus1_stage2_results.json"
    md_path = out_dir / "P_MINUS1_STAGE2_REPORT.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_report(md_path, tuning=tuning_manifest, test=test_manifest, challenge=challenge_manifest, stage0_audit=stage0_audit, aggregates=aggregates, comparisons=comparisons, plots=summary["plots"], rep_gifs=rep_gifs)
    print("stage2_json", json_path)
    print("stage2_md", md_path)
    for path in summary["plots"].values():
        print("stage2_plot", path)
    for path in rep_gifs:
        print("stage2_gif", path)


if __name__ == "__main__":
    main()
