"""
test_random_structs.py
----------------------
Benchmark three planner heuristics (AO, FB, STABLE) across randomly generated
stacking problems of varying sizes.

For each block count:
  - Generate `n_trials` random structures (each with a unique seed)
  - Run all three heuristics on each structure
  - Record time taken and whether a plan was found
  - Report average time and success rate per heuristic per block count

Usage:
  python test_random_structs.py --blocks 4 6 8 10 --trials 5 --seed 0
  python test_random_structs.py --blocks 4 8 12 --trials 10 --visualise
"""

import argparse
import time
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum

from test_construction_pipeline import run_construction_testing_pipeline, HEURISTIC
from modular_construction_task_planner.generate_problem import (
    PALETTE, Block, generate_problem, visualise_structure
)

# ---------------------------------------------------------------------------
# Result containers
# ---------------------------------------------------------------------------

@dataclass
class TrialResult:
    heuristic: HEURISTIC
    n_blocks: int
    seed: int
    plan_found: bool
    time_taken: float          # seconds; set to None if not found and excluded

@dataclass
class HeuristicStats:
    heuristic: HEURISTIC
    n_blocks: int
    n_trials: int
    n_success: int
    times: List[float] = field(default_factory=list)   # only successful trials

    @property
    def success_rate(self) -> float:
        return self.n_success / self.n_trials if self.n_trials > 0 else 0.0

    @property
    def avg_time(self) -> Optional[float]:
        return statistics.mean(self.times) if self.times else None

    @property
    def std_time(self) -> Optional[float]:
        return statistics.stdev(self.times) if len(self.times) > 1 else 0.0

    def row(self) -> str:
        avg = f"{self.avg_time:.3f}s" if self.avg_time is not None else "  N/A  "
        std = f"±{self.std_time:.3f}" if self.std_time is not None else "      "
        return (f"  {self.heuristic.name:<8} | "
                f"success: {self.n_success}/{self.n_trials} "
                f"({self.success_rate*100:5.1f}%) | "
                f"avg time: {avg} {std}")

# ---------------------------------------------------------------------------
# Core benchmark
# ---------------------------------------------------------------------------

HEURISTICS = [HEURISTIC.AO, HEURISTIC.FB, HEURISTIC.STABLE]

def benchmark(
    block_counts: List[int],
    n_trials: int,
    base_seed: int = 0,
    block_size: Optional[float] = None,
    min_size: float = 0.4,
    max_size: float = 0.8,
    output_dir: str = './benchmark_problems',
    verbose: bool = True,
) -> Dict[Tuple[int, HEURISTIC], HeuristicStats]:
    """
    Run the full benchmark. Returns a dict keyed by (n_blocks, heuristic)
    mapping to HeuristicStats.
    """
    results: Dict[Tuple[int, HEURISTIC], HeuristicStats] = {}

    # Pre-initialise stats containers
    for n in block_counts:
        for h in HEURISTICS:
            results[(n, h)] = HeuristicStats(
                heuristic=h, n_blocks=n, n_trials=n_trials, n_success=0
            )

    total_runs = len(block_counts) * n_trials * len(HEURISTICS)
    run_idx = 0

    for n_blocks in block_counts:
        if verbose:
            print(f"\n{'='*60}")
            print(f"  Block count: {n_blocks}")
            print(f"{'='*60}")

        for trial in range(n_trials):
            seed = base_seed + n_blocks * 1000 + trial   # unique, reproducible
            problem_name = f"bench_{n_blocks}blocks_trial{trial}"

            if verbose:
                print(f"\n  Trial {trial + 1}/{n_trials}  (seed={seed})")

            # Generate problem — same structure tested across all heuristics
            init_d, goal_d = generate_problem(
                problem_name=problem_name,
                n_blocks=n_blocks,
                block_size=block_size,
                min_size=min_size,
                max_size=max_size,
                seed=seed,
                output_dir=output_dir,
            )

            # Run each heuristic on the same problem
            for heuristic in HEURISTICS:
                run_idx += 1
                if verbose:
                    print(f"    [{run_idx}/{total_runs}] "
                          f"Heuristic: {heuristic.name} ... ", end='', flush=True)

                t_start = time.perf_counter()
                plan_found, time_taken = run_construction_testing_pipeline(
                    problem_name=problem_name,
                    init_config=init_d,
                    goal_config=goal_d,
                    heuristic=heuristic,
                )
                t_elapsed = time.perf_counter() - t_start

                # Use the time reported by the pipeline if available, else wall time
                reported_time = time_taken if time_taken is not None else t_elapsed

                stat = results[(n_blocks, heuristic)]
                if plan_found:
                    stat.n_success += 1
                    stat.times.append(reported_time)

                if verbose:
                    status = "✓" if plan_found else "✗"
                    print(f"{status}  {reported_time:.3f}s")

    return results

# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_summary(
    results: Dict[Tuple[int, HEURISTIC], HeuristicStats],
    block_counts: List[int],
) -> None:
    print(f"\n{'='*60}")
    print("  BENCHMARK SUMMARY")
    print(f"{'='*60}")
    for n in block_counts:
        print(f"\n  n_blocks = {n}")
        print(f"  {'-'*54}")
        for h in HEURISTICS:
            stat = results[(n, h)]
            print(stat.row())

def plot_results(
    results: Dict[Tuple[int, HEURISTIC], HeuristicStats],
    block_counts: List[int],
    save_path: Optional[str] = None,
) -> None:
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib not available — skipping plot")
        return

    fig, (ax_time, ax_success) = plt.subplots(1, 2, figsize=(13, 5))
    colors = {'AO': '#2196F3', 'FB': '#4CAF50', 'STABLE': '#FF5722'}
    x = np.arange(len(block_counts))
    bar_w = 0.25

    for i, h in enumerate(HEURISTICS):
        avg_times = []
        std_times = []
        success_rates = []

        for n in block_counts:
            stat = results[(n, h)]
            avg_times.append(stat.avg_time if stat.avg_time is not None else 0.0)
            std_times.append(stat.std_time if stat.std_time is not None else 0.0)
            success_rates.append(stat.success_rate * 100)

        offset = (i - 1) * bar_w
        c = colors[h.name]

        ax_time.bar(x + offset, avg_times, bar_w, yerr=std_times,
                    label=h.name, color=c, alpha=0.85, capsize=4)
        ax_success.bar(x + offset, success_rates, bar_w,
                       label=h.name, color=c, alpha=0.85)

    ax_time.set_xlabel('Number of blocks')
    ax_time.set_ylabel('Average planning time (s)')
    ax_time.set_title('Planning time by heuristic')
    ax_time.set_xticks(x)
    ax_time.set_xticklabels([str(n) for n in block_counts])
    ax_time.legend()
    ax_time.grid(axis='y', alpha=0.3)

    ax_success.set_xlabel('Number of blocks')
    ax_success.set_ylabel('Success rate (%)')
    ax_success.set_title('Success rate by heuristic')
    ax_success.set_xticks(x)
    ax_success.set_xticklabels([str(n) for n in block_counts])
    ax_success.set_ylim(0, 105)
    ax_success.legend()
    ax_success.grid(axis='y', alpha=0.3)

    plt.suptitle('Planner Heuristic Benchmark', fontsize=13, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n  Plot saved to {save_path}")

    plt.show()

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Benchmark planner heuristics on random stacking problems'
    )
    parser.add_argument('--blocks',   type=int,   nargs='+', default=[4, 6, 8],
                        help='List of block counts to test (e.g. --blocks 4 6 8 10)')
    parser.add_argument('--trials',   type=int,   default=5,
                        help='Number of random trials per block count (default: 5)')
    parser.add_argument('--seed',     type=int,   default=0,
                        help='Base random seed (default: 0)')
    parser.add_argument('--size',     type=float, default=None,
                        help='Fixed block size. If omitted, sizes are random.')
    parser.add_argument('--min-size', type=float, default=0.4)
    parser.add_argument('--max-size', type=float, default=0.8)
    parser.add_argument('--output',   default='./benchmark_problems',
                        help='Directory for generated problem files')
    parser.add_argument('--plot',     action='store_true',
                        help='Show and save result plots')
    parser.add_argument('--quiet',    action='store_true',
                        help='Suppress per-trial output')
    args = parser.parse_args()

    results = benchmark(
        block_counts=args.blocks,
        n_trials=args.trials,
        base_seed=args.seed,
        block_size=args.size,
        min_size=args.min_size,
        max_size=args.max_size,
        output_dir=args.output,
        verbose=not args.quiet,
    )

    print_summary(results, args.blocks)

    if args.plot:
        plot_results(results, args.blocks,
                     save_path=f"{args.output}/benchmark_results.png")