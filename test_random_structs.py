import argparse

from test_construction_pipeline import run_construction_testing_pipeline, HEURISTIC
from modular_construction_task_planner.generate_problem import PALETTE, Block, generate_problem, visualise_structure

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate random stacking problems')
    parser.add_argument('--name',      default='problem_01',  help='Problem name (output folder)')
    parser.add_argument('--blocks',    type=int, default=8,   help='Number of blocks')
    parser.add_argument('--size',      type=float, default=None,
                        help='Fixed block size (cube). If omitted, sizes are random.')
    parser.add_argument('--min-size',  type=float, default=0.4, help='Min block dimension')
    parser.add_argument('--max-size',  type=float, default=0.8, help='Max block dimension')
    parser.add_argument('--seed',      type=int,   default=None, help='Random seed')
    parser.add_argument('--output',    default='.',            help='Output directory')
    parser.add_argument('--visualise', action='store_true',    help='Show 3D visualisation')
    args = parser.parse_args()

    init_d, goal_d = generate_problem(
        problem_name=args.name,
        n_blocks=args.blocks,
        block_size=args.size,
        min_size=args.min_size,
        max_size=args.max_size,
        seed=args.seed,
        output_dir=args.output,
    )
    run_construction_testing_pipeline(problem_name="problem_01", init_config=init_d, goal_config=goal_d)

    if args.visualise:
        # Reconstruct Block objects from goal dict for visualisation
        vis_blocks = []
        for bname, bdata in goal_d.items():
            vis_blocks.append(Block(
                name=bname,
                size=tuple(bdata['size']),
                position=tuple(bdata['position']),
                orientation=tuple(bdata['orientation']),
                color=PALETTE[len(vis_blocks) % len(PALETTE)],
            ))
        visualise_structure(vis_blocks, title=args.name)