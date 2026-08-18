import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.block_domain import Object

def plot_com_projection(problem_name: str = "interlocking_pyramid", problem_config_path: str = "configs/problem_configs/"):
    world = parse_configs_to_world(problem_name, problem_config_path)
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))

    ground_points = []
    total_mass = 0.0
    weighted_com = np.zeros(2)

    for name, data in goal_config.items():
        if 'position' not in data:
            continue
        
        pos = np.array(data['position'])
        size = np.array(data['size'])
        mass = data.get('mass', 1.0)

        # 2D COM accumulation (X, Y)
        weighted_com += pos[:2] * mass
        total_mass += mass

        # Ground contact footprint (Z close to base floor)
        if abs(pos[2] - size[2] / 2.0) < 1e-2:
            hx, hy = size[0] / 2.0, size[1] / 2.0
            ground_points.extend([
                [pos[0] - hx, pos[1] - hy],
                [pos[0] + hx, pos[1] - hy],
                [pos[0] + hx, pos[1] + hy],
                [pos[0] - hx, pos[1] + hy],
            ])

    global_com = weighted_com / total_mass
    ground_pts = np.array(ground_points)

    plt.figure(figsize=(7, 7))
    plt.title(f"Center of Mass Projection vs Ground Hull\n({problem_name})", fontsize=13, fontweight='bold')

    # Draw support base convex hull
    if len(ground_pts) >= 3:
        hull = ConvexHull(ground_pts)
        for simplex in hull.simplices:
            plt.plot(ground_pts[simplex, 0], ground_pts[simplex, 1], 'k-', linewidth=2.0)
        plt.fill(ground_pts[hull.vertices, 0], ground_pts[hull.vertices, 1], color='lightgray', alpha=0.5, label='Base Support Polygon')

    # Plot individual block footprints
    for name, data in goal_config.items():
        if 'position' in data:
            pos, sz = data['position'], data['size']
            plt.scatter(pos[0], pos[1], c='blue', s=30, alpha=0.6)
            plt.text(pos[0] + 0.05, pos[1] + 0.05, name, fontsize=8)

    # Plot Global CoM
    plt.scatter(global_com[0], global_com[1], c='red', s=180, marker='X', zorder=10, label=f'Global CoM ({global_com[0]:.2f}, {global_com[1]:.2f})')

    plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.xlabel('X Coordinate [m]')
    plt.ylabel('Y Coordinate [m]')
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_com_projection("scaffolding_tower")