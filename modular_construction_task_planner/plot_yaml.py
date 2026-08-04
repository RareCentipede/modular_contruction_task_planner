import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

def load_yaml_config(filepath: str) -> dict:
    """Safely loads a YAML configuration file."""
    if not os.path.exists(filepath):
        print(f"Warning: File not found at {filepath}")
        return {}
    with open(filepath, 'r') as f:
        return yaml.safe_load(f) or {}

def plot_joint_2d_layout(init_path: str, goal_path: str, workspace_limit: float = 6.0, problem_name: str = ''):
    """Plots initial (-Y) and goal (+Y) configurations together on a single 2D top-down map."""
    init_data = load_yaml_config(init_path)
    goal_data = load_yaml_config(goal_path)

    plt.rcParams.update({'font.size': 12, 'axes.labelsize': 14, 'axes.titlesize': 14, 'xtick.labelsize': 12, 'ytick.labelsize': 12})
    fig, ax = plt.subplots(figsize=(6, 6))

    # Configure axes limits and aspect ratio
    ax.set_xlim(-workspace_limit, workspace_limit)
    ax.set_ylim(-workspace_limit, workspace_limit)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f"{problem_name}")
    ax.grid(True, linestyle=':', alpha=0.6)

    # Draw the Y=0 separation line dividing the zones
    ax.axhline(0, color='crimson', linestyle='--', linewidth=2.0, label='Y=0 Workspace Boundary')

    # Color fillings to distinguish zones clearly in print
    ax.fill_between([-workspace_limit, workspace_limit], 0, workspace_limit, color='red', alpha=0.02)
    ax.fill_between([-workspace_limit, workspace_limit], -workspace_limit, 0, color='blue', alpha=0.02)

    # Dictionaries to track stacked labels at identical (X, Y) footprints
    init_stacks = defaultdict(list)
    goal_stacks = defaultdict(list)

    # --- 1. Process and Plot Initial Blocks (Blue) ---
    for name, data in init_data.items():
        if name == 'robot' or 'position' not in data:
            continue
        pos = data['position']
        sz = data['size']

        # Draw 2D footprint rectangle box
        # xy anchor in Matplotlib Rectangle is the bottom-left corner
        rect = plt.Rectangle((pos[0] - sz[0]/2, pos[1] - sz[1]/2), sz[0], sz[1], #type: ignore
                             facecolor='#3498db', edgecolor='#1a4e72', alpha=0.5, linewidth=1.5)
        ax.add_patch(rect)

        # Track for label composition (round coordinates to handle floating-point inaccuracies)
        coord_key = (round(pos[0], 2), round(pos[1], 2))
        init_stacks[coord_key].append(name.replace("block", "b"))

    # --- 2. Process and Plot Goal Blocks (Red) ---
    for name, data in goal_data.items():
        if name == 'robot' or 'position' not in data:
            continue
        pos = data['position']
        sz = data['size']

        rect = plt.Rectangle((pos[0] - sz[0]/2, pos[1] - sz[1]/2), sz[0], sz[1], #type: ignore
                             facecolor='#e74c3c', edgecolor='#781e15', alpha=0.5, linewidth=1.5)
        ax.add_patch(rect)

        coord_key = (round(pos[0], 2), round(pos[1], 2))
        goal_stacks[coord_key].append(name.replace("block", "b"))

    # --- 3. Place Stacked Annotation Text Labels ---
    for coord, blocks in init_stacks.items():
        # Combine labels vertically if stacked (e.g., "b1\nb2\nb3")
        label_text = "\n".join(sorted(blocks, key=lambda x: int(x[1:])))
        ax.text(coord[0], coord[1], label_text, ha='center', va='center', 
                fontsize=8, fontweight='bold', color='#1a4e72',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'))

    for coord, blocks in goal_stacks.items():
        label_text = "\n".join(sorted(blocks, key=lambda x: int(x[1:])))
        ax.text(coord[0], coord[1], label_text, ha='center', va='center', 
                fontsize=8, fontweight='bold', color='#781e15',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'))

    # Build an clean explicit visual legend
    legend_elements = [
        Line2D([0], [0], color='crimson', linestyle='--', linewidth=2, label='Y=0 Boundary'),
        Patch(facecolor='#3498db', edgecolor='#1a4e72', alpha=0.6, label='Blocks (-Y)'),
        Patch(facecolor='#e74c3c', edgecolor='#781e15', alpha=0.6, label='Goals (+Y)')
    ]

    ax.legend(handles=legend_elements, loc='upper right', framealpha=0.95)

    plt.tight_layout()

    # Save crisp vector layout asset for LaTeX
    output_filename = "joint_workspace_2d_layout.pdf"
    print(f"2D joint plot successfully saved as a vector graphic: {output_filename}")

    plt.savefig(f"new_plots/structured_scenarios/structured_scene_{problem_name}.pdf", bbox_inches='tight', dpi=300)
    # plt.show()

if __name__ == "__main__":
    config_path = "src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/configs/"
    # problems = ['box', 'triangle', 'circle']
    problems = ['house']
    limits = [12.0, 6.0, 12.0]
    for problem, limit in zip(problems, limits):
        plot_joint_2d_layout(
            init_path=f"{config_path}/{problem}/init.yaml",
            goal_path=f"{config_path}/{problem}/goal.yaml",
            workspace_limit=limit,
            problem_name=problem
        )