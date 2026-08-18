import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict, Optional, cast
from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.block_domain import Object

def calculate_overhang_strain_step(pos: list, size: list, active_config: dict) -> float:
    """Computes strain based on unsupported area ratio for active blocks in a construction step."""
    com_x, com_y, com_z = pos[0], pos[1], pos[2]
    half_x, half_y, half_z = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    
    # Base blocks touching ground have zero strain
    if com_z <= half_z + 0.1:
        return 0.0

    supporting_blocks = []
    for other_name, other_data in active_config.items():
        if 'position' not in other_data:
            continue
        other_pos = other_data['position']
        other_size = other_data['size']
        
        # Check if top of lower block aligns with bottom of current block
        bot_z = com_z - half_z
        top_other_z = other_pos[2] + other_size[2] / 2.0
        
        if abs(bot_z - top_other_z) < 0.1:
            dx = abs(com_x - other_pos[0])
            dy = abs(com_y - other_pos[1])
            if dx < (half_x + other_size[0] / 2.0) and dy < (half_y + other_size[1]/2.0):
                supporting_blocks.append((other_pos, other_size))

    if not supporting_blocks:
        return 1.0  # Floating / Unsupported block

    total_overlap_area = 0.0
    block_area = size[0] * size[1]

    for s_pos, s_size in supporting_blocks:
        x_overlap = max(0.0, min(com_x + half_x, s_pos[0] + s_size[0]/2.0) - max(com_x - half_x, s_pos[0] - s_size[0]/2.0))
        y_overlap = max(0.0, min(com_y + half_y, s_pos[1] + s_size[1]/2.0) - max(com_y - half_y, s_pos[1] - s_size[1]/2.0))
        total_overlap_area += (x_overlap * y_overlap)

    unsupported_ratio = 1.0 - min(1.0, total_overlap_area / block_area)
    return float(np.clip(unsupported_ratio, 0.0, 1.0))

def plot_construction_sequence_strain(
    problem_name: str = "scaffolding_tower", 
    problem_config_path: str = "configs/problem_configs/",
    construction_sequence: Optional[List[str]] = None
):
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    
    # If no custom sequence provided, default to sorted block key order
    if construction_sequence is None:
        construction_sequence = [k for k in goal_config.keys() if 'position' in goal_config[k]]
        # Sort sequence by Z height so base blocks are placed first
        construction_sequence.sort(key=lambda k: goal_config[k]['position'][2])

    steps = []
    max_strains = []
    mean_strains = []
    latest_strains = []
    active_config = {}

    # Iterate step-by-step through the construction sequence
    for idx, block_name in enumerate(construction_sequence, start=1):
        if block_name in goal_config:
            active_config[block_name] = goal_config[block_name]
        
        # Calculate strains for all blocks currently placed
        current_strains = []
        for name, data in active_config.items():
            if 'position' in data:
                s = calculate_overhang_strain_step(data['position'], data['size'], active_config)
                current_strains.append(s)

        steps.append(idx)
        max_strains.append(max(current_strains) if current_strains else 0.0)
        mean_strains.append(np.mean(current_strains) if current_strains else 0.0)
        latest_strains.append(current_strains[-1] if current_strains else 0.0)

    # Plot Line Chart
    plt.figure(figsize=(9, 5))
    plt.plot(steps, max_strains, color='#d95f02', marker='o', linewidth=2.5, label='Peak Overhang Strain Ratio')
    plt.plot(steps, mean_strains, color='#7570b3', linestyle='--', marker='s', linewidth=1.5, label='Average Structure Strain Ratio')
    plt.plot(steps, latest_strains, color='#1b9e77', linestyle=':', marker='^', linewidth=1.5, label='Latest Block Strain Ratio')

    # Critical threshold indicator line
    plt.axhline(y=0.7, color='r', linestyle=':', label='High Strain Warning Threshold (0.7)')

    plt.title(f"Construction Sequence Strain Evolution: {problem_name}", fontsize=13, fontweight='bold', pad=12)
    plt.xlabel("Construction Step (Block Placed)", fontweight='bold')
    plt.ylabel("Strain Ratio [0.0 = Stable, 1.0 = Max Moment]", fontweight='bold')
    plt.xticks(steps, [f"Step {s}\n({construction_sequence[s-1]})" for s in steps], rotation=30, ha='right', fontsize=9)
    plt.ylim(-0.05, 1.05)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper left')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_construction_sequence_strain("cantilever_bridge")