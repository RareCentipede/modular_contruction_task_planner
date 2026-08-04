import numpy as np

from typing import List, Dict
from scipy.spatial import KDTree
from enum import Enum

from modular_construction_task_planner.eas.core import Pose
from mpnp_interfaces.msg import Block

OCCUPANCY = Enum('OCCUPANCY', 'FREE OCCUPIED')

# Make this compatible with a list of Objects instead of just blocks
def partition_block_space(blocks: List[Block] | Dict[str, Pose], block_size: float, base_dist: float = 0.7) -> np.ndarray:
    # Extend the grid by the safety margin
    if isinstance(blocks, list):
        block_init_positions = [[block.init_pose.pose.position.x, block.init_pose.pose.position.y] for block in blocks]
        block_goal_positions = [[block.goal_pose.pose.position.x, block.goal_pose.pose.position.y] for block in blocks]
        block_pos_arr = np.array(block_init_positions + block_goal_positions)
    elif isinstance(blocks, dict) and isinstance(next(iter(blocks.values())), Pose):
        block_pos_arr = np.array([pose.position[:2] for pose in blocks.values()])

    x_axis = block_pos_arr[:, 0]
    x_min = np.min(x_axis) - base_dist*2
    x_max = np.max(x_axis) + base_dist*2

    y_axis = block_pos_arr[:, 1]
    y_min = np.min(y_axis) - base_dist*2
    y_max = np.max(y_axis) + base_dist*2

    # Create a 3D grid
    x_grid = np.arange(x_min, x_max, block_size)
    y_grid = np.arange(y_min, y_max, block_size)

    grid = np.meshgrid(x_grid, y_grid)
    grid = np.array(grid).reshape(2, -1).T
    return np.array(grid)

def create_occupancy_grid(blocks: List[Block] | Dict[str, Pose], grid: np.ndarray, block_size: float, margin: float = 0.1) -> np.ndarray:
    grid = partition_block_space(blocks, block_size)
    grid_tree = KDTree(grid)

    occupancy_grid = np.full(grid.shape, OCCUPANCY.FREE)
    if blocks and isinstance(blocks, list):
        for block in blocks:
            block_pos = [block.init_pose.pose.position.x, block.init_pose.pose.position.y]
            indices = grid_tree.query_ball_point(block_pos, block_size/2 + margin, p=2)
            occupancy_grid[indices] = OCCUPANCY.OCCUPIED
    elif blocks and isinstance(blocks, Dict):
        for pose in blocks.values():
            block_pos = pose.position[:2]
            indices = grid_tree.query_ball_point(block_pos, block_size/2 + margin, p=2)
            occupancy_grid[indices] = OCCUPANCY.OCCUPIED
    return occupancy_grid

def toggle_occupancy_in_region(grid_tree: KDTree, occupancy_grid: np.ndarray, block_pos: List[float], occupancy: OCCUPANCY,
                               block_size: float, margin: float = 0.1):
    indices = grid_tree.query_ball_point(block_pos, block_size/2 + margin, p=2)
    occupancy_grid[indices] = occupancy