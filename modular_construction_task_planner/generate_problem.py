"""
    generate_problem.py
    -------------------
    Generate random stacking structure problems as init.yaml + goal.yaml files.

    Algorithm:
    1. Start with a ground surface (infinite XY plane at z=0)
    2. For each block (in placement order):
        a. Randomly choose a surface to place it on (ground or top face of a placed block)
        b. Randomly sample an (x, y) position within the bounds of that surface,
            shrunk by half the new block's footprint so it doesn't hang off completely
        c. Compute the z position from the surface height + half the new block's height
        d. Check that the new block does not overlap any already-placed block (AABB check)
        e. If placement fails after max_attempts, try a different surface
    3. Record the goal config from the placed positions
    4. Scatter the same blocks at random positions away from the goal area for init config
    5. Write both to YAML

    Usage:
    python generate_problem.py --name my_problem --blocks 8 --size 0.4 --seed 42
    python generate_problem.py --name tower --blocks 12 --min-size 0.3 --max-size 0.8 --seed 7
"""

from __future__ import annotations

import argparse
import os
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any

import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Block:
    name: str
    size: Tuple[float, float, float]         # (lx, ly, lz)
    position: Tuple[float, float, float]     # center (x, y, z)
    orientation: Tuple[float, float, float]  # euler (rx, ry, rz) — always 0 for now
    color: Tuple[float, float, float]        # RGB in [0, 1]

    @property
    def min_xyz(self) -> np.ndarray:
        p = np.array(self.position)
        h = np.array(self.size) / 2
        return p - h

    @property
    def max_xyz(self) -> np.ndarray:
        p = np.array(self.position)
        h = np.array(self.size) / 2
        return p + h

    @property
    def top_surface_z(self) -> float:
        return self.position[2] + self.size[2] / 2

    @property
    def top_surface_bounds(self) -> Tuple[float, float, float, float]:
        """(x_min, x_max, y_min, y_max) of the top face."""
        cx, cy, _ = self.position
        hx, hy = self.size[0] / 2, self.size[1] / 2
        return cx - hx, cx + hx, cy - hy, cy + hy

@dataclass
class GroundSurface:
    """Pseudo-surface representing the infinite ground plane at z=0."""
    bounds: Tuple[float, float, float, float] = (-5.0, 5.0, -5.0, 5.0)  # (xmin,xmax,ymin,ymax)
    z: float = 0.0

# ---------------------------------------------------------------------------
# Collision check
# ---------------------------------------------------------------------------

def aabb_overlap(a: Block, b: Block, margin: float = 1e-3) -> bool:
    """True if two blocks overlap (with a small margin to allow touching)."""
    a_min, a_max = a.min_xyz, a.max_xyz
    b_min, b_max = b.min_xyz, b.max_xyz
    for i in range(3):
        if a_max[i] <= b_min[i] + margin or b_max[i] <= a_min[i] + margin:
            return False
    return True

def collides_with_any(candidate: Block, placed: List[Block]) -> bool:
    return any(aabb_overlap(candidate, p) for p in placed)

# ---------------------------------------------------------------------------
# Random colour palette
# ---------------------------------------------------------------------------

PALETTE = [
    (0.4, 0.4, 0.4),
    (0.2, 0.6, 0.4),
    (0.2, 0.4, 0.8),
    (0.6, 0.3, 0.7),
    (0.8, 0.4, 0.2),
    (0.3, 0.7, 0.7),
    (0.7, 0.2, 0.4),
    (0.5, 0.6, 0.2),
    (0.9, 0.7, 0.1),
    (0.1, 0.5, 0.9),
]

# ---------------------------------------------------------------------------
# Core generator
# ---------------------------------------------------------------------------

def sample_position_on_ground(
    placed: List[Block],
    block_size: Tuple[float, float, float],
    rng: random.Random,
    ground_bounds: Tuple[float, float, float, float],
    gap: float = 0.05,
    max_attempts: int = 50,
) -> Optional[Tuple[float, float]]:
    """
    If no blocks are on the ground yet, place randomly within ground_bounds.
    Otherwise, place adjacent to a randomly chosen ground-level block,
    on one of its four sides (±X or ±Y).
    """
    ground_blocks = [b for b in placed if _is_on_ground(b)]

    if not ground_blocks:
        # Empty ground — place anywhere
        return sample_position_on_surface_with_overhang(ground_bounds, block_size, rng)

    lx, ly, lz = block_size

    for _ in range(max_attempts):
        neighbour = rng.choice(ground_blocks)
        side = rng.choice(['x+', 'x-', 'y+', 'y-'])

        nx, ny, _ = neighbour.position
        nhx, nhy = neighbour.size[0] / 2, neighbour.size[1] / 2

        if side == 'x+':
            x = nx + nhx + lx / 2 + gap
            y = ny + rng.uniform(-nhy + ly / 2, nhy - ly / 2) if nhy > ly / 2 else ny
        elif side == 'x-':
            x = nx - nhx - lx / 2 - gap
            y = ny + rng.uniform(-nhy + ly / 2, nhy - ly / 2) if nhy > ly / 2 else ny
        elif side == 'y+':
            x = nx + rng.uniform(-nhx + lx / 2, nhx - lx / 2) if nhx > lx / 2 else nx
            y = ny + nhy + ly / 2 + gap
        else:  # y-
            x = nx + rng.uniform(-nhx + lx / 2, nhx - lx / 2) if nhx > lx / 2 else nx
            y = ny - nhy - ly / 2 - gap

        # Check within ground bounds
        xmin, xmax, ymin, ymax = ground_bounds
        if not (xmin + lx/2 <= x <= xmax - lx/2 and ymin + ly/2 <= y <= ymax - ly/2):
            continue

        return round(x, 4), round(y, 4)

    # Fallback — random position if adjacency repeatedly fails
    return sample_position_on_surface_with_overhang(ground_bounds, block_size, rng)

def _is_on_ground(block: Block, ground_z: float = 0.0, tol: float = 1e-3) -> bool:
    """True if the block's bottom face is resting on the ground."""
    return abs(block.position[2] - block.size[2] / 2 - ground_z) < tol

def sample_position_on_surface_with_overhang(
    surface_bounds: Tuple[float, float, float, float],
    block_size: Tuple[float, float, float],
    rng: random.Random,
    overhang_factor: float = 0.4,
) -> Tuple[float, float]:
    """
        Sample a random (x, y) center for a block on a surface, allowing the block
        to hang over the edge by up to overhang_factor * block_half_extent.
        overhang_factor=0.0 means fully on surface, 1.0 means half the block is off.
    """
    xmin, xmax, ymin, ymax = surface_bounds
    hx, hy = block_size[0] / 2, block_size[1] / 2

    # Allow the block center to be closer to the edge than its half-extent
    # by reducing the inset — negative inset means the center can go past the edge
    inset_x = hx * (1.0 - overhang_factor)
    inset_y = hy * (1.0 - overhang_factor)

    sx_min, sx_max = xmin + inset_x, xmax - inset_x
    sy_min, sy_max = ymin + inset_y, ymax - inset_y

    # If the surface is very small relative to the block, center on it
    if sx_min >= sx_max:
        sx_min = sx_max = (xmin + xmax) / 2
    if sy_min >= sy_max:
        sy_min = sy_max = (ymin + ymax) / 2

    x = rng.uniform(sx_min, sx_max)
    y = rng.uniform(sy_min, sy_max)
    return round(x, 4), round(y, 4)

def generate_goal_structure(
    n_blocks: int,
    block_sizes: List[Tuple[float, float, float]],
    rng: random.Random,
    ground_bounds: Tuple[float, float, float, float] = (-4.0, 4.0, -4.0, 4.0),
    ground_ratio: float = 1/3,
    overhang_factor: float = 0.4,
    max_attempts_per_block: int = 100,
    max_attempts_per_surface: int = 20,
) -> List[Block]:
    """
        Place blocks with ground_ratio of them on the ground, the rest stacked.
        Stacked blocks can be larger than the one below and placed off-center.
    """
    ground = GroundSurface(bounds=ground_bounds)
    placed: List[Block] = []

    n_ground = max(1, round(n_blocks * ground_ratio))

    for i in range(n_blocks):
        size = block_sizes[i]
        name = f"block{i + 1}"
        color = PALETTE[i % len(PALETTE)]
        placed_successfully = False

        # Decide whether this block must go on the ground
        n_placed_on_ground = sum(1 for b in placed if _is_on_ground(b))
        n_remaining = n_blocks - i
        must_be_ground = n_placed_on_ground < n_ground
        must_be_stacked = (n_placed_on_ground >= n_ground) and bool(placed)

        for _ in range(max_attempts_per_block):
            # Build surface candidate list respecting ground ratio
            if must_be_ground or not placed:
                surfaces = [ground]
            elif must_be_stacked:
                surfaces = [b for b in placed]
            else:
                surfaces = [ground] + placed

            surface = rng.choice(surfaces)

            if isinstance(surface, GroundSurface):
                xy = sample_position_on_ground(placed, size, rng, ground_bounds)
                if xy is None:
                    continue
                x, y = xy
                z_center = surface.z + size[2] / 2
            else:
                bounds = surface.top_surface_bounds
                x, y = sample_position_on_surface_with_overhang(bounds, size, rng, overhang_factor)
                z_center = surface.top_surface_z + size[2] / 2

            candidate = Block(
                name=name,
                size=size,
                position=(x, y, round(z_center, 4)),
                orientation=(0.0, 0.0, 0.0),
                color=color,
            )

            if not collides_with_any(candidate, placed):
                placed.append(candidate)
                placed_successfully = True
                break

        if not placed_successfully:
            print(f"  Warning: could not place {name} — skipping.")

    return placed

def generate_init_positions(
    blocks: List[Block],
    rng: random.Random,
    staging_origin: Tuple[float, float] = (-12.0, -6.0),
    gap: float = 0.0,
    max_row_width: float = 12.0,
) -> List[Block]:
    """
        Place all blocks in a compact row layout away from the goal area,
        packing them side by side with a small gap. Each block rests on the ground.
        A new row starts when the current row exceeds max_row_width.
    """
    init_blocks = []
    x_cursor = staging_origin[0]
    y_cursor = staging_origin[1]
    row_height = 0.0   # tallest block footprint (y extent) in the current row

    for block in blocks:
        lx, ly, lz = block.size

        # Start a new row if this block would exceed max_row_width
        if init_blocks and (x_cursor - staging_origin[0] + lx / 2) > max_row_width:
            x_cursor = staging_origin[0]
            y_cursor += row_height + gap
            row_height = 0.0

        x = x_cursor + lx / 2
        y = y_cursor + ly / 2
        z = lz / 2   # resting on ground

        init_blocks.append(Block(
            name=block.name,
            size=block.size,
            position=(round(x, 4), round(y, 4), round(z, 4)),
            orientation=(0.0, 0.0, 0.0),
            color=block.color,
        ))

        x_cursor += lx + gap
        row_height = max(row_height, ly)

    return init_blocks

# ---------------------------------------------------------------------------
# YAML serialisation
# ---------------------------------------------------------------------------

def block_to_goal_dict(block: Block) -> Dict[str, Any]:
    return {
        'position': list(block.position),
        'orientation': list(block.orientation),
        'size': list(block.size),
    }


def block_to_init_dict(block: Block) -> Dict[str, Any]:
    return {
        'type': 'dynamic',
        'position': list(block.position),
        'orientation': list(block.orientation),
        'color': list(block.color),
        'size': list(block.size),
    }

def generate_problem(
    problem_name: str,
    n_blocks: int,
    block_size: Optional[float] = None,
    min_size: float = 0.4,
    max_size: float = 0.8,
    seed: Optional[int] = None,
    output_dir: str = '../configs/problem_configs/',
    robot_position: Tuple[float, float, float] = (-15.0, 0.0, 0.0),
) -> Tuple[Dict, Dict]:
    """
        Main entry point. Returns (init_dict, goal_dict) and writes YAML files.
    """
    rng = random.Random(seed)
    np_rng = np.random.default_rng(seed)

    # --- Block sizes ---
    if block_size is not None:
        sizes = [(block_size, block_size, block_size)] * n_blocks
    else:
        sizes = []
        for _ in range(n_blocks):
            lx = round(rng.uniform(min_size, max_size), 2)
            ly = round(rng.uniform(min_size, max_size), 2)
            lz = round(rng.uniform(min_size, max_size), 2)
            sizes.append((lx, ly, lz))

    print(f"Generating '{problem_name}': {n_blocks} blocks, seed={seed}")

    # --- Goal structure ---
    goal_blocks = generate_goal_structure(n_blocks, sizes, rng)
    print(f"  Placed {len(goal_blocks)}/{n_blocks} blocks in goal structure")

    # --- Init positions ---
    init_blocks = generate_init_positions(goal_blocks, rng)

    # --- Build dicts ---
    goal_dict: Dict[str, Any] = {}
    for block in goal_blocks:
        goal_dict[block.name] = block_to_goal_dict(block)

    init_dict: Dict[str, Any] = {}
    for block in init_blocks:
        init_dict[block.name] = block_to_init_dict(block)
    init_dict['robot'] = {
        'type': 'robot',
        'position': list(robot_position),
        'orientation': [0.0, 0.0, 0.0],
    }

    # --- Write files ---
    problem_dir = os.path.join(output_dir, problem_name)
    os.makedirs(problem_dir, exist_ok=True)

    goal_path = os.path.join(problem_dir, 'goal.yaml')
    init_path = os.path.join(problem_dir, 'init.yaml')

    with open(goal_path, 'w') as f:
        yaml.dump(goal_dict, f, default_flow_style=False, sort_keys=False)

    with open(init_path, 'w') as f:
        yaml.dump(init_dict, f, default_flow_style=False, sort_keys=False)

    print(f"  Written: {goal_path}")
    print(f"  Written: {init_path}")

    return init_dict, goal_dict

# ---------------------------------------------------------------------------
# Visualisation (optional, requires matplotlib)
# ---------------------------------------------------------------------------

def visualise_structure(goal_blocks: List[Block], title: str = 'Goal structure') -> None:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        print("matplotlib not available — skipping visualisation")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for block in goal_blocks:
        cx, cy, cz = block.position
        hx, hy, hz = [s / 2 for s in block.size]

        # 8 corners
        corners = np.array([
            [cx-hx, cy-hy, cz-hz], [cx+hx, cy-hy, cz-hz],
            [cx+hx, cy+hy, cz-hz], [cx-hx, cy+hy, cz-hz],
            [cx-hx, cy-hy, cz+hz], [cx+hx, cy-hy, cz+hz],
            [cx+hx, cy+hy, cz+hz], [cx-hx, cy+hy, cz+hz],
        ])

        faces = [
            [corners[0], corners[1], corners[2], corners[3]],  # bottom
            [corners[4], corners[5], corners[6], corners[7]],  # top
            [corners[0], corners[1], corners[5], corners[4]],  # front
            [corners[2], corners[3], corners[7], corners[6]],  # back
            [corners[0], corners[3], corners[7], corners[4]],  # left
            [corners[1], corners[2], corners[6], corners[5]],  # right
        ]

        color = block.color
        poly = Poly3DCollection(faces, alpha=0.6)
        poly.set_facecolor(color)
        poly.set_edgecolor('k')
        poly.set_linewidth(0.4)
        ax.add_collection3d(poly)

        ax.text(cx, cy, cz, block.name, fontsize=6, ha='center', va='center')

    all_pos = np.array([b.position for b in goal_blocks])
    all_size = np.array([b.size for b in goal_blocks])
    margin = 1.0
    ax.set_xlim(all_pos[:,0].min() - margin, all_pos[:,0].max() + margin)
    ax.set_ylim(all_pos[:,1].min() - margin, all_pos[:,1].max() + margin)
    ax.set_zlim(0, all_pos[:,2].max() + all_size[:,2].max() + margin)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(title)
    ax.view_init(elev=25, azim=-45)
    plt.tight_layout()
    plt.savefig(f'{title.replace(" ", "_")}.png', dpi=150)
    plt.show()
    print(f"Saved visualisation.")

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
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