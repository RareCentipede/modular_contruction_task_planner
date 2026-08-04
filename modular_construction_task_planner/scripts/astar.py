import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
from typing import List, Optional, Tuple
from mpnp_interfaces.msg import Block
from scripts.mapping import OCCUPANCY, partition_block_space

class GridGraph:
    def __init__(self, blocks: List[Block], block_size: float, margin: float = 0.1, connectivity: int = 8):
        self.block_size = block_size
        self.margin = margin
        self.connectivity = connectivity  # 4 or 8

        # Build grid and occupancy state
        self.grid = partition_block_space(blocks, block_size)
        self.grid_tree = KDTree(self.grid)
        self.occupancy = np.full(len(self.grid), OCCUPANCY.FREE)

        for block in blocks:
            self._mark_occupied([block.init_pose.pose.position.x,
                                  block.init_pose.pose.position.y])

        self.G = self._build_graph()

    # --- Graph construction ---

    def _build_graph(self) -> nx.Graph:
        G = nx.Graph()
        free_mask = self.occupancy == OCCUPANCY.FREE
        free_indices = np.where(free_mask)[0]

        # Add all free nodes (store world position as attribute)
        for i in free_indices:
            G.add_node(i, pos=self.grid[i])

        # Connect neighbours
        for i in free_indices:
            self._add_edges_for_node(G, i)

        return G

    def _neighbours(self, idx: int) -> List[int]:
        """Return grid indices of cells adjacent to idx (4- or 8-connected)."""
        pos = self.grid[idx]
        _, candidates = self.grid_tree.query(pos, k=self.connectivity+1)
        candidates = np.atleast_1d(candidates)
        return [int(c) for c in candidates if c != idx]

    def _add_edges_for_node(self, G: nx.Graph, idx: int, dest: bool = False):
        """Add weighted edges from idx to all free neighbours."""
        for nb in self._neighbours(idx):
            if G.has_node(nb):  # neighbour is free
                dist = np.linalg.norm(self.grid[idx] - self.grid[nb])
                G.add_edge(idx, nb, weight=dist)
                if dest:
                    G.add_edge(nb, idx, weight=dist)  # Ensure bidirectional for destination node

    # --- Occupancy helpers ---

    def _affected_indices(self, pos: List[float]) -> List[int]:
        return self.grid_tree.query_ball_point(pos, self.block_size / 2 + self.margin, p=2)

    def _mark_occupied(self, pos: List[float]):
        self.occupancy[self._affected_indices(pos)] = OCCUPANCY.OCCUPIED

    def _mark_free(self, pos: List[float]):
        self.occupancy[self._affected_indices(pos)] = OCCUPANCY.FREE

    # --- Efficient update when a block moves ---

    def update_block_move(self, pos: List[float], oc: OCCUPANCY):
        """O(k log k) update — only touches cells near old and new positions."""

        if oc == OCCUPANCY.FREE:
            # 1. Free up old region: add nodes, then stitch edges to free neighbours
            self._mark_free(pos)
            for idx in self._affected_indices(pos):
                if not self.G.has_node(idx):
                    self.G.add_node(idx, pos=self.grid[idx])
                    self._add_edges_for_node(self.G, idx)
        elif oc == OCCUPANCY.OCCUPIED:
            # 2. Occupy new region: remove nodes (nx removes incident edges automatically)
            self._mark_occupied(pos)
            for idx in self._affected_indices(pos):
                if self.G.has_node(idx):
                    self.G.remove_node(idx)
        else:
            raise ValueError(f"Invalid occupancy state: {oc}")

    # --- Path query ---

    def nearest_free_node(self, world_pos: List[float]) -> int:
        """Snap a world position to the nearest free grid node."""
        for idx in np.atleast_1d(self.grid_tree.query(world_pos, k=20)[1]):
            if self.G.has_node(idx):
                return idx
        raise ValueError(f"No free node found near {world_pos}")

    def heuristic(self, u: int, v: int) -> float:
        """Euclidean distance between nodes u and v."""
        return np.linalg.norm(self.grid[u] - self.grid[v]).item()

    def plan(self, start_world: List[float], goal_world: List[float]) -> np.ndarray:
        """Return list of world-space waypoints, or [] if no path exists."""
        self.grid = np.vstack((self.grid, start_world, goal_world))  # Add start and goal to grid for path extraction
        start = len(self.grid) - 2  # Start is second to last node
        goal  = len(self.grid) - 1  # Goal is last node

        self._add_edges_for_node(self.G, start)  # Connect start
        self._add_edges_for_node(self.G, goal, dest=True)  # Connect goal

        try:
            node_path = nx.astar_path(self.G, start, goal,
                                       heuristic=self.heuristic, weight='weight')
            path = np.array([self.grid[n] for n in node_path])
            path = np.vstack((start_world, path, goal_world))  # Ensure start and goal are included
        except nx.NetworkXNoPath:
            path = np.array([])  # No path found

        # Clean up: remove start and goal from graph and grid
        self.G.remove_node(start)
        self.G.remove_node(goal)
        self.grid = self.grid[:-2]  # Remove start and goal from grid

        return path

    def plot_grid_and_path(self, path: np.ndarray, start: List[float], goal: List[float]):
        free_mask = self.occupancy == OCCUPANCY.FREE
        fig, ax = plt.subplots(figsize=(8, 8))
        plt.scatter(self.grid[free_mask][:, 0], self.grid[free_mask][:, 1], c='lightgray', s=10)
        plt.scatter(self.grid[~free_mask][:, 0], self.grid[~free_mask][:, 1], c='red', s=10)
        plt.scatter(start[0], start[1], c='green', s=100, marker='^') # type: ignore
        plt.scatter(goal[0], goal[1], c='orange', s=100, marker='*') # type: ignore

        if path.size > 0:
            plt.plot(path[:, 0], path[:, 1], c='blue', linewidth=2)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Grid Graph with Planned Path')
        plt.axis('equal')