"""
    test_stability.py
    -----------------
    Test scenarios for compute_placement_stability and get_contact_polygon.

    Scenarios:
    1. Perfect stack        — box perfectly centered on box below   → stable (ratio ~1.0)
    2. Half overhang        — box shifted 50% over edge             → marginal (~0.5)
    3. Full overhang        — box completely off the supporting box → unstable (0.0)
    4. Ground contact       — box resting directly on ground        → always stable
    5. Two supporters       — box supported by two boxes side by side
    6. Offset two supports  — one supporter carries more than other

    Each scenario prints the support_data and ratio, and renders a 3D matplotlib figure.
"""

import numpy as np
import trimesh
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon
from typing import Optional, Dict, Tuple, List
from modular_construction_task_planner.stability import compute_placement_stability, get_contact_polygon

# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------

def make_box(size, position) -> trimesh.Trimesh:
    """Create a box mesh centered at position."""
    mesh = trimesh.creation.box(extents=size)
    mesh.apply_translation([
        position[0],
        position[1],
        position[2] + size[2] / 2   # position is bottom-center
    ])
    return mesh

def make_ground(extent=5.0) -> trimesh.Trimesh:
    verts = np.array([[-extent, -extent, 0],
                      [ extent, -extent, 0],
                      [ extent,  extent, 0],
                      [-extent,  extent, 0]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    return trimesh.Trimesh(vertices=verts, faces=faces)

# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

COLORS = {
    'below':   (0.3, 0.6, 0.9, 0.5),   # blue
    'above':   (0.9, 0.4, 0.3, 0.5),   # red
    'ground':  (0.7, 0.7, 0.7, 0.3),   # grey
    'contact': (0.2, 0.8, 0.2, 0.9),   # green
    'footprint':(1.0, 0.8, 0.0, 0.4),  # yellow
}

def trimesh_to_poly3d(mesh: trimesh.Trimesh, color):
    tris = mesh.vertices[mesh.faces]
    poly = Poly3DCollection(tris, alpha=color[3])
    poly.set_facecolor(color[:3])
    poly.set_edgecolor('k')
    poly.set_linewidth(0.3)
    return poly

def draw_polygon_xy(ax, polygon: Polygon, z: float, color, label=None):
    if polygon.is_empty:
        return
    xs, ys = polygon.exterior.xy
    zs = [z] * len(xs)
    ax.plot(xs, ys, zs, color=color[:3], linewidth=2, label=label)
    verts = [list(zip(xs, ys, zs))]
    poly3d = Poly3DCollection(verts, alpha=color[3])
    poly3d.set_facecolor(color[:3])
    ax.add_collection3d(poly3d)

def visualize_scenario(title: str,
                       mesh_above: trimesh.Trimesh,
                       meshes_below: List[trimesh.Trimesh],
                       names_below: List[str],
                       support_data: Dict[str, float],
                       ratio: float,
                       ax):
    ax.set_title(f"{title}\nstability ratio: {ratio:.2f}", fontsize=9)

    # Draw ground lightly
    ground = make_ground(2.0)
    ax.add_collection3d(trimesh_to_poly3d(ground, COLORS['ground']))

    # Draw supporting meshes
    for mesh in meshes_below:
        ax.add_collection3d(trimesh_to_poly3d(mesh, COLORS['below']))

    # Draw the object being placed
    ax.add_collection3d(trimesh_to_poly3d(mesh_above, COLORS['above']))

    # Draw footprint of upper object at its bottom Z
    footprint = Polygon(mesh_above.vertices[:, :2])
    bottom_z = mesh_above.bounds[0][2]
    draw_polygon_xy(ax, footprint, bottom_z, COLORS['footprint'], 'footprint')

    # Draw contact polygons
    for mesh_below, name in zip(meshes_below, names_below):
        contact = get_contact_polygon(mesh_above, mesh_below)
        if contact:
            draw_polygon_xy(ax, contact, bottom_z + 0.02, COLORS['contact'], # type: ignore
                            f'contact ({name})')

    # Axis formatting
    all_verts = np.vstack([mesh_above.vertices] +
                          [m.vertices for m in meshes_below])
    margin = 0.3
    ax.set_xlim(all_verts[:,0].min()-margin, all_verts[:,0].max()+margin)
    ax.set_ylim(all_verts[:,1].min()-margin, all_verts[:,1].max()+margin)
    ax.set_zlim(0, all_verts[:,2].max()+margin)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.view_init(elev=25, azim=-45)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def run_scenarios():
    ground = make_ground()
    box_size = [0.4, 0.4, 0.4]   # all blocks same size

    scenarios = []

    # 1. Perfect stack
    b1 = make_box(box_size, [0, 0, 0])
    b2 = make_box(box_size, [0, 0, 0.4])
    scenarios.append(("1. Perfect stack", b2, [b1], ['block_A']))

    # 2. Half overhang (50% offset in X)
    b3 = make_box(box_size, [0, 0, 0])
    b4 = make_box(box_size, [0.2, 0, 0.4])
    scenarios.append(("2. Half overhang (50%)", b4, [b3], ['block_B']))

    # 3. Full overhang (completely off)
    b5 = make_box(box_size, [0, 0, 0])
    b6 = make_box(box_size, [0.5, 0, 0.4])
    scenarios.append(("3. Full overhang (unstable)", b6, [b5], ['block_C']))

    # 4. Ground contact
    b7 = make_box(box_size, [0, 0, 0])
    scenarios.append(("4. Ground contact", b7, [], []))

    # 5. Two supporters, equal
    b8  = make_box(box_size, [-0.2, 0, 0])
    b9  = make_box(box_size, [ 0.2, 0, 0])
    b10 = make_box([0.8, 0.4, 0.4], [0, 0, 0.4])  # wide block on top
    scenarios.append(("5. Two equal supporters", b10, [b8, b9], ['left', 'right']))

    # 6. Two supporters, unequal (top block offset toward one)
    b11 = make_box(box_size, [-0.2, 0, 0])
    b12 = make_box(box_size, [ 0.2, 0, 0])
    b13 = make_box(box_size, [-0.1, 0, 0.4])   # shifted left → b11 carries more
    scenarios.append(("6. Two supporters, unequal", b13, [b11, b12], ['heavy', 'light']))

    # ---------------------------------------------------------------------------
    # Run and plot
    # ---------------------------------------------------------------------------
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Stability Analysis Test Scenarios", fontsize=13, fontweight='bold')

    for idx, (title, mesh_above, meshes_below, names_below) in enumerate(scenarios):
        ax = fig.add_subplot(2, 3, idx + 1, projection='3d')
        sd, ratio = compute_placement_stability(mesh_above, meshes_below, names_below, ground)

        print(f"\n{'='*50}")
        print(f"Scenario: {title}")
        print(f"  support_data:  {sd}")
        print(f"  overall ratio: {ratio:.3f}")
        stable = ratio >= 0.7
        print(f"  stable:        {stable}")

        visualize_scenario(title, mesh_above, meshes_below, names_below, sd, ratio, ax)

    # Legend
    legend_patches = [
        mpatches.Patch(color=COLORS['above'][:3],    alpha=0.6, label='Object being placed'),
        mpatches.Patch(color=COLORS['below'][:3],    alpha=0.6, label='Supporting object'),
        mpatches.Patch(color=COLORS['footprint'][:3],alpha=0.6, label='Footprint'),
        mpatches.Patch(color=COLORS['contact'][:3],  alpha=0.9, label='Contact polygon'),
    ]
    fig.legend(handles=legend_patches, loc='lower center', ncol=4, fontsize=9)
    plt.tight_layout(rect=[0, 0.05, 1, 1]) # type: ignore
    plt.savefig('plots/stability_test.png', dpi=150, bbox_inches='tight')
    print("\nFigure saved.")
    plt.show()

if __name__ == '__main__':
    run_scenarios()