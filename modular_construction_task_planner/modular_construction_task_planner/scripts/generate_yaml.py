import os
import yaml
import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- CONFIGURATION ---
BLOCK_SIZE = [1.0, 1.0, 1.0]  # Width (X), Length (Y), Height (Z)
WORKSPACE_LIMIT = 12.0         # Bounds for the grid (-6 to +6)
GRID_SNAP = 0.5               # Grid snapping size for clean layout configurations

# State Tracking
placed_blocks = {}            # Format: 'block1': {'position': [x, y, z], 'size': [...]}
block_counter = 1
placement_history = []        # Track order of added blocks to enable step-by-step undo operations

# Setup a clean 2-panel figure layout
fig = plt.figure(figsize=(16, 8))
ax_2d = fig.add_subplot(121)                  # Left: Click interface
ax_3d = fig.add_subplot(122, projection='3d') # Right: 3D Visualization

def compute_box_vertices(size, pos):
    """Calculates the 8 corner vertices of a 3D box."""
    dx, dy, dz = size
    x, y, z = pos
    x0, x1 = x - dx/2, x + dx/2
    y0, y1 = y - dy/2, y + dy/2
    z0, z1 = z - dz/2, z + dz/2 # Center of mass aligned to match your template format
    
    return np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], 
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]  
    ])

def draw_block_mesh(ax, vertices, face_color, edge_color, alpha=0.7):
    """Draws a 3D rectangular box mesh."""
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]], # Bottom
        [vertices[4], vertices[5], vertices[6], vertices[7]], # Top
        [vertices[0], vertices[1], vertices[5], vertices[4]], # Front
        [vertices[2], vertices[3], vertices[7], vertices[6]], # Back
        [vertices[0], vertices[3], vertices[7], vertices[4]], # Left
        [vertices[1], vertices[2], vertices[6], vertices[5]]  # Right
    ]
    poly = Poly3DCollection(faces, alpha=alpha)
    poly.set_facecolor(face_color)
    poly.set_edgecolor(edge_color)
    poly.set_linewidth(0.8)
    ax.add_collection3d(poly)

def redraw_scene():
    """Renders both the 2D blueprint layout and the 3D isometric view."""
    # --- Part A: Redraw Left Panel (2D Blueprint) ---
    ax_2d.clear()
    ax_2d.set_xlim(-WORKSPACE_LIMIT, WORKSPACE_LIMIT)
    ax_2d.set_ylim(-WORKSPACE_LIMIT, WORKSPACE_LIMIT)
    ax_2d.set_xlabel('X Coordinate')
    ax_2d.set_ylabel('Y Coordinate')
    ax_2d.grid(True, linestyle=':', alpha=0.6)
    ax_2d.set_title("1. LEFT-CLICK HERE TO PLACE (+Y ONLY)", fontsize=11, fontweight='bold')
    
    # Boundary Line at Y=0 separating staging and construction site
    ax_2d.axhline(0, color='red', linestyle='--', linewidth=2.0, label='Y=0 Construction Border')
    ax_2d.fill_between([-WORKSPACE_LIMIT, WORKSPACE_LIMIT], 0, WORKSPACE_LIMIT, color='green', alpha=0.04)
    ax_2d.fill_between([-WORKSPACE_LIMIT, WORKSPACE_LIMIT], -WORKSPACE_LIMIT, 0, color='blue', alpha=0.03)
    
    # --- Part B: Redraw Right Panel (3D Monitor) ---
    ax_3d.clear()
    ax_3d.set_xlim(-WORKSPACE_LIMIT, WORKSPACE_LIMIT)
    ax_3d.set_ylim(-WORKSPACE_LIMIT, WORKSPACE_LIMIT)
    ax_3d.set_zlim(0, WORKSPACE_LIMIT)
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title("2. 3D STRUCTURAL PREVIEW\n(Press 'u' to undo last block | 's' to save YAML)", fontsize=11, fontweight='bold')
    
    # Ground grid on the 3D window
    gx, gy = np.meshgrid(np.linspace(-WORKSPACE_LIMIT, WORKSPACE_LIMIT, 13), np.linspace(-WORKSPACE_LIMIT, WORKSPACE_LIMIT, 13))
    ax_3d.plot_wireframe(gx, gy, np.zeros_like(gx), color=(0.7, 0.7, 0.7, 0.15), linewidth=0.8)
    ax_3d.plot([-WORKSPACE_LIMIT, WORKSPACE_LIMIT], [0, 0], [0, 0], color='red', linestyle='--', linewidth=2.0)

    # Render data states onto both layouts synchronously
    for name, data in placed_blocks.items():
        pos = data['position']
        sz = data['size']
        
        # Draw 2D footprint bounding box
        rect = plt.Rectangle((pos[0] - sz[0]/2, pos[1] - sz[1]/2), sz[0], sz[1], #type: ignore
                             facecolor='#3498db', edgecolor='#2c3e50', alpha=0.6)
        ax_2d.add_patch(rect)
        ax_2d.text(pos[0], pos[1], name, ha='center', va='center', fontsize=9, fontweight='bold')
        
        # Draw 3D stacked structure mesh
        verts = compute_box_vertices(sz, pos)
        draw_block_mesh(ax_3d, verts, face_color='#3498db', edge_color='#2c3e50')
        ax_3d.text(pos[0], pos[1], pos[2], name, ha='center', va='center', fontsize=8)

    ax_3d.view_init(elev=25, azim=-45)
    fig.canvas.draw()

def on_click(event):
    global block_counter
    # Only capture inputs coming from the flat 2D blueprint panel axis
    if event.inaxes != ax_2d or event.xdata is None or event.ydata is None:
        return
        
    snap_x = round(event.xdata / GRID_SNAP) * GRID_SNAP
    snap_y = round(event.ydata / GRID_SNAP) * GRID_SNAP
    
    if snap_y <= 0:
        print("Blocked: Goal positions must sit strictly in the +Y zone (Green Area)!")
        return

    # Handle automatic 3D physical stacking layers calculation
    current_highest_z = 0.0
    for b_data in placed_blocks.values():
        px, py, pz = b_data['position']
        # If overlapping on the 2D footprint
        if abs(px - snap_x) < BLOCK_SIZE[0]/2 and abs(py - snap_y) < BLOCK_SIZE[1]/2:
            # Reconstruct top boundary edge matching center of mass format
            top_surface = pz + b_data['size'][2]/2
            if top_surface > current_highest_z:
                current_highest_z = top_surface
                
    new_block_name = f"block{block_counter}"
    target_z_center = current_highest_z + BLOCK_SIZE[2]/2
    
    placed_blocks[new_block_name] = {
        'position': [float(snap_x), float(snap_y), float(target_z_center)],
        'orientation': [0.0, 0.0, 0.0],
        'size': [float(i) for i in BLOCK_SIZE]
    }
    
    placement_history.append(new_block_name) # Track history sequence
    block_counter += 1
    redraw_scene()

def generate_random_initial_staging():
    """Generates non-overlapping random initial configurations in the -Y layout zone."""
    init_positions = {}
    min_x, max_x = -WORKSPACE_LIMIT + 1.0, WORKSPACE_LIMIT - 1.0
    min_y, max_y = -WORKSPACE_LIMIT + 1.0, -1.0  
    margin = 1.2  
    
    for name in placed_blocks.keys():
        placed = False
        attempts = 0
        while not placed and attempts < 1000:
            rand_x = round(random.uniform(min_x, max_x) / GRID_SNAP) * GRID_SNAP
            rand_y = round(random.uniform(min_y, max_y) / GRID_SNAP) * GRID_SNAP
            rand_z = BLOCK_SIZE[2] / 2  
            
            collision = False
            for existing_data in init_positions.values():
                ex, ey, _ = existing_data['position']
                if abs(ex - rand_x) < margin and abs(ey - rand_y) < margin:
                    collision = True
                    break
                    
            if not collision:
                init_positions[name] = {
                    'position': [float(rand_x), float(rand_y), float(rand_z)],
                    'orientation': [0.0, 0.0, 0.0],
                    'size': [float(i) for i in BLOCK_SIZE]
                }
                placed = True
            attempts += 1
            
        if not placed:
            raise RuntimeError("Staging area space saturated. Increase workspace boundaries.")
            
    return init_positions

def on_key(event):
    global block_counter
    
    # NEW FEATURE: STEP-BY-STEP UNDO CONTROLLER
    if event.key == 'u':
        if not placement_history:
            print("No blocks left to undo!")
            return
            
        # Extract the last added block identifier
        last_block_name = placement_history.pop()
        del placed_blocks[last_block_name]
        
        # Adjust naming index step backward so sequence indexing remains flawless
        block_counter -= 1
        print(f"Removed {last_block_name}. Standing ready for substitution layout.")
        redraw_scene()

    elif event.key == 's':
        if not placed_blocks:
            print("No blocks layout active! Click on the left grid first.")
            return
        try:
            init_blocks = generate_random_initial_staging()
            
            with open('goal.yaml', 'w') as f_goal:
                yaml.dump(placed_blocks, f_goal, default_flow_style=False, sort_keys=False)
            with open('init.yaml', 'w') as f_init:
                yaml.dump(init_blocks, f_init, default_flow_style=False, sort_keys=False)
                
            print(f"\nSaved {len(placed_blocks)} blocks to 'goal.yaml' (+Y) and 'init.yaml' (-Y)!")
        except RuntimeError as e:
            print(f"Export failed: {e}")

fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('key_press_event', on_key)

redraw_scene()
plt.show()