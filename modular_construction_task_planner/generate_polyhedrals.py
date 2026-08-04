import numpy as np
import polyscope as ps
from scipy.spatial import ConvexHull
from typing import List, Tuple

def generate_diced_block(radius, num_points=12, angle_threshold=0.98) -> Tuple[np.ndarray, np.ndarray, List[dict]]:
    """
        Generates a dodecahedron-like block with extruded faces.

        Args:
            radius: Size of the block.
            num_points: Points to pick. ~12-16 is good for dice-like shapes.
            angle_threshold: Dot product threshold (0.98 ~= 11 degrees). Normals closer than this will be merged.
        Returns:
            vertices: The vertices of the generated block.
            simplices: The faces (simplices) of the block.
            unique_faces: A list of unique faces with their normals and centers.
    """
    # 1. Generate points with a "minimum distance" feel
    # We use a Fibonacci Sphere or Golden Spiral to get even distribution
    indices = np.arange(0, num_points, dtype=float) + 0.5
    phi = np.arccos(1 - 2*indices/num_points)
    theta = np.pi * (1 + 5**0.5) * indices

    x = radius * np.cos(theta) * np.sin(phi)
    y = radius * np.sin(theta) * np.sin(phi)
    z = radius * np.cos(phi)

    # Add a little jitter so they aren't 'too' perfect
    points = np.vstack((x, y, z)).T
    points += np.random.normal(0, radius*0.05, points.shape)

    # 2. Compute Hull
    hull = ConvexHull(points, qhull_options='QJ')

    # 3. Optimize: Merge faces with similar normals
    unique_faces = []
    seen_normals = []

    for i, eq in enumerate(hull.equations):
        normal = eq[:3]
        is_duplicate = False

        for sn in seen_normals:
            # Dot product check: 1.0 means identical, 0.0 means perpendicular
            if np.dot(normal, sn) > angle_threshold:
                is_duplicate = True
                break

        if not is_duplicate:
            # Calculate the face center (mean of vertices in this simplex)
            face_verts = hull.points[hull.simplices[i]]
            center = np.mean(face_verts, axis=0)

            unique_faces.append({
                'normal': normal,
                'center': center,
                'equation': eq
            })
            seen_normals.append(normal)

    return hull.points, hull.simplices, unique_faces

def compute_base_positions(clean_faces, base_offset=0.7) -> Tuple[List[List[float]], List[List[float]], List[List[float]]]:
    """
        Computes the base positions for the block based on the clean faces.

        Args:
            clean_faces: List of unique faces with normals and centers.
            base_offset: Distance to offset the base from the center of the face.
        Returns:
            List of base positions for the block.
    """
    # Extract centers and normals from our 'clean_faces' list
    centers = np.array([f['center'] for f in clean_faces])
    normals = np.array([f['normal'] for f in clean_faces])
    valid_normals = []
    valid_centers = []
    base_placements = []

    for i in range(len(normals)):
        n = normals[i]
        c = centers[i]

        # Filter: Only Side Faces (ignore top/bottom)
        if n[2] > -0.2 and np.linalg.norm(n[:2]) > 0.5:  # Mostly horizontal normals
            # Project normal onto XY plane and extend it
            # Base position = center + (normal_xy * offset), then forced to ground (z=0)
            dir_xy = n.copy()
            dir_xy[2] = 0
            # Normalize the XY direction
            dir_xy /= np.linalg.norm(dir_xy)

            base_pos = c + (dir_xy * base_offset)
            base_pos[2] = 0 # Force to ground
            base_placements.append(base_pos)

            valid_normals.append(n)
            valid_centers.append(c)

    return base_placements, valid_centers, valid_normals

def visualize_planning_scene(blocks: List[Tuple[List, List, List, List, List]]):
    ps.init()
    ps.set_up_dir("z_up")

    for i, (vertices, faces, valid_centers, valid_normals, base_placements) in enumerate(blocks):
        vertices = np.array(vertices)
        faces = np.array(faces)
        vertices[:, 0] += i * 0.5  # Offset each block for visibility
        ps.register_surface_mesh(f"Diced_Block_{i}", vertices, faces)

        # Display base placements
        if base_placements:
            base_placements = np.array(base_placements)
            base_placements[:, 0] += i * 0.5  # Offset for visibility

            ps_cloud = ps.register_point_cloud(f"Base_Targets_{i}", np.array(base_placements), radius=0.02)
            ps_cloud.add_color_quantity("target_color", np.full((len(base_placements), 3), [0, 1, 0]))

        valid_centers = np.array(valid_centers)
        valid_centers[:, 0] += i * 0.5  # Offset for visibility

        # Show only the 'Optimized' normals
        ps.register_point_cloud(f"Surface_Centers_{i}", np.array(valid_centers))
        ps.get_point_cloud(f"Surface_Centers_{i}").add_vector_quantity("unique_normals", np.array(valid_normals))

    ps.show()

def main():
    # --- Example Usage ---
    radius = 0.2  # 20cm block
    num_blocks = 5
    blocks = []

    for _ in range(num_blocks):
        verts, simplices, clean_faces = generate_diced_block(radius, 14)
        base_placements, valid_centers, valid_normals = compute_base_positions(clean_faces)
        blocks.append((verts, simplices, valid_centers, valid_normals, base_placements))

    visualize_planning_scene(blocks)

if __name__ == "__main__":
    main()