import trimesh
import numpy as np
from dataclasses import dataclass
from scipy.optimize import linprog
from typing import List, Dict, Tuple, TYPE_CHECKING

from eas.core import Pose  # Imported from core.py

if TYPE_CHECKING:
    from modular_construction_task_planner.block_domain import Object  # Imported from block_domain.py

@dataclass
class ContactPoint:
    position: np.ndarray  # 3D contact coordinate [x, y, z]
    normal: np.ndarray    # Unit normal vector (pointing into supported object)
    tangent1: np.ndarray  # First orthogonal friction vector
    tangent2: np.ndarray  # Second orthogonal friction vector

def extract_object_pose_matrix(obj: "Object", world_poses: Dict[str, Pose]) -> np.ndarray:
    """Extracts the 4x4 homogeneous transformation matrix for an Object."""
    pose_val = obj.at.value
    if pose_val in world_poses:
        return world_poses[pose_val].homogeneous
    return np.eye(4)

def extract_contact_points_between_objects(
    obj_top: "Object",
    obj_bot: "Object",
    world_poses: Dict[str, Pose]
) -> List[ContactPoint]:
    """
    Computes contact surface patches between two Object meshes in world space.
    """
    T_top = extract_object_pose_matrix(obj_top, world_poses)
    T_bot = extract_object_pose_matrix(obj_bot, world_poses)

    # Transform meshes into world coordinates
    mesh_top = obj_top.mesh.copy().apply_transform(T_top)
    mesh_bot = obj_bot.mesh.copy().apply_transform(T_bot)

    # Collision / Contact Query using Trimesh
    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object("bot", mesh_bot)

    is_collision, contacts = collision_manager.in_collision_single( # type: ignore
        mesh_top, return_data=True
    )

    if not is_collision or not contacts:
        return []

    contact_points = []
    normal = np.array([0.0, 0.0, 1.0])  # Dominant support normal (+Z)
    tangent1 = np.array([1.0, 0.0, 0.0])
    tangent2 = np.array([0.0, 1.0, 0.0])

    for contact_data in contacts:
        # Extract contact point location
        pt = contact_data.point
        contact_points.append(ContactPoint(pt, normal, tangent1, tangent2))

    return contact_points

def compute_stablelego_equilibrium(
    objects: List["Object"],
    world_poses: Dict[str, Pose],
    default_mass: float = 1.0,
    default_mu: float = 0.5,
    gravity: float = 9.81
) -> Tuple[bool, Dict[str, float]]:
    """
    Evaluates static equilibrium for a set of placed Object entities.
    Returns (is_stable, per_object_residuals).
    """
    # Filter only objects currently placed in the world (ignore held or unplaced objects)
    active_objects = [o for o in objects if o.at.value is not None]
    N = len(active_objects)
    if N == 0:
        return True, {}

    all_contacts: List[Tuple[ContactPoint, "Object", float]] = []

    # 1. Collect ground and inter-object contacts
    for obj in active_objects:
        T_matrix = extract_object_pose_matrix(obj, world_poses)
        p_com = T_matrix[:3, 3]
        half_height = obj.dim[2] / 2.0 if obj.dim else 0.05

        # Ground contact condition (near Z=0)
        if abs(p_com[2] - half_height) < 1e-3:
            # Create 4 support points at base polygon corners
            half_x = obj.dim[0] / 2.0 if obj.dim else 0.05
            half_y = obj.dim[1] / 2.0 if obj.dim else 0.05
            corners = [
                np.array([p_com[0] - half_x, p_com[1] - half_y, 0.0]),
                np.array([p_com[0] + half_x, p_com[1] - half_y, 0.0]),
                np.array([p_com[0] + half_x, p_com[1] + half_y, 0.0]),
                np.array([p_com[0] - half_x, p_com[1] + half_y, 0.0]),
            ]
            for pt in corners:
                cp = ContactPoint(pt, np.array([0, 0, 1]), np.array([1, 0, 0]), np.array([0, 1, 0]))
                all_contacts.append((cp, obj, default_mu))

    # Collect pairwise object contacts
    for i, top_obj in enumerate(active_objects):
        for j, bot_obj in enumerate(active_objects):
            if i != j:
                cps = extract_contact_points_between_objects(top_obj, bot_obj, world_poses)
                for cp in cps:
                    all_contacts.append((cp, top_obj, default_mu))

    K = len(all_contacts)
    if K == 0:
        return False, {"error": 1.0}

    # 2. Variable Vector Setup: 5 variables per contact point [f_n, f_t1+, f_t1-, f_t2+, f_t2-]
    num_vars = 5 * K
    A_eq = np.zeros((6 * N, num_vars))
    b_eq = np.zeros(6 * N)

    # 3. Construct Force Balance Equations
    for idx, obj in enumerate(active_objects):
        T_matrix = extract_object_pose_matrix(obj, world_poses)
        p_com = T_matrix[:3, 3]

        # Gravity Wrench
        b_eq[6 * idx + 2] = default_mass * gravity

        for k, (cp, target_obj, _) in enumerate(all_contacts):
            if target_obj.name == obj.name:
                col = 5 * k

                # Force translational component
                A_eq[6 * idx + 0, col + 1] = cp.tangent1[0]
                A_eq[6 * idx + 0, col + 2] = -cp.tangent1[0]
                A_eq[6 * idx + 0, col + 3] = cp.tangent2[0]
                A_eq[6 * idx + 0, col + 4] = -cp.tangent2[0]

                A_eq[6 * idx + 1, col + 1] = cp.tangent1[1]
                A_eq[6 * idx + 1, col + 2] = -cp.tangent1[1]
                A_eq[6 * idx + 1, col + 3] = cp.tangent2[1]
                A_eq[6 * idx + 1, col + 4] = -cp.tangent2[1]

                A_eq[6 * idx + 2, col + 0] = cp.normal[2]

                # Torque component: (c - p_com) x Force
                r = cp.position - p_com
                t_n = np.cross(r, cp.normal)
                t_t1 = np.cross(r, cp.tangent1)
                t_t2 = np.cross(r, cp.tangent2)

                A_eq[6 * idx + 3: 6 * idx + 6, col + 0] = t_n
                A_eq[6 * idx + 3: 6 * idx + 6, col + 1] = t_t1
                A_eq[6 * idx + 3: 6 * idx + 6, col + 2] = -t_t1
                A_eq[6 * idx + 3: 6 * idx + 6, col + 3] = t_t2
                A_eq[6 * idx + 3: 6 * idx + 6, col + 4] = -t_t2

    # 4. Friction Pyramid Constraints
    A_ub = np.zeros((2 * K, num_vars))
    b_ub = np.zeros(2 * K)

    for k, (_, _, mu) in enumerate(all_contacts):
        col = 5 * k
        coeff = mu / np.sqrt(2.0)

        A_ub[2 * k, col + 0] = -coeff
        A_ub[2 * k, col + 1] = 1.0
        A_ub[2 * k, col + 2] = 1.0

        A_ub[2 * k + 1, col + 0] = -coeff
        A_ub[2 * k + 1, col + 3] = 1.0
        A_ub[2 * k + 1, col + 4] = 1.0

    # Objective: Minimize normal force sum
    c = np.zeros(num_vars)
    for k in range(K):
        c[5 * k] = 1.0

    bounds = [(0, None) for _ in range(num_vars)]

    # 5. Solve Linear Program
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')

    if res.success:
        residuals = {
            obj.name: float(np.linalg.norm(A_eq[6 * idx: 6 * idx + 6] @ res.x - b_eq[6 * idx: 6 * idx + 6]))
            for idx, obj in enumerate(active_objects)
        }
        return True, residuals
    else:
        return False, {"error": 1.0}