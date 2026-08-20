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
    normal: np.ndarray    # Unit normal vector
    tangent1: np.ndarray  # First orthogonal friction vector
    tangent2: np.ndarray  # Second orthogonal friction vector

def extract_object_pose_matrix(obj: "Object", world_poses: Dict[str, Pose]) -> np.ndarray:
    pose_val = obj.at.value
    if pose_val in world_poses:
        return world_poses[pose_val].homogeneous
    return np.eye(4)

def extract_contact_points_between_objects(
    obj_top: "Object",
    obj_bot: "Object",
    world_poses: Dict[str, Pose],
    tolerance: float = 0.05
) -> List[ContactPoint]:
    """Computes contact surface patches between two objects using 2D overlapping projections."""
    T_top = extract_object_pose_matrix(obj_top, world_poses)
    T_bot = extract_object_pose_matrix(obj_bot, world_poses)

    pos_top = T_top[:3, 3]
    pos_bot = T_bot[:3, 3]

    dim_top = obj_top.dim if obj_top.dim else [1.0, 1.0, 1.0]
    dim_bot = obj_bot.dim if obj_bot.dim else [1.0, 1.0, 1.0]

    # Check vertical proximity (bottom of top object vs top of bot object)
    z_bottom_top = pos_top[2] - dim_top[2] / 2.0
    z_top_bot = pos_bot[2] + dim_bot[2] / 2.0

    if abs(z_bottom_top - z_top_bot) > tolerance:
        return []

    # Check XY overlap bounds
    x_overlap = min(pos_top[0] + dim_top[0]/2, pos_bot[0] + dim_bot[0]/2) - max(pos_top[0] - dim_top[0]/2, pos_bot[0] - dim_bot[0]/2)
    y_overlap = min(pos_top[1] + dim_top[1]/2, pos_bot[1] + dim_bot[1]/2) - max(pos_top[1] - dim_top[1]/2, pos_bot[1] - dim_bot[1]/2)

    if x_overlap <= 0 or y_overlap <= 0:
        return []

    # Generate 4 corner contact points on the overlapping surface patch
    min_x = max(pos_top[0] - dim_top[0]/2, pos_bot[0] - dim_bot[0]/2)
    max_x = min(pos_top[0] + dim_top[0]/2, pos_bot[0] + dim_bot[0]/2)
    min_y = max(pos_top[1] - dim_top[1]/2, pos_bot[1] - dim_bot[1]/2)
    max_y = min(pos_top[1] + dim_top[1]/2, pos_bot[1] + dim_bot[1]/2)
    z_contact = z_bottom_top

    normal = np.array([0.0, 0.0, 1.0])
    tangent1 = np.array([1.0, 0.0, 0.0])
    tangent2 = np.array([0.0, 1.0, 0.0])

    pts = [
        np.array([min_x, min_y, z_contact]),
        np.array([max_x, min_y, z_contact]),
        np.array([max_x, max_y, z_contact]),
        np.array([min_x, max_y, z_contact]),
    ]

    return [ContactPoint(pt, normal, tangent1, tangent2) for pt in pts]


def compute_stablelego_equilibrium(
    objects: List["Object"],
    world_poses: Dict[str, Pose],
    default_mass: float = 1.0,
    default_mu: float = 0.5,
    gravity: float = 9.81
) -> Tuple[bool, Dict[str, float], Dict[str, float]]:
    active_objects = [o for o in objects if o.at.value is not None and o.at.value in world_poses]
    N = len(active_objects)
    if N == 0:
        return True, {}, {}
    all_contacts: List[Tuple[ContactPoint, "Object", float]] = []

    # 1. Ground Contacts (Z near 0)
    for obj in active_objects:
        T_matrix = extract_object_pose_matrix(obj, world_poses)
        p_com = T_matrix[:3, 3]
        half_height = obj.dim[2] / 2.0 if obj.dim else 0.5

        if abs(p_com[2] - half_height) < 0.05:
            half_x = obj.dim[0] / 2.0 if obj.dim else 0.5
            half_y = obj.dim[1] / 2.0 if obj.dim else 0.5
            corners = [
                np.array([p_com[0] - half_x, p_com[1] - half_y, 0.0]),
                np.array([p_com[0] + half_x, p_com[1] - half_y, 0.0]),
                np.array([p_com[0] + half_x, p_com[1] + half_y, 0.0]),
                np.array([p_com[0] - half_x, p_com[1] + half_y, 0.0]),
            ]
            for pt in corners:
                cp = ContactPoint(pt, np.array([0, 0, 1]), np.array([1, 0, 0]), np.array([0, 1, 0]))
                all_contacts.append((cp, obj, default_mu))

    # 2. Inter-Object Contacts
    for i, top_obj in enumerate(active_objects):
        for j, bot_obj in enumerate(active_objects):
            if i != j:
                cps = extract_contact_points_between_objects(top_obj, bot_obj, world_poses)
                for cp in cps:
                    all_contacts.append((cp, top_obj, default_mu))

    K = len(all_contacts)
    if K == 0:
        return False, {obj.name: 1.0 for obj in active_objects}, {obj.name: 0.0 for obj in active_objects}

    num_vars = 5 * K
    A_eq = np.zeros((6 * N, num_vars))
    b_eq = np.zeros(6 * N)

    # 3. Construct Balance Matrices
    for idx, obj in enumerate(active_objects):
        T_matrix = extract_object_pose_matrix(obj, world_poses)
        p_com = T_matrix[:3, 3]

        # Gravity Wrench
        b_eq[6 * idx + 2] = obj.mass * gravity

        for k, (cp, target_obj, _) in enumerate(all_contacts):
            if target_obj.name == obj.name:
                col = 5 * k

                A_eq[6 * idx + 0, col + 1] = cp.tangent1[0]
                A_eq[6 * idx + 0, col + 2] = -cp.tangent1[0]
                A_eq[6 * idx + 0, col + 3] = cp.tangent2[0]
                A_eq[6 * idx + 0, col + 4] = -cp.tangent2[0]

                A_eq[6 * idx + 1, col + 1] = cp.tangent1[1]
                A_eq[6 * idx + 1, col + 2] = -cp.tangent1[1]
                A_eq[6 * idx + 1, col + 3] = cp.tangent2[1]
                A_eq[6 * idx + 1, col + 4] = -cp.tangent2[1]

                A_eq[6 * idx + 2, col + 0] = cp.normal[2]

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

    c = np.zeros(num_vars)
    for k in range(K):
        c[5 * k] = 1.0

    bounds = [(0, None) for _ in range(num_vars)]

    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')

    # In rbe_solver.py, replace the final return section (lines 142-152):

    if res.success:
        residuals = {}
        object_forces = {}

        for idx, obj in enumerate(active_objects):
            # Compute static force balance error
            err = np.linalg.norm(A_eq[6 * idx: 6 * idx + 6] @ res.x - b_eq[6 * idx: 6 * idx + 6])
            residuals[obj.name] = float(err)

            # Sum normal forces (f_n at col = 5*k) acting on this object
            obj_normal_force = 0.0
            for k, (_, target_obj, _) in enumerate(all_contacts):
                if target_obj.name == obj.name:
                    obj_normal_force += res.x[5 * k]  # col + 0 is cp.normal
            object_forces[obj.name] = float(obj_normal_force)

        return True, residuals, object_forces
    else:
        return False, {obj.name: 1.0 for obj in active_objects}, {obj.name: 0.0 for obj in active_objects}