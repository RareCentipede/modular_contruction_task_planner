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
    world_poses: Dict[str, "Pose"],
    default_mass: float = 1.0,
    default_mu: float = 0.5,
    gravity: float = 9.81
) -> Tuple[bool, float, float]:
    active_objects = [o for o in objects if o.at.value is not None and o.at.value in world_poses]
    N = len(active_objects)
    if N == 0:
        return True, 0.0, 0.0

    obj_idx_map = {obj.name: idx for idx, obj in enumerate(active_objects)}
    all_contacts: List[Tuple["ContactPoint", "Object", "Object", float]] = []

    # --------------------------------------------------------------------------
    # 1. Ground Contacts (Transformed to World Frame)
    # --------------------------------------------------------------------------
    for obj in active_objects:
        T_matrix = extract_object_pose_matrix(obj, world_poses)
        R = T_matrix[:3, :3]
        p_com = T_matrix[:3, 3]
        half_height = obj.dim[2] / 2.0 if obj.dim else 0.5

        # Check if bottom face is near ground (Z=0)
        min_z = p_com[2] - half_height
        if min_z < 0.05:
            half_x = obj.dim[0] / 2.0 if obj.dim else 0.5
            half_y = obj.dim[1] / 2.0 if obj.dim else 0.5

            local_corners = [
                np.array([-half_x, -half_y, -half_height]),
                np.array([ half_x, -half_y, -half_height]),
                np.array([ half_x,  half_y, -half_height]),
                np.array([-half_x,  half_y, -half_height]),
            ]

            n_world = np.array([0.0, 0.0, 1.0])  # Ground normal always points +Z
            t1_world = np.array([1.0, 0.0, 0.0])
            t2_world = np.array([0.0, 1.0, 0.0])

            for l_pt in local_corners:
                w_pt = p_com + R @ l_pt
                cp = ContactPoint(w_pt, n_world, t1_world, t2_world)
                all_contacts.append((cp, obj, None, default_mu)) # type: ignore

    # --------------------------------------------------------------------------
    # 2. Inter-Object Contacts
    # --------------------------------------------------------------------------
    for i, obj_a in enumerate(active_objects):
        for j, obj_b in enumerate(active_objects):
            if i < j:
                T_a = extract_object_pose_matrix(obj_a, world_poses)
                T_b = extract_object_pose_matrix(obj_b, world_poses)

                # Determine top vs bottom based on Z position
                if T_a[2, 3] >= T_b[2, 3]:
                    top_obj, bot_obj = obj_a, obj_b
                else:
                    top_obj, bot_obj = obj_b, obj_a

                cps = extract_contact_points_between_objects(top_obj, bot_obj, world_poses)
                for cp in cps:
                    # Enforce normal pointing from bot_obj TO top_obj
                    if cp.normal[2] < 0:
                        cp.normal = -cp.normal
                    all_contacts.append((cp, top_obj, bot_obj, default_mu))

    K = len(all_contacts)
    if K == 0:
        # No contacts: net force is downward gravity, net torque is zero
        net_force = np.linalg.norm(sum(np.array([0.0, 0.0, -getattr(obj, 'mass', default_mass) * gravity]) for obj in active_objects)).item()
        net_torque = np.linalg.norm(sum(np.array([0.0, 0.0, 0.0]) for obj in active_objects)).item()
        return False, net_force, net_torque

    num_vars = 5 * K
    A_eq = np.zeros((6 * N, num_vars))
    b_eq = np.zeros(6 * N)

    # --------------------------------------------------------------------------
    # 3. Construct Unified Equilibrium System
    # --------------------------------------------------------------------------
    for idx, obj in enumerate(active_objects):
        m = getattr(obj, 'mass', default_mass)
        b_eq[6 * idx + 2] = m * gravity  # Gravity acts in -Z

    for k, (cp, top_obj, bot_obj, _) in enumerate(all_contacts):
        col = 5 * k

        v_n  = cp.normal
        v_t1 = cp.tangent1
        v_t2 = cp.tangent2

        # A. Force/Torque on TOP OBJECT (+ Force)
        if top_obj and top_obj.name in obj_idx_map:
            top_idx = obj_idx_map[top_obj.name]
            p_com_top = extract_object_pose_matrix(top_obj, world_poses)[:3, 3]
            r_top = cp.position - p_com_top

            # Linear Force Balance (+F)
            A_eq[6 * top_idx + 0, col + 0] += v_n[0]
            A_eq[6 * top_idx + 1, col + 0] += v_n[1]
            A_eq[6 * top_idx + 2, col + 0] += v_n[2]

            A_eq[6 * top_idx + 0, col + 1] += v_t1[0]
            A_eq[6 * top_idx + 0, col + 2] -= v_t1[0]
            A_eq[6 * top_idx + 1, col + 1] += v_t1[1]
            A_eq[6 * top_idx + 1, col + 2] -= v_t1[1]
            A_eq[6 * top_idx + 2, col + 1] += v_t1[2]
            A_eq[6 * top_idx + 2, col + 2] -= v_t1[2]

            A_eq[6 * top_idx + 0, col + 3] += v_t2[0]
            A_eq[6 * top_idx + 0, col + 4] -= v_t2[0]
            A_eq[6 * top_idx + 1, col + 3] += v_t2[1]
            A_eq[6 * top_idx + 1, col + 4] -= v_t2[1]
            A_eq[6 * top_idx + 2, col + 3] += v_t2[2]
            A_eq[6 * top_idx + 2, col + 4] -= v_t2[2]

            # Torque (+ r_top x F)
            t_n  = np.cross(r_top, v_n)
            t_t1 = np.cross(r_top, v_t1)
            t_t2 = np.cross(r_top, v_t2)

            A_eq[6 * top_idx + 3, col + 0] += t_n[0]
            A_eq[6 * top_idx + 4, col + 0] += t_n[1]
            A_eq[6 * top_idx + 5, col + 0] += t_n[2]

            A_eq[6 * top_idx + 3, col + 1] += t_t1[0]; A_eq[6 * top_idx + 3, col + 2] -= t_t1[0]
            A_eq[6 * top_idx + 4, col + 1] += t_t1[1]; A_eq[6 * top_idx + 4, col + 2] -= t_t1[1]
            A_eq[6 * top_idx + 5, col + 1] += t_t1[2]; A_eq[6 * top_idx + 5, col + 2] -= t_t1[2]

            A_eq[6 * top_idx + 3, col + 3] += t_t2[0]; A_eq[6 * top_idx + 3, col + 4] -= t_t2[0]
            A_eq[6 * top_idx + 4, col + 3] += t_t2[1]; A_eq[6 * top_idx + 4, col + 4] -= t_t2[1]
            A_eq[6 * top_idx + 5, col + 3] += t_t2[2]; A_eq[6 * top_idx + 5, col + 4] -= t_t2[2]

        # B. Force/Torque on BOTTOM OBJECT (- Force)
        if bot_obj and bot_obj.name in obj_idx_map:
            bot_idx = obj_idx_map[bot_obj.name]
            p_com_bot = extract_object_pose_matrix(bot_obj, world_poses)[:3, 3]
            r_bot = cp.position - p_com_bot

            # Linear Force Balance (-F)
            A_eq[6 * bot_idx + 0, col + 0] -= v_n[0]
            A_eq[6 * bot_idx + 1, col + 0] -= v_n[1]
            A_eq[6 * bot_idx + 2, col + 0] -= v_n[2]

            A_eq[6 * bot_idx + 0, col + 1] -= v_t1[0]
            A_eq[6 * bot_idx + 0, col + 2] += v_t1[0]
            A_eq[6 * bot_idx + 1, col + 1] -= v_t1[1]
            A_eq[6 * bot_idx + 1, col + 2] += v_t1[1]
            A_eq[6 * bot_idx + 2, col + 1] -= v_t1[2]
            A_eq[6 * bot_idx + 2, col + 2] += v_t1[2]

            A_eq[6 * bot_idx + 0, col + 3] -= v_t2[0]
            A_eq[6 * bot_idx + 0, col + 4] += v_t2[0]
            A_eq[6 * bot_idx + 1, col + 3] -= v_t2[1]
            A_eq[6 * bot_idx + 1, col + 4] += v_t2[1]
            A_eq[6 * bot_idx + 2, col + 3] -= v_t2[2]
            A_eq[6 * bot_idx + 2, col + 4] += v_t2[2]

            # Torque (- r_bot x F)
            t_n  = np.cross(r_bot, v_n)
            t_t1 = np.cross(r_bot, v_t1)
            t_t2 = np.cross(r_bot, v_t2)

            A_eq[6 * bot_idx + 3, col + 0] -= t_n[0]
            A_eq[6 * bot_idx + 4, col + 0] -= t_n[1]
            A_eq[6 * bot_idx + 5, col + 0] -= t_n[2]

            A_eq[6 * bot_idx + 3, col + 1] -= t_t1[0]; A_eq[6 * bot_idx + 3, col + 2] += t_t1[0]
            A_eq[6 * bot_idx + 4, col + 1] -= t_t1[1]; A_eq[6 * bot_idx + 4, col + 2] += t_t1[1]
            A_eq[6 * bot_idx + 5, col + 1] -= t_t1[2]; A_eq[6 * bot_idx + 5, col + 2] += t_t1[2]

            A_eq[6 * bot_idx + 3, col + 3] -= t_t2[0]; A_eq[6 * bot_idx + 3, col + 4] -= t_t2[0]
            A_eq[6 * bot_idx + 4, col + 3] -= t_t2[1]; A_eq[6 * bot_idx + 4, col + 4] += t_t2[1]
            A_eq[6 * bot_idx + 5, col + 3] -= t_t2[2]; A_eq[6 * bot_idx + 5, col + 4] += t_t2[2]

    # --------------------------------------------------------------------------
    # 4. Friction Pyramids
    # --------------------------------------------------------------------------
    A_ub = np.zeros((2 * K, num_vars))
    b_ub = np.zeros(2 * K)

    for k, (_, _, _, mu) in enumerate(all_contacts):
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
        c[5 * k] = 1.0  # Minimize contact normal forces

    bounds = [(0, None) for _ in range(num_vars)]

    # --------------------------------------------------------------------------
    # 5. Solve LP & Compute Net Force Vector + Net Torque Vector
    # --------------------------------------------------------------------------
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')

    net_force = 0.0
    net_torque = 0.0

    if res.success:
        # Calculate resulting wrench (F_contact - F_gravity) per object
        for idx, obj in enumerate(active_objects):
            wrench_result = A_eq[6 * idx: 6 * idx + 6] @ res.x - b_eq[6 * idx: 6 * idx + 6]
            net_force += np.linalg.norm(wrench_result[0:3]).item()   # 3D Force Vector [Fx, Fy, Fz]
            net_torque += np.linalg.norm(wrench_result[3:6]).item()  # 3D Torque Vector [Tx, Ty, Tz]

        return True, net_force, net_torque
    else:
        # Unstable case: Return net uncompensated forces (pure gravity & zero contact reaction)
        for idx, obj in enumerate(active_objects):
            m = getattr(obj, 'mass', default_mass)
            net_force += np.linalg.norm(np.array([0.0, 0.0, -m * gravity])).item()

        return False, net_force, net_torque