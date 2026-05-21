from copy import deepcopy

from trimesh import Trimesh
import numpy as np

from enum import Enum
from math import factorial
from typing import Tuple, Dict, cast, List, Any
from scipy.spatial import cKDTree
from modular_construction_task_planner.eas.core import (
    Optional, Pose, State, LinkedState,
    Entity, StateStatus, World
)
from modular_construction_task_planner.scripts.block_domain import (
    Action, Object, PosEntity, Robot,
)
from modular_construction_task_planner.scripts.stability import SupportNode, compute_placement_stability
from path_planner.path_planner_node import GridGraph, OCCUPANCY

HEURISTIC = Enum('HEURISTIC', 'LAZY SIMPLE_COLLISION DILIGENT ANTICIPATORY STABLE_DISCRETE STABLE STABLE_NAV')
"""
    LAZY: Only considers the euclidean distance to the target for the preferred action.
    SIMPLE_COLLISION: Checks for collisions and adds lazy collision cost if applicable. Lazy collision cost is the arc length
                       the robot needs to travel around the obstacle. The radius will be the the radius of the obstacle + the
                       half with of the robot base.
    DILIGENT: Runs full path planning for evaluation.
    ANTICIPATORY: Considers the hindrance the current action places onto future actions.
    STABLE_DISCRETE: For place actions, just checks if the place action can be performed, use lazy cost.
    STABLE: For place actions, uses the stability score as a heuristic.
    STABLE_NAV: For place actions, uses the stability score as a heuristic and considers navigation.
"""

class OrderedLandmarksPlanner:
    def __init__(self, world: World, action_dict: Dict[str, Action], gg: Optional[GridGraph] = None) -> None:
        self.world: World = world
        self.original_world = deepcopy(world)
        self.action_dict: Dict[str, Action] = action_dict
        self.gg: Optional[GridGraph] = gg
        self.support_graph: Dict[str, SupportNode] = {}
        if self.gg:
            self.original_gg = deepcopy(gg)

        self.state_counter: int = 0

        self.current_state: State = world.current_state
        self.s0: LinkedState = LinkedState(self.state_counter, self.current_state)
        self.current_linked_state: LinkedState = self.s0
        self.goal_linked_state: Optional[LinkedState] = None
        self.goal_linked_states: List[LinkedState] = []
        self.current_cost = 0.0
        self.solution_count = 100

        robot = self.world.entities.get_entities("robot")
        self.robot = cast(Robot, robot)

        blocks = self.world.entities.get_entities(Object)
        blocks = cast(List[Object], blocks)

        self.goal_positions = [b.goal.value for b in blocks]
        self.pick_positions = [b.reachable_from for b in blocks if b.goal.value]
        self.pick_positions = [pos for sublist in self.pick_positions for pos in sublist]
        self.place_positions = [b.placeable_from for b in blocks if b.goal.value]
        self.place_positions = [pos for sublist in self.place_positions for pos in sublist]

        self.num_potential_solutions = factorial(len(blocks)-3)
        print(f"Blocks: {[block.name for block in blocks]}")
        print(f"Number of blocks: {len(blocks)-3}")
        for block in blocks:
            if not block.goal.value:
                continue
            print(f"Block {block.name} can be reached from {len(block.reachable_from)} positions")
            self.num_potential_solutions *= len(block.reachable_from)
        print(f"Number of potential solutions: {self.num_potential_solutions}")

    def reset(self, solution_count: int = 100) -> None:
        self.world = self.original_world
        if self.gg:
            self.gg = self.original_gg
        if self.support_graph and self.original_support_graph:
            print("Resetting support graph to original state.")
            self.support_graph = self.original_support_graph

        self.current_state: State = self.world.current_state
        self.s0: LinkedState = LinkedState(self.state_counter, self.current_state)
        self.current_linked_state: LinkedState = self.s0
        self.state_counter = 0
        self.goal_linked_state = None
        self.goal_linked_states = []
        self.current_cost = 0.0
        self.solution_count = solution_count

        robot = self.world.entities.get_entities("robot")
        self.robot = cast(Robot, robot)

    def run_bfs_planner(self) -> List[LinkedState]:
        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state)
            weighted_branch = self.current_linked_state.branches_to_explore.pop(0)

            action_name, action_params, cost, additional_properties = weighted_branch
            action = self.action_dict[action_name]
            # print(f"Executing: {action_name} with params {[str(param) + ': ' + str(ent.name) \
                # for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)
            self.world.update_state()
            self.generate_new_linked_state(action_name, action_params, cost, additional_properties)

            if self.world.goal_reached:
                print("GOAL REACHED!")
                self.current_linked_state.goal = True
                self.goal_linked_states.append(self.current_linked_state)
                print(f"{len(self.goal_linked_states)} goal linked states found so far. {len(self.goal_linked_states)}/"
                      f"{self.num_potential_solutions} potential solutions explored.")

                self.backtrack()

        return self.goal_linked_states

    def run_heuristic_planner(self, heuristic: HEURISTIC = HEURISTIC.LAZY) -> List[LinkedState]:
        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state, heuristic)
            if not self.current_linked_state.branches_to_explore:
                print("No branches to explore, backtracking...")
                self.backtrack()
                continue

            weighted_branch = min(self.current_linked_state.branches_to_explore, key=lambda x: x[2])
            action_name, action_params, cost, additional_properties = weighted_branch
            self.current_linked_state.branches_to_explore.remove(weighted_branch)
            self.current_cost += cost
            action = self.action_dict[action_name]
            # print(f"Executing: {action_name} with params {[str(param) + ': ' + str(ent.name) \
                # for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)
            if self.gg:
                if action_name == 'pick':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                elif action_name == 'place':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)

            self.world.update_state()
            self.generate_new_linked_state(action_name, action_params, self.current_cost, additional_properties)

            if self.world.goal_reached:
                print("GOAL REACHED using lazy heuristic!")
                self.current_linked_state.goal = True
                self.goal_linked_states.append(self.current_linked_state)
                print(f"{len(self.goal_linked_states)} goal linked states found so far. {len(self.goal_linked_states)}/"
                      f"{self.num_potential_solutions} potential solutions explored.")
                break

        return self.goal_linked_states

    def run_multi_bound_planner(self) -> Optional[LinkedState]:
        best_cost = float('inf')
        home_state = self.s0
        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state, HEURISTIC.LAZY)
            if not self.current_linked_state.branches_to_explore:
                print("No branches to explore, backtracking...")
                self.backtrack()
                home_state = self.current_linked_state
                continue

            weighted_branch = min(self.current_linked_state.branches_to_explore, key=lambda x: x[2])
            self.current_linked_state.branches_to_explore.remove(weighted_branch)
            action_name, action_params, cost, additional_properties = weighted_branch
            action = self.action_dict[action_name]
            self.current_cost += cost
            if self.current_cost > best_cost:
                # print(f"Current cost {self.current_cost} exceeds best cost {best_cost}, skipping action {action_name}, "
                    #   f"removing cost {cost} and backtracking...")
                self.current_cost -= cost
                self.backtrack()
                home_state = self.current_linked_state
                continue
            # print(f"Executing: {action_name} with params {[str(param) + ': ' + str(ent.name) \
                # for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)
            if self.gg:
                if action_name == 'pick':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                elif action_name == 'place':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)

            self.world.update_state()
            self.generate_new_linked_state(action_name, action_params, self.current_cost, additional_properties)

            if self.world.goal_reached:
                print(f"GOAL REACHED using lazy heuristic in {self.current_linked_state.state_id} steps!")
                self.current_linked_state.goal = True
                goal_state = self.current_linked_state
                nav_cost = self.nav_cost_from_home_to_target(home_state, goal_state)

                upper_bound = nav_cost + home_state.cost
                if upper_bound < best_cost:
                    best_cost = upper_bound
                    self.goal_linked_state = self.current_linked_state
                    print(f"New best cost found: {best_cost}.")
                else:
                    print(f"New bound {upper_bound} is not better than current best cost {best_cost}.")

                self.backtrack()
                home_state = self.current_linked_state
                self.solution_count -= 1
                if self.solution_count <= 0:
                    print("Solution count limit reached, terminating search.")
                    break

        return self.goal_linked_state

    def run_stable_planner(self, support_graph: Dict[str, SupportNode], ground_mesh: Trimesh,
                           h: HEURISTIC = HEURISTIC.STABLE_DISCRETE, verbose: bool = False) -> Optional[LinkedState]:
        if not self.support_graph:
            self.support_graph = support_graph
            self.original_support_graph = deepcopy(self.support_graph)
        self.ground_mesh = ground_mesh

        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state, h, forecast=True, verbose=verbose)
            if not self.current_linked_state.branches_to_explore:
                print("No branches, terminating")
                return self.goal_linked_state
                # print("No branches to explore, backtracking...")
                # self.backtrack()
                # continue

            weighted_branch = min(self.current_linked_state.branches_to_explore, key=lambda x: x[2])
            action_name, action_params, cost, additional_properties = weighted_branch
            self.current_linked_state.branches_to_explore.remove(weighted_branch)
            self.current_cost += cost
            action = self.action_dict[action_name]
            print(f"Executing: {action_name} with params {[str(param) + ' : ' + str(ent.name) \
                for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)
            if self.gg:
                if action_name == 'pick':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                elif action_name == 'place':
                    obj_pos = self.world.pose_dict[action_params['target_pose'].name].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)

            self.world.update_state()
            self.generate_new_linked_state(action_name, action_params, self.current_cost, additional_properties)

            if action_name == 'place':
                support_score = 1 - weighted_branch[2]
                obj_entity = cast(Object, action_params['object'])
                obj_support_node = self.support_graph[obj_entity.name]
                obj_support_node.current_support_score = support_score

                for supported_name, supported_edge in obj_support_node.supported_objects.items():
                    obj_support_node.supported_objects[supported_name] = (supported_edge[0], True)
                    supported_obj_support_node = self.support_graph[supported_name]
                    supported_obj_support_node.supporting_objects[obj_entity.name] = (supported_edge[0], True)

            if self.world.goal_reached:
                print("GOAL REACHED using stable lazy heuristic!")
                self.current_linked_state.goal = True
                self.goal_linked_states.append(self.current_linked_state)
                self.goal_linked_state = self.current_linked_state
                print(f"{len(self.goal_linked_states)} goal linked states found so far. {len(self.goal_linked_states)}/"
                      f"{self.num_potential_solutions} potential solutions explored.")
                break

        return self.goal_linked_state

    def generate_new_linked_state(self, action_name: str, action_params: Dict[str, Entity], cost: float,
                                  additional_properties: Dict[str, Any] = {}) -> None:
        new_state = self.world.current_state
        self.state_counter += 1
        action_log = (action_name, tuple(f"{ent.name}" for ent in action_params.values()))
        new_linked_state = LinkedState(self.state_counter, new_state, parent=(action_name, self.current_linked_state),
                                        cost=cost, action_from_parent=action_log, properties=additional_properties)
        self.current_linked_state.children.append((action_name, new_linked_state))
        self.current_linked_state = new_linked_state
        self.current_state = new_state

    def branch_out(self, linked_state: LinkedState, heuristic: HEURISTIC = HEURISTIC.LAZY,
                   forecast: bool = False, verbose: bool = False) -> None:
        """
            Branch out from the current state by defining branches based on the preferred action and evaluating the branches.
            Assigns the weighted branches to the linked state.
        """
        if linked_state.branches_to_explore:
            # Need to check the branches again after backtacking, since the state of the world is different.
            for branch in linked_state.branches_to_explore:
                action_name, branch_params, _, _ = branch
                if not self.action_dict[action_name].check(branch_params):
                    linked_state.branches_to_explore.remove(branch)
            return

        preferred_action_name = self.get_preferred_action()
        branches = self.define_branches_based_on_action(preferred_action_name)

        if forecast:
            weighted_branches = self.evaluate_cost_to_next_transit(branches, preferred_action_name, heuristic, verbose)
        else:
            weighted_branches = self.evaluate_branches(branches, preferred_action_name, heuristic, verbose)
        linked_state.branches_to_explore = weighted_branches

    def get_preferred_action(self) -> str:
        """
            Find the preferred action to perform at the given state.
        """
        robot_pos = self.robot.at.value
        gripper_empty = self.robot.gripper_empty.value

        # Since now there are many options for picking and placing positions for each object,
        # we need to perform preferred action selection differently.
        if robot_pos in self.pick_positions and gripper_empty:
            preferred_action_name = "pick"
        elif robot_pos in self.place_positions and not gripper_empty:
            preferred_action_name = "place"
        else:
            # These two are just move actions, but it's good to disinguish these two cases to help with target selection.
            if gripper_empty:
                preferred_action_name = "transit"
            else:
                preferred_action_name = "transport"

        return preferred_action_name

    def define_branches_based_on_action(self, action_name: str) -> List[Dict[str, Entity]]:
        """
            Define branches based on the given action name and state. Returns a list of dictionaries of action parameters
            for the preferred action, with each element being a set of potential parameters for the action, or None if no
            branches can be defined. This is either no targets can be found or if all potential targets are not
            applicable.
        """
        branches = []
        robot_pos = self.robot.at.value
        robot_pos = cast(str, robot_pos)
        current_pos_entity = self.world.entities.get_entities(robot_pos)
        current_pos_entity = cast(PosEntity, current_pos_entity)
        # print(f"Defining branches for action: {action_name} at robot position: {robot_pos}")

        match action_name:
            case "pick":
                potential_target_objs = cast(List[Object], self.world.not_at_goal_entities)
                for potential_obj in potential_target_objs:
                    if robot_pos in potential_obj.reachable_from:
                        obj_pos = cast(str, potential_obj.at.value)
                        obj_pos_entity = cast(PosEntity, self.world.entities.get_entities(obj_pos))
                        branch_params = {
                            'robot': self.robot,
                            'object': potential_obj,
                            'target_pose': obj_pos_entity
                        }
                        branches.append(branch_params)

            case "place":
                obj_in_gripper = cast(str, self.robot.holding.value)
                obj_entity_in_gripper = cast(Object, self.world.entities.get_entities(obj_in_gripper))
                obj_goal_pos = cast(str, obj_entity_in_gripper.goal.value)
                obj_goal_pos_entity = cast(PosEntity, self.world.entities.get_entities(obj_goal_pos))

                if current_pos_entity.name in obj_entity_in_gripper.placeable_from:
                    branch_params = {
                        'robot': self.robot,
                        'object': obj_entity_in_gripper,
                        'target_pose': obj_goal_pos_entity
                    }
                    branches.append(branch_params)

            case "transit":
                potential_target_objs = cast(List[Object], self.world.not_at_goal_entities)
                potential_target_objs = [obj for obj in potential_target_objs if obj.goal.value] # Only consider objects that still need to be placed
                potential_target_pos_vals = [obj.reachable_from for obj in potential_target_objs]

                for potential_target_pos_sublist, potential_obj in zip(potential_target_pos_vals, potential_target_objs):
                    for target_pos in potential_target_pos_sublist:
                        pos_entity = cast(PosEntity, self.world.entities.get_entities(target_pos))
                        branch_params = {
                            'robot': self.robot,
                            'start_pose': current_pos_entity,
                            'target_pose': pos_entity,
                            'object': potential_obj
                        }
                        branches.append(branch_params)

            case "transport":
                obj_in_gripper = cast(str, self.robot.holding.value)
                obj_entity_in_gripper = cast(Object, self.world.entities.get_entities(obj_in_gripper))
                pose_num = robot_pos[-1]
                place_pos_val = f"{obj_in_gripper}_place_target{pose_num}"
                placeable_pos_entity = cast(PosEntity, self.world.entities.get_entities(place_pos_val))

                branch_params = {
                    'robot': self.robot,
                    'start_pose': current_pos_entity,
                    'target_pose': placeable_pos_entity,
                    'object': obj_entity_in_gripper
                }
                branches.append(branch_params)

            case _:
                pass

        for branch in branches:
            if not self.action_dict[action_name].check(branch):
                branches.remove(branch)

        return branches

    def evaluate_branches(self,
                          branches: List[Dict[str, Entity]],
                          action_name: str,
                          heuristic: HEURISTIC = HEURISTIC.LAZY,
                          verbose: bool = False) -> List[Tuple[str, Dict[str, Entity], float, Dict[str, Any]]]:
        """
            Evaluate the given branches and return a list of tuples of the branch and its cost, which is defined as the
            euclidean distance between the robot and the target. The branch with the lowest cost will be explored first.
        """
        evaluated_branches = []
        additional_properties = {}

        if action_name == "pick" or action_name == "place":
            evaluated_branches = [(action_name, branch, 0.0, additional_properties) for branch in branches]
        elif action_name == "transit" or action_name == "transport":
            for branch in branches:
                start_pos = branch['start_pose'].name
                target_pos = branch['target_pose'].name
                start_pos = self.world.pose_dict[start_pos].position
                target_pos = self.world.pose_dict[target_pos].position

                match heuristic:
                    case HEURISTIC.LAZY:
                        cost = np.linalg.norm(np.array(start_pos) - np.array(target_pos))
                    case HEURISTIC.STABLE_DISCRETE:
                        obj_entity = cast(Object, branch['object'])
                        is_stable, support_score = self.evaluate_obj_stability(obj_entity, self.support_graph, self.ground_mesh, verbose=verbose)
                        print(f"Branch for {action_name} object {obj_entity.name} is {'stable' if is_stable else 'unstable'} "
                            f"with support score {support_score}.")

                        if is_stable:
                            cost = 1 - support_score
                        else:
                            support_node = self.support_graph[obj_entity.name]
                            print(support_node)
                            continue
                    case HEURISTIC.STABLE:
                        obj_entity = cast(Object, branch['object'])
                        is_stable, support_score = self.evaluate_obj_stability(obj_entity, self.support_graph, self.ground_mesh, verbose=verbose)
                        print(f"Branch for {action_name} object {obj_entity.name} is {'stable' if is_stable else 'unstable'} "
                            f"with support score {support_score}.")

                        if is_stable:
                            cost = (1 - support_score) + np.linalg.norm(np.array(start_pos) - np.array(target_pos))
                        else:
                            continue
                    case HEURISTIC.SIMPLE_COLLISION:
                        cost = self.simple_collision_heuristic(start_pos, target_pos)
                    case HEURISTIC.DILIGENT:
                        if not self.gg:
                            raise ValueError("GridGraph is not initialized for diligent heuristic.")
                        path = self.gg.plan(start_pos[:2], target_pos[:2])
                        if path.size == 0:
                            cost = float('inf')
                        else:
                            path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                            cost = path_length
                    case HEURISTIC.ANTICIPATORY:
                        pass
                    case _:
                        print(f"Unknown heuristic: {heuristic}, defaulting to lazy.")
                        cost = np.linalg.norm(np.array(start_pos) - np.array(target_pos))

                # print(f"Evaluated branch for action {action_name} with target position {branch['target_pose'].name} has cost {cost}")

                evaluated_branches.append((action_name, branch, cost, additional_properties))

        return evaluated_branches

    def evaluate_cost_to_next_transit(self,
                                      branches: List[Dict[str, Entity]],
                                      action_name: str,
                                      heuristic: HEURISTIC = HEURISTIC.LAZY,
                                      verbose: bool = False) -> List[Tuple[str, Dict[str, Entity], float, Dict[str, Any]]]:
        """
            At a Transit state, meaning a state to perform a transit action, look ahead from current position to transit targets,
            then from each target to the transport target. And check if the target object can be placed stably.

            Return [prune, cost], where prune is a boolean indicating whether to prune this branch or not, and cost is
            the evaluated cost to the next transit state.
        """
        evaluated_branches = []
        additional_properties = {}

        if action_name != 'transit':
            evaluated_branches.append((action_name, branches[0], 0.0, additional_properties))
            return evaluated_branches

        for transit_branch in branches:
            obj_entity = cast(Object, transit_branch['object'])
            transit_start_pos_name = transit_branch['start_pose'].name
            transit_target_pos_name = transit_branch['target_pose'].name
            transit_start_pos = self.world.pose_dict[transit_start_pos_name].position
            transit_target_pos = self.world.pose_dict[transit_target_pos_name].position

            goal_pos_name = obj_entity.goal.value
            goal_pos_name = cast(str, goal_pos_name)
            goal_pos = self.world.pose_dict[goal_pos_name].position

            is_stable, support_score = self.evaluate_obj_stability(obj_entity, self.support_graph, self.ground_mesh, verbose=verbose)
            # support_score = np.clip(support_score, 0.0, 1.0)
            additional_properties = {'support_score': support_score}
            print(f"Branch for transit object {obj_entity.name} is {'stable' if is_stable else 'unstable'} "
                f"with support score {support_score}.")

            if not is_stable:
                continue

            match heuristic:
                case HEURISTIC.LAZY:
                    cost = np.linalg.norm(np.array(transit_start_pos) - np.array(transit_target_pos)).item()
                    cost += np.linalg.norm(np.array(transit_target_pos) - np.array(goal_pos)).item()
                case HEURISTIC.STABLE:
                    cost = (1 - support_score)
                case HEURISTIC.STABLE_NAV:
                    cost = (1 - support_score)
                    cost += np.linalg.norm(np.array(transit_start_pos) - np.array(transit_target_pos)).item() * 0.8
                    cost += np.linalg.norm(np.array(transit_target_pos) - np.array(goal_pos)).item() * 0.8
                case HEURISTIC.SIMPLE_COLLISION:
                    cost = self.simple_collision_heuristic(transit_start_pos, transit_target_pos)
                    cost += self.simple_collision_heuristic(transit_target_pos, goal_pos)
                case HEURISTIC.DILIGENT:
                    if not self.gg:
                        raise ValueError("GridGraph is not initialized for diligent heuristic.")
                    path_to_transit = self.gg.plan(transit_start_pos[:2], transit_target_pos[:2])
                    obj_pos = cast(str, obj_entity.at.value)
                    obj_pos = self.world.pose_dict[obj_pos].position
                    self.gg.update_block_move(obj_pos[:2], OCCUPANCY.FREE)

                    path_to_transport = self.gg.plan(transit_target_pos[:2], goal_pos[:2])
                    self.gg.update_block_move(obj_pos[:2], OCCUPANCY.OCCUPIED)
                    if path_to_transit.size == 0:
                        continue
                    else:
                        transit_path_length = np.sum(np.linalg.norm(np.diff(path_to_transit, axis=0), axis=1))
                        transport_path_length = np.sum(np.linalg.norm(np.diff(path_to_transport, axis=0), axis=1))
                        cost = transit_path_length.item() + transport_path_length.item()
                case HEURISTIC.ANTICIPATORY:
                    pass
                case _:
                    print(f"Unknown heuristic: {heuristic}, defaulting to lazy.")
                    cost = np.linalg.norm(np.array(transit_start_pos) - np.array(transit_target_pos)).item()
                    cost += np.linalg.norm(np.array(transit_target_pos) - np.array(goal_pos)).item()

            evaluated_branches.append((action_name, transit_branch, cost, additional_properties))

        return evaluated_branches

    def evaluate_obj_stability(self, obj: Object, support_graph: Dict[str, SupportNode], ground_mesh: Trimesh,
                               verbose: bool = False) -> Tuple[bool, float]:
        """
            Evaluate the stability of the branch by checking the support score of the target block after placing.
            If the score is below the threshold, return a high cost to discourage exploring this branch.

            Returns a tuple of whether the placement is stable and the support score.
        """
        branch_support_score = 0.0
        obj_support_node = support_graph[obj.name]
        supporting_objs = []
        supp_names = []
        sd = {}

        for parent_name, (score, is_placed) in obj_support_node.supporting_objects.items():
            if is_placed:
                branch_support_score += score
                parent_obj = self.world.entities.get_entities(parent_name)
                parent_obj = cast(Object, parent_obj)
                supporting_objs.append(parent_obj)
                supp_names.append(parent_name)

        is_stable = branch_support_score >= obj_support_node.support_threshold

        if not is_stable:
            overall_support_score = obj_support_node.support_combo_dict.get(tuple(supp_names), None)
            branch_support_score = overall_support_score if overall_support_score else branch_support_score
            if not overall_support_score and supp_names:
                sd, overall_support_score = compute_placement_stability(obj, supporting_objs, [], ground_mesh, verbose)
                branch_support_score = overall_support_score
                obj_support_node.support_combo_dict.update({tuple(supp_names): overall_support_score})

        if verbose:
            print(f"Evaluating stability for object {obj.name} with supporting objects {supp_names}. Initial check: {'passed' if is_stable else 'not passed'}\n"
                  f"Individual support scores: {[supp_name + ': ' + str(score) for supp_name, (score, _) in support_graph[obj.name].supporting_objects.items()]}.\n"
                  f"Combined support score: {branch_support_score}. Threshold: {obj_support_node.support_threshold}.")
            print(f"Support data: {sd}")

        is_stable = branch_support_score >= obj_support_node.support_threshold
        return is_stable, branch_support_score

    def simple_collision_heuristic(self, start_pos: List[float], target_pos: List[float]) -> float:
        """
            Simple heuristic to check for potential collisions along the path from start_pos to target_pos. If there are
            potential collisions, adds a cost based on the distance the robot needs to travel around the obstacle, which is
            defined as the radius of the obstacle + half the width of the robot base.
        """
        start_target_vec = np.array(target_pos) - np.array(start_pos)

        obstacle_entities = self.world.entities.get_entities(Object)
        obstacle_entities = cast(List[Object], obstacle_entities)
        obstacle_positions = [self.world.pose_dict[obs.at.value].position for obs in obstacle_entities if obs.at.value]

        robot_radius = 0.6 / 2.0
        block_radius = 0.15
        col_radius = robot_radius + block_radius
        collision_cost = np.linalg.norm(start_target_vec).item()

        dists, scalings = compute_dists_from_points_to_vector(np.array(obstacle_positions), start_target_vec, np.array(start_pos))
        dists = dists[(scalings >= 0) & (scalings <= 1)]

        for obj_idx, dist in enumerate(dists):
            if dist <= col_radius:
                additional_cost = compute_arc_length(start_pos, target_pos, obstacle_positions[obj_idx], col_radius)
                collision_cost += additional_cost
                # print(f"Potential collision detected with obstacle at distance {dist}. Adding collision cost {additional_cost}, total cost now {collision_cost}.")

        return collision_cost

    def backtrack(self) -> None:
        """
            Backtrack until current state is alive and has branches to explore.
        """
        # print("----------Backtracking----------")
        while not self.current_linked_state.branches_to_explore:
            parent_action_linked_state = self.current_linked_state.parent
            parent = parent_action_linked_state[1] if parent_action_linked_state is not None else None

            # print(
            #     f"Querying parent: {(parent.state_id, parent.status) if parent else None}"
            #     f" from current state: {(self.current_linked_state.state_id, self.current_linked_state.status)}"
            # )

            if parent is not None:
                self.current_linked_state = parent
                self.current_cost = parent.cost
                self.current_state = self.current_linked_state.state

                if self.gg:
                    action_from_parent = parent.action_from_parent
                    if action_from_parent is None:
                        print(f"No action from parent for linked state with id {parent.state_id}, cannot update grid graph.")
                        continue
                    action_name, action_params = action_from_parent
                    if action_name == 'pick':
                        obj_pos = self.world.pose_dict[action_params[2]].position[:2]
                        self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)
                    elif action_name == 'place':
                        obj_pos = self.world.pose_dict[action_params[2]].position[:2]
                        self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)

                # print(
                #     f"Backtracking to state id: {self.current_linked_state.state_id},"
                #     f" with {len(self.current_linked_state.branches_to_explore)} branches to explore."
                # )
            else:
                print("No parent to backtrack to, terminating.")
                break

        # print("----------Finished----------")
        self.world.update_entities_from_state(self.current_state)
        self.world.update_state()

    def nav_cost_from_home_to_target(self, home_linked_state: LinkedState, target_linked_state: LinkedState) -> float:
        self.gg = cast(GridGraph, self.gg)  # Type hint for better code completion
        cost = home_linked_state.cost
        current_linked_state = target_linked_state
        parent = current_linked_state.parent
        state_id_path = [current_linked_state.state_id]
        while parent is not None and parent[1] != home_linked_state:
            _, parent_linked_state = parent
            state_id_path.append(parent_linked_state.state_id)
            current_linked_state = parent_linked_state
            parent = current_linked_state.parent

        state_id_path.reverse()
        parent = home_linked_state
        for state_id in state_id_path:
            child_linked_state = next((child for child in parent.children if child[1].state_id == state_id), None)
            if child_linked_state is not None:
                action_from_parent = child_linked_state[1].action_from_parent
                if action_from_parent is None:
                    print(f"No action from parent for linked state with id {child_linked_state[1].state_id}")
                    continue
                action_name, action_params = action_from_parent
                if action_name == 'pick':
                    obj_pos = self.world.pose_dict[action_params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                elif action_name == 'place':
                    obj_pos = self.world.pose_dict[action_params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)
                elif action_name == 'transit' or action_name == 'transport':
                    start_pos = self.world.pose_dict[action_params[1]].position
                    target_pos = self.world.pose_dict[action_params[2]].position
                    path = self.gg.plan(start_pos[:2], target_pos[:2])
                    if path.size == 0:
                        cost = float('inf')
                    else:
                        path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                        cost += path_length
                        child_linked_state[1].cost = cost

                parent = child_linked_state[1]

        return cost

    @staticmethod
    def retrace_best_plan(goal_linked_states: List[LinkedState] | LinkedState) -> Tuple[List[Tuple[str, Tuple[str, ...]]], float]:
        # Backtrack to get all the plans with total costs
        plans = []
        if isinstance(goal_linked_states, LinkedState):
            goal_linked_states = [goal_linked_states]

        for goal_linked_state in goal_linked_states:
            plan = []
            total_cost = 0.0
            current_linked_state = goal_linked_state
            parent = current_linked_state.parent
            while parent is not None:
                plan.append(current_linked_state.action_from_parent)  # (action_name, involved_entities)
                current_linked_state = parent[1]
                parent = current_linked_state.parent

            total_cost = goal_linked_state.cost
            plan.reverse()  # Reverse to get the correct order from initial state to goal
            plans.append((plan, total_cost))

        best_plan = min(plans, key=lambda plan: plan[1])  # Get the plan with the lowest total cost
        return best_plan

    @staticmethod
    def unpack_stability_results(goal_linked_state: LinkedState) -> Tuple[List[float], float, float]:
        stability_scores = []
        total_stability_score = 0.0
        average_stability_score = 0.0
        current_linked_state = goal_linked_state
        parent = current_linked_state.parent
        while parent is not None and current_linked_state.action_from_parent is not None:
            action_name, _ = current_linked_state.action_from_parent
            if action_name == 'transit':
                support_score = current_linked_state.properties.get('support_score', None)
                if support_score is not None:
                    stability_scores.append(support_score)
            current_linked_state = parent[1]
            parent = current_linked_state.parent

        if stability_scores:
            total_stability_score = sum(stability_scores)
            average_stability_score = total_stability_score / len(stability_scores)

        return stability_scores[::-1], total_stability_score, average_stability_score

    def compute_lazy_nav_cost(self, plan: List[Tuple[str, Tuple[str, ...]]]) -> float:
        pp_cost = 0.0
        for action_name, params in plan:
            match action_name:
                case 'pick' | 'place':
                    continue
                case 'transit' | 'transport':
                    start_pos = self.world.pose_dict[params[1]].position
                    target_pos = self.world.pose_dict[params[2]].position
                    path_length = np.linalg.norm(np.array(start_pos) - np.array(target_pos)).item()
                    pp_cost += path_length
                case _:
                    print(f"Unknown action {action_name} in plan. Skipping...")
        return pp_cost

    def compute_full_nav_cost(self, plan: List[Tuple[str, Tuple[str, ...]]]) -> float:
        self.gg = cast(GridGraph, self.gg)  # Type hint for better code completion
        pp_cost = 0.0
        for action_name, params in plan:
            match action_name:
                case 'pick':
                    obj_pos = self.world.pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                case 'place':
                    obj_pos = self.world.pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)
                case 'transit' | 'transport':
                    start_pos = self.world.pose_dict[params[1]].position[:2]
                    target_pos = self.world.pose_dict[params[2]].position[:2]
                    path = self.gg.plan(start_pos, target_pos)
                    if path.size == 0:
                        print(f"No path found for action {action_name} from {start_pos} to {target_pos}.")
                    else:
                        path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                        pp_cost += path_length
                        print(f"Path found for action {action_name} from {start_pos} to {target_pos} "
                              f"with length {path_length:.2f}.")
                        # Optionally, visualize the path here using RViz or another tool
                case _:
                    print(f"Unknown action {action_name} in plan. Skipping...")

        return pp_cost

def compute_dists_from_points_to_vector(points: np.ndarray, vector: np.ndarray, start_point: np.ndarray, 
                                        verbose: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """
        Compute the shortest distances from a set of points to a vector.
        1. Project each point onto the vector
        2. Scale the original vector based on the projection.
        3. Compute the difference between the vector to the point and the scaled vector on the original vector.
        4. Compute the norm of the resultant vectors as the distance.
    """
    init_to_other_blocks_vecs = [np.array(other_pos) - np.array(start_point) for other_pos in points]
    vecs_projected_on_init_goal_vec = np.array([np.dot(vec, vector) for vec in init_to_other_blocks_vecs])

    vector_mag = np.linalg.norm(vector)
    if vector_mag <= 1e-6:
        projected_vecs_scaling_factors = np.zeros(len(init_to_other_blocks_vecs))
    else:
        projected_vecs_scaling_factors = vecs_projected_on_init_goal_vec / (vector_mag**2)

    for i in range(len(projected_vecs_scaling_factors)):
        projection = vecs_projected_on_init_goal_vec[i]

        if abs(projection) <= 1e-6:
            projected_vecs_scaling_factors[i] = np.linalg.norm(init_to_other_blocks_vecs[i]) / vector_mag \
                                                                                               if vector_mag > 1e-6 \
                                                                                               else 0.0

    # If scaling == 0, need to do something different
    scaled_projected_vecs = [scaling * vector for scaling in projected_vecs_scaling_factors]
    dists = [np.linalg.norm(vec - proj_vec) if np.linalg.norm(proj_vec) > 1e-6 else 0.0 
             for vec, proj_vec in zip(init_to_other_blocks_vecs, scaled_projected_vecs)]

    if verbose:
        print(f"Vector: {vector}, Start point: {start_point}")
        for i, point in enumerate(points):
            print(f"Point: {point}, Vector to point: {init_to_other_blocks_vecs[i]}, "
                  f"Projected vector: {scaled_projected_vecs[i]}, Distance: {dists[i]}")

    return np.array(dists), np.array(projected_vecs_scaling_factors)

def compute_arc_length(start_pos: List[float], target_pos: List[float], obj_pos: List[float], col_radius: float) -> float:
    """
        Compute the arc length the robot needs to travel around the obstacle, which is defined as the radius of the
        obstacle + half the width of the robot base.
    """
    start_target_vec = np.array(target_pos) - np.array(start_pos)
    start_obj_vec = np.array(obj_pos) - np.array(start_pos)
    target_obj_vec = np.array(obj_pos) - np.array(target_pos)

    start_target_dist = np.linalg.norm(start_target_vec)
    if start_target_dist <= 1e-6:
        return 0.0

    start_obj_dist = np.linalg.norm(start_obj_vec)
    target_obj_dist = np.linalg.norm(target_obj_vec)

    if start_obj_dist <= 1e-6 or target_obj_dist <= 1e-6:
        return 0.0

    angle_at_obstacle = np.arccos(np.clip(np.dot(start_obj_vec, target_obj_vec) / (start_obj_dist * target_obj_dist), -1.0, 1.0))
    arc_length = angle_at_obstacle * col_radius

    return arc_length