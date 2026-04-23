import numpy as np

from math import factorial
from typing import Tuple, Dict, cast, List
from modular_construction_task_planner.eas.core import (
    Pose, State, LinkedState,
    Entity, StateStatus, World
)
from modular_construction_task_planner.scripts.block_domain import (
    Action, Object, PosEntity, Robot,
)

class OrderedLandmarksPlanner:
    def __init__(self, world: World, action_dict: Dict[str, Action]) -> None:
        self.world: World = world
        self.action_dict: Dict[str, Action] = action_dict
        self.state_counter: int = 0

        self.current_state: State = world.current_state
        self.s0: LinkedState = LinkedState(self.state_counter, self.current_state)
        self.current_linked_state: LinkedState = self.s0
        self.goal_linked_states: List[LinkedState] = []

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
        print(f"Number of blocks: {len(blocks)}-3")
        for block in blocks:
            if not block.goal.value:
                continue
            print(f"Block {block.name} can be reached from {len(block.reachable_from)} positions")
            self.num_potential_solutions *= len(block.reachable_from)
        print(f"Number of potential solutions: {self.num_potential_solutions}")

    def run_optimal_planner(self) -> List[LinkedState]:
        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state)
            weighted_branch = self.current_linked_state.branches_to_explore.pop(0)

            action_name, action_params, cost = weighted_branch
            action = self.action_dict[action_name]
            # print(f"Executing: {action_name} with params {[str(param) + ': ' + str(ent.name) \
            #     for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)

            self.world.update_state()

            new_state = self.world.current_state
            self.state_counter += 1
            action_log = (action_name, tuple(f"{ent.name}" for ent in action_params.values()))
            new_linked_state = LinkedState(self.state_counter, new_state, parent=(action_name, self.current_linked_state),
                                           cost=cost, action_from_parent=action_log)
            self.current_linked_state.children.append((action_name, new_linked_state))
            self.current_linked_state = new_linked_state
            self.current_state = new_state

            if self.world.goal_reached:
                print("GOAL REACHED!")
                self.current_linked_state.goal = True
                self.goal_linked_states.append(self.current_linked_state)
                print(f"{len(self.goal_linked_states)} goal linked states found so far. {len(self.goal_linked_states)}/{self.num_potential_solutions} potential solutions explored.")

                self.backtrack()

        return self.goal_linked_states

    def branch_out(self, linked_state: LinkedState) -> None:
        """
            Branch out from the current state by defining branches based on the preferred action and evaluating the branches.
            Assigns the weighted branches to the linked state.
        """
        if linked_state.branches_to_explore:
            # Need to check the branches again after backtacking, since the state of the world is different.
            for branch in linked_state.branches_to_explore:
                action_name, branch_params, _ = branch
                if not self.action_dict[action_name].check(branch_params):
                    linked_state.branches_to_explore.remove(branch)
            return

        preferred_action_name = self.get_preferred_action()
        branches = self.define_branches_based_on_action(preferred_action_name)
        weighted_branches = self.evaluate_branches(branches, preferred_action_name)
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

        # Think about if these checks for pick and place are necessary, since:
        # 1. Preferred action selection should have already filtered out some states
        # 2. The condition check at the end should be responsible for filtering out invalid branches.
        # For now just keep doing hacky solutions lol
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
                            'object_pose': obj_pos_entity
                        }
                        branches.append(branch_params)

            case "place":
                current_pos_clear = current_pos_entity.clear.value
                if current_pos_clear:
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
                potential_target_pos_vals = [obj.reachable_from for obj in potential_target_objs if obj.goal.value]
                potential_target_pos_vals = [pos for sublist in potential_target_pos_vals for pos in sublist]

                for target_pos in potential_target_pos_vals:
                    if not target_pos:
                        continue

                    pos_entity = cast(PosEntity, self.world.entities.get_entities(target_pos))
                    branch_params = {
                        'robot': self.robot,
                        'start_pose': current_pos_entity,
                        'target_pose': pos_entity
                    }
                    branches.append(branch_params)

            case "transport":
                obj_in_gripper = cast(str, self.robot.holding.value)
                obj_entity_in_gripper = cast(Object, self.world.entities.get_entities(obj_in_gripper))
                target_pos_val = cast(str, obj_entity_in_gripper.goal.value)

                obj_current_pose_name = cast(str, obj_entity_in_gripper.at.value)
                obj_current_pose = self.world.pose_dict[obj_current_pose_name]
                target_pose = self.world.pose_dict[target_pos_val]

                # Compute transformation matrix and apply it to the robot's current pose to get the target pose for the robot.
                robot_current_pose_name = cast(str, self.robot.at.value)
                robot_current_pose = self.world.pose_dict[robot_current_pose_name]
                robot_place_pose = robot_current_pose.transform_to_frame(obj_current_pose, target_pose)

                obj_in_gripper_entity = cast(Object, self.world.entities.get_entities(obj_in_gripper))
                placeable_pose_names = obj_in_gripper_entity.placeable_from

                # IMPORTANT: Need to publish these new tfs to the world, otherwise motion planning cannot be done.
                # Might be easier to calculate and publish all of them in WorldManager, and make sure the name is consistent
                # with pick placement poses, so I can just find the matching name to get the corresponding place pose.
                placeable_pos_entity = None
                for placeable_pose_name in placeable_pose_names:
                    placeable_pose = self.world.pose_dict.get(placeable_pose_name)
                    placeable_pos_entity = self.world.entities.get_entities(placeable_pose_name)
                    if not placeable_pose:
                        self.world.pose_dict[placeable_pose_name] = Pose(
                            position=robot_place_pose[:3, 3].tolist(),
                            orientation=[0, 0, 0, 1]
                        )
                        break

                branch_params = {
                    'robot': self.robot,
                    'start_pose': current_pos_entity,
                    'target_pose': placeable_pos_entity
                }
                branches.append(branch_params)

            case _:
                pass

        for branch in branches:
            if not self.action_dict[action_name].check(branch):
                branches.remove(branch)

        return branches

    def evaluate_branches(self, branches: List[Dict[str, Entity]], action_name: str) -> List[Tuple[str, Dict[str, Entity], float]]:
        """
            Evaluate the given branches and return a list of tuples of the branch and its cost, which is defined as the
            euclidean distance between the robot and the target. The branch with the lowest cost will be explored first.
        """
        evaluated_branches = []
        if action_name == "pick" or action_name == "place":
            evaluated_branches = [(action_name, branch, 0.0) for branch in branches]
        elif action_name == "transit" or action_name == "transport":
            for branch in branches:
                start_pos = branch['start_pose'].name
                target_pos = branch['target_pose'].name
                start_pos = self.world.pose_dict[start_pos].position
                target_pos = self.world.pose_dict[target_pos].position

                cost = np.linalg.norm(np.array(start_pos) - np.array(target_pos))
                evaluated_branches.append((action_name, branch, cost))

        return evaluated_branches

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

            if parent_action_linked_state is not None:
                self.current_linked_state = parent_action_linked_state[1]
                self.current_state = self.current_linked_state.state
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