import numpy as np

from typing import Tuple, Dict, cast, List
from modular_construction_task_planner.eas.core import (
    State, LinkedState,
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
        print(f"Initial state: {self.current_state}")
        self.current_linked_state: LinkedState = self.s0
        self.goal_linked_states: List[LinkedState] = []

        robot = self.world.entities.get_entities("robot")
        self.robot = cast(Robot, robot)

        blocks = self.world.entities.get_entities(Object)
        blocks = cast(List[Object], blocks)
        self.goal_positions = [b.goal.value for b in blocks]

    def run_optimal_planner(self) -> List[LinkedState]:
        while self.current_linked_state.status == StateStatus.ALIVE:
            self.branch_out(self.current_linked_state)
            weighted_branch = self.current_linked_state.branches_to_explore.pop(0)

            action_name, action_params, cost = weighted_branch
            action = self.action_dict[action_name]
            print(f"Executing: {action_name} with params {[str(param) + ': ' + str(ent.state) for param, ent in action_params.items()]} and cost {cost}")
            action.execute(action_params)

            self.world.update_state()

            new_state = self.world.current_state
            self.state_counter += 1
            new_linked_state = LinkedState(self.state_counter, new_state, parent=(action_name, self.current_linked_state),
                                           cost=cost)
            self.current_linked_state.children.append((action_name, new_linked_state))
            self.current_linked_state = new_linked_state
            self.current_state = new_state

            if self.world.goal_reached:
                print("GOAL REACHED!")
                self.current_linked_state.goal = True
                self.goal_linked_states.append(self.current_linked_state)
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
        branches = self.define_branches_based_on_action(preferred_action_name, linked_state.state)
        weighted_branches = self.evaluate_branches(branches, preferred_action_name)
        linked_state.branches_to_explore = weighted_branches

    def get_preferred_action(self) -> str:
        """
            Find the preferred action to perform at the given state.
        """
        robot_pos = self.robot.at.value
        gripper_empty = self.robot.gripper_empty.value

        blocks = self.world.entities.get_entities(Object)
        blocks = cast(List[Object], blocks)
        current_block_positions = [b.at.value for b in blocks]

        if robot_pos in current_block_positions and gripper_empty and robot_pos not in self.goal_positions:
            preferred_action_name = "pick"
        elif robot_pos in self.goal_positions and not gripper_empty:
            preferred_action_name = "place"
        else:
            # These two are just move actions, but it's good to disinguish these two cases to help with target selection.
            if gripper_empty:
                preferred_action_name = "transit"
            else:
                preferred_action_name = "transport"

        return preferred_action_name

    def define_branches_based_on_action(self, action_name: str, state: State) -> List[Dict[str, Entity]]:
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

        match action_name:
            case "pick":
                obj_at_pos = current_pos_entity.occupied_by.value
                if obj_at_pos is not None:
                    obj_entity = self.world.entities.get_entities(obj_at_pos)
                    obj_entity = cast(Object, obj_entity)

                    branch_params = {
                        'robot': self.robot,
                        'object': obj_entity,
                        'object_pose': current_pos_entity
                    }
                    branches.append(branch_params)

            case "place":
                current_pos_clear = current_pos_entity.clear.value
                if current_pos_clear:
                    obj_in_gripper = cast(str, self.robot.holding.value)
                    obj_entity_in_gripper = cast(Object, self.world.entities.get_entities(obj_in_gripper))

                    if obj_entity_in_gripper.goal.value == current_pos_entity.name:
                        branch_params = {
                            'robot': self.robot,
                            'object': obj_entity_in_gripper,
                            'target_pose': current_pos_entity
                        }
                        branches.append(branch_params)

            case "transit":
                potential_target_objs = cast(List[Object], self.world.not_at_goal_entities)
                potential_target_pos_vals = [obj.at.value for obj in potential_target_objs]
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
                target_pos_entity = cast(PosEntity, self.world.entities.get_entities(target_pos_val))
                branch_params = {
                    'robot': self.robot,
                    'start_pose': current_pos_entity,
                    'target_pose': target_pos_entity
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
        print("----------Backtracking----------")
        while not self.current_linked_state.branches_to_explore:
            parent_action_linked_state = self.current_linked_state.parent
            parent = parent_action_linked_state[1] if parent_action_linked_state is not None else None

            print(f"Querying parent: {(parent.state_id, parent.status) if parent else None} from current state: {(self.current_linked_state.state_id, self.current_linked_state.status)}")

            if parent_action_linked_state is not None:
                self.current_linked_state = parent_action_linked_state[1]
                self.current_state = self.current_linked_state.state
                print(f"Backtracking to state id: {self.current_linked_state.state_id}, with {len(self.current_linked_state.branches_to_explore)} branches to explore.")
            else:
                print("No parent to backtrack to, terminating.")
                break

        print("----------Finished----------")
        self.world.update_entities_from_state(self.current_state)
        self.world.update_state()