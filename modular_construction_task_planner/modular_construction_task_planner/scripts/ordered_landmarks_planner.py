import numpy as np

from typing import Tuple, Dict, cast, List, Optional
from copy import deepcopy

from modular_construction_task_planner.eas.core import (
    State, LinkedState,
    Entity, Entities, StateStatus, World
)
from modular_construction_task_planner.scripts.block_domain import (
    Object, PosEntity, Robot,
    MoveAction, PickAction, PlaceAction
)

# Outline of the planner:
# First make the exhaustive optimal planner
# 1. Initialize LinkedState with initial state
# 2. While current state is alive:
#    a. If no branches, define branches
#    b. Pop a branch and branch out to get to the next state, if no more branches, mark state as dead
#    c. Update current state to new state, and connect the new state to the current state with the action taken
#    d. If goal reached, add goal state to goal list and backtrack to a previous alive state
# 3. Return all goal states

# Branching out:
# 1. Find the preferred action at the current state
# 2. Find potential targets for the action, and create branches for each target
#    a. Assign euclidean distance as cost for each branch
# 3. If no targets, mark state as dead and backtrack

# Backtrack
# 1. If parent is not None and ALIVE, set current state to parent and mark current state as DEAD.


class OrderedLandmarksPlanner:
    def __init__(self, world: World):
        self.world: World = world
        self.state_counter: int = 0

        self.current_state: State = world.current_state
        self.s0: LinkedState = LinkedState(self.state_counter, self.current_state)
        self.current_linked_state: LinkedState = self.s0
        self.goal_linked_states: List[LinkedState] = []

        robot = self.world.entities.get_entities("robot")
        self.robot = cast(Robot, robot)

    def run_optimal_planner(self) -> List[LinkedState]:

        return self.goal_linked_states

    def get_preferred_action(self, state: State) -> Optional[str]:
        """
            Find the preferred action to perform at the given state. None if no preferred action can be found.
        """
        
        pass

    def define_branches_based_on_action(self, action_name: str, state: State) -> Optional[List[Dict[str, Entity]]]:
        """
            Define branches based on the given action name and state. Returns a list of dictionaries of action parameters
            for the preferred action, with each element being a set of potential parameters for the action, or None if no
            branches can be defined. This is either no targets can be found or if all potential targets are not
            applicable.
        """
        
        pass

    def evaluate_branches(self, branches: List[Dict[str, Entity]], action_name: str) -> List[Tuple[Dict[str, Entity], float]]:
        """
            Evaluate the given branches and return a list of tuples of the branch and its cost, which is defined as the
            euclidean distance between the robot and the target. The branch with the lowest cost will be explored first.
        """
        evaluated_branches = []

        return evaluated_branches

    def backtrack(self) -> None:
        """
            Backtrack until current state is alive and has branches to explore.
        """
        while not self.current_linked_state.branches_to_explore:
            parent_action_linked_state = self.current_linked_state.parent
            if parent_action_linked_state is not None and parent_action_linked_state[1].state_type == StateStatus.ALIVE:
                self.current_linked_state = parent_action_linked_state[1]
                self.current_state = self.current_linked_state.state

        self.world.update_entities_from_state(self.current_state)
        self.world.update_state()