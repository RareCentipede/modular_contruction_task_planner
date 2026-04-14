from modular_construction_task_planner.scripts.block_domain import PosEntity, Object, Robot
from modular_construction_task_planner.eas.core import load_domains, Condition, Effect, Action

POSE_VAR_DOMAIN = ("g", "p1", "p2", "p3", "")
BLOCK_VAR_DOMAIN = ("robot", "block1", "block2", "block3", "")
BOOL_VAR_DOMAIN = (True, False)

VarDomains = {'pos': POSE_VAR_DOMAIN, 'block': BLOCK_VAR_DOMAIN, 'bool': BOOL_VAR_DOMAIN}

load_domains(VarDomains)

block1 = Object('block1')
p1 = PosEntity('p1')
p1.occupied_by.value = 'robot'
p2 = PosEntity('p2')
p3 = PosEntity('p3')
robot = Robot('robot')
robot.at.value = 'p2'
block1.at.value = 'p1'

move_params = {
    'robot': Robot,
    'current_pose': PosEntity,
    'target_pose': PosEntity
}
move_conditions = [
    Condition('robot_at_src', 'robot', 'at', 'current_pose'),
    Condition('robot_not_at_target', 'robot', 'at', 'target_pose', negate=True)
]
move_effects = [
    Effect('move_robot', 'robot', 'at', 'target_pose'),
    Effect('unset_pose_occupied', 'current_pose', 'occupied_by', None),
    Effect('set_pose_occupied', 'target_pose', 'occupied_by', 'robot')
]
move_action = Action('move', move_params, move_conditions, move_effects)
print(f"Robot at: {robot.at.value}")
print(f"Checking robot move action: {move_action.check({'robot': robot, 'current_pose': p1, 'target_pose': p3})}")
print(p1, p3)
move_action.execute({'robot': robot, 'current_pose': p1, 'target_pose': p3})
print(f"Robot new position: {robot.at.value}")
print(p1, p3)