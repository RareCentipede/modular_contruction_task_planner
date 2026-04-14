from modular_construction_task_planner.scripts.block_domain import PosEntity, Object, Robot, pick_action, place_action, move_action
from modular_construction_task_planner.eas.core import load_domains, Condition, Effect, Action

POSE_VAR_DOMAIN = ("g", "p1", "p2", "p3", "p4", "p5", "")
BLOCK_VAR_DOMAIN = ("robot", "block1", "block2", "block3", "")
BOOL_VAR_DOMAIN = (True, False)

VarDomains = {'pos': POSE_VAR_DOMAIN, 'block': BLOCK_VAR_DOMAIN, 'bool': BOOL_VAR_DOMAIN}

load_domains(VarDomains)

block1 = Object('block1')
block2 = Object('block2')
block3 = Object('block3')
robot = Robot('robot')
g = PosEntity('g')
p1 = PosEntity('p1')
p2 = PosEntity('p2')
p3 = PosEntity('p3')
p4 = PosEntity('p4')
p5 = PosEntity('p5')

block1.at.value = 'p1'
p1.occupied_by.value = 'block1'
block2.at.value = 'p3'
p2.occupied_by.value = 'block2'
block3.at.value = 'p5'
p5.occupied_by.value = 'block3'

robot.at.value = 'p2'
p2.occupied_by.value = 'robot'

# Print initial state
print("Initial State:")
for entity in [block1, block2, block3, robot, g, p1, p2, p3, p4, p5]:
    print(f"  {entity}")

# Move from p2 to p3
move_params = {'robot': robot, 'start_pose': p2, 'target_pose': p3}
# move_action.check(move_params)
move_action.execute(move_params)
print("\nAfter moving robot from p2 to p3:")
print(f"  {robot}")
print(f"  {p2}")
print(f"  {p3}")

# Pick block2 at p3
pick_params = {'robot': robot, 'object': block2, 'object_pose': p3}
pick_action.check(pick_params)
pick_action.execute(pick_params)
print("\nAfter picking block2 at p3:")
for entity in [block2, robot]:
    print(f"  {entity}")

# Move from p3 to p4
move_params = {'robot': robot, 'start_pose': p3, 'target_pose': p4}
move_action.check(move_params)
move_action.execute(move_params)
print("\nAfter moving robot from p3 to p4:")
for entity in [robot, p3, p4]:
    print(f"  {entity}")

# Place block2 at p4
place_params = {'robot': robot, 'object': block2, 'target_pose': p4}
place_action.check(place_params)
place_action.execute(place_params)
print("\nAfter placing block2 at p4:")
for entity in [block2, robot, p4]:
    print(f"  {entity}")