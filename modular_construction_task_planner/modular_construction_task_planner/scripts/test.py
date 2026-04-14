from modular_construction_task_planner.scripts.block_domain import *

POSE_VAR_DOMAIN = ("g", "pos1", "pos2", "pos3", "")
BLOCK_VAR_DOMAIN = ("block1", "block2", "block3", "")
BOOL_VAR_DOMAIN = (True, False)
VarDomains = {'pos': POSE_VAR_DOMAIN, 'block': BLOCK_VAR_DOMAIN, 'bool': BOOL_VAR_DOMAIN}

p1 = Pose('p1')
print(p1.state)

block1 = Object('block1')
block1.at.value = 'p1'
print(block1.state)