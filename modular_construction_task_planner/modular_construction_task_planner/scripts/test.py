from modular_construction_task_planner.scripts.block_domain import Pose, Object
from modular_construction_task_planner.eas.core import load_domains, Condition, Effect

POSE_VAR_DOMAIN = ("g", "p1", "p2", "p3", "")
BLOCK_VAR_DOMAIN = ("block1", "block2", "block3", "")
BOOL_VAR_DOMAIN = (True, False)

VarDomains = {'pos': POSE_VAR_DOMAIN, 'block': BLOCK_VAR_DOMAIN, 'bool': BOOL_VAR_DOMAIN}

load_domains(VarDomains)

block1 = Object('block1')
p1 = Pose('p1')
block1.at.value = 'p1'

print(block1.at.value)
cond = Condition('block_at_src', 'at')
print(cond(block1, 'p1'))
eff = Effect('move_block', 'at')
print(eff(block1, 'p2'))
print(block1.at.value)