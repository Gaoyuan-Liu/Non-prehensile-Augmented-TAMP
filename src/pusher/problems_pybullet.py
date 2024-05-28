from __future__ import print_function

import numpy as np
import time

from utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, get_movable_joints, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user, set_euler, \
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY


# for panda
from utils.pybullet_tools.panda_problems import create_panda, Problem
from utils.pybullet_tools.panda_utils import set_arm_conf, open_arm
   


def problem_fn():
    panda = create_panda()
    return panda

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)

    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface[0]) # Only sample on the table
            # break
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in ([body] + surface)):
                obstacles.append(body)
                break
    return True

def packed(grasp_type='top', num=1):
    # Show the pybullet window
    # connect(use_gui=True)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    print(f"No. floor = {floor}")

    panda = create_panda() # Here panda is a int(1)

    joints = get_movable_joints(panda)
    

    # sample_fn = get_sample_fn(panda, joints)
    # initial_conf = sample_fn()
    # Config is (joint1, joint2, ..., joint7, )
    initial_conf = (0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.02, 0.02)
    print(initial_conf)
    
    arm = 'panda'
    set_arm_conf(panda, joints, initial_conf)
    open_arm(panda, arm)
    
    # Table
    # table_width = 2
    # table_height = 0.001
    # table = create_box(table_width, table_width, table_height, color=BROWN)
    # set_point(table, Point(x=0.0, y=0.0, z=-table_height/2))
    # print(f"No. table = {table}")

    # Block
    block_width = 0.05
    block_height = 0.05
    num = num
    blocks = [create_box(block_width, block_width, block_height, color=GREEN) for _ in range(num)]
    grasp_type = grasp_type
    # min_distances = {block: 0.05 for block in blocks}
    # initial_surfaces = {block: [table, floor] for block in blocks}
    # sample_placements(initial_surfaces, min_distances=min_distances)
    # set_point(blocks[0], Point(x=0.6, y=0.0, z=block_height/2 ))
    # set_point(blocks[1], Point(x=0.6, y=0.06, z=block_height/2 ))
    # set_point(blocks[2], Point(x=0.6, y=-0.06, z=block_height/2))
    # # set_point(blocks[2], Point(x=0.6-0.06, y=0.0, z=block_height/2 ))
    # # set_euler(blocks[2], Euler(roll=0, pitch=0, yaw=3.14/2))
    # set_point(blocks[3], Point(x=0.6+0.06, y=0.0, z=block_height/2))

    set_point(blocks[0], Point(x=0.6, y=0.0, z=block_height/2 ))
    set_point(blocks[2], Point(x=0.54, y=0.0, z=block_height/2 ))
    # set_euler(blocks[2], Euler(roll=0, pitch=0, yaw=3.14/2))
    set_point(blocks[1], Point(x=0.6, y=-0.06, z=block_height/2 ))

    # Plate data
    plate_width = 0.5
    plate_length = 0.8
    plate_height = 0.002
    plate = create_box(plate_width, plate_length, plate_height, color=TAN)
    set_point(plate, Point(x=-0.6, y=0.0, z=0))

    surfaces = [floor, plate]


    return Problem(robot=panda, movable=blocks, grasp_types=[grasp_type], surfaces=surfaces,
                   #goal_holding=[(arm, block) for block in blocks])
                   goal_on=[(block, plate) for block in blocks])





PROBLEMS = [
    packed,
]

if __name__ == '__main__':
    packed()