#!/usr/bin/env python

from __future__ import print_function

from utils.pybullet_tools.panda_primitives import get_gripper_joints, GripperCommand, Attach, Detach
from utils.pybullet_tools.utils import get_max_limit




def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            # print(f"here the arm is {a}")
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, a, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot, a)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
            detach = Detach(problem.robot, a, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands


#######################################################

def main(verbose=True):
    return 0

if __name__ == '__main__':
    main()