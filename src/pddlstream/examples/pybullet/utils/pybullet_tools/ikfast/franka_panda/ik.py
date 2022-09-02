from ..utils import IKFastInfo, get_ik_limits, compute_forward_kinematics, compute_inverse_kinematics, select_solution, \
    USE_ALL, USE_CURRENT
from ..ikfast import * # For legacy purposes

from ...panda_utils import get_gripper_link, get_arm_joints

from ...utils import pairwise_collision, sub_inverse_kinematics, get_movable_joints, multiply, get_link_pose, link_from_name, get_joint_positions, \
    joint_from_name, invert, get_custom_limits, all_between, sub_inverse_kinematics, set_joint_positions, \
    get_joint_positions

    



# TODO: deprecate this file
#FRANKA_URDF = "models/franka_description/robots/panda_arm.urdf"
#FRANKA_URDF = "models/franka_description/robots/hand.urdf"
#FRANKA_URDF = "examples/pybullet/utils/models/franka_description/robots/panda_arm_hand.urdf"

FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"
SAFE_FRANKA_URDF = "models/franka_description/robots/panda_arm_hand_safe.urdf"

PANDA_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
                        ee_link='panda_link8', free_joints=['panda_joint7'])


# For the IK

def is_ik_compiled():
    try:
        from . import ikfast_panda_arm 
        return True
    except ImportError:
        return False

def get_ik_generator(robot, arm, ik_pose, custom_limits={}):
    # from .ikLeft import leftIK
    # from .ikRight import rightIK
    # arm_ik = {'left': leftIK, 'right': rightIK}
    # world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    # base_from_ik = multiply(invert(world_from_base), ik_pose)
    # sampled_joints = [joint_from_name(robot, name) for name in [TORSO_JOINT, UPPER_JOINT[arm]]]
    # sampled_limits = [get_ik_limits(robot, joint, limits) for joint, limits in zip(sampled_joints, [torso_limits, upper_limits])]

    
    tool_pose = ik_pose
    info = PANDA_INFO
    tool_link = link_from_name(robot, 'panda_hand')
    arm_joints = get_ik_joints(robot, info, tool_link)

    min_limits, max_limits = get_custom_limits(robot, arm_joints, custom_limits)
    # print(f"min_limits = {min_limits}")
    # print(f"max_limits = {max_limits}")

    while True:
        # sampled_values = [random.uniform(*limits) for limits in sampled_limits]
        confs = either_inverse_kinematics(robot, info, tool_link, tool_pose, use_pybullet=False,
                                          max_distance=INF, max_time=0.1, max_candidates=INF)#compute_inverse_kinematics(arm_ik[arm], base_from_ik, sampled_values)
        solutions = [q for q in confs if all_between(min_limits, q, max_limits)]
        # TODO: return just the closest solution
        #print(len(confs), len(solutions))
        yield solutions
        if len(solutions) == 0:
            break
        # if all(lower == upper for lower, upper in sampled_limits):
        #     break

# def get_tool_from_ik(robot, arm):
#     # TODO: change PR2_TOOL_FRAMES[arm] to be IK_LINK[arm]
#     world_from_tool = get_link_pose(robot, link_from_name(robot, PR2_TOOL_FRAMES[arm]))
#     world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME[arm]))
#     return multiply(invert(world_from_tool), world_from_ik)

def sample_tool_ik(robot, arm, tool_pose, nearby_conf=USE_ALL, max_attempts=25, **kwargs):
    # print(f"tool_pose = {tool_pose}")
    ik_pose = tool_pose # multiply(tool_pose, get_tool_from_ik(robot, arm))
    generator = get_ik_generator(robot, arm, ik_pose, **kwargs)
    info = PANDA_INFO
    tool_link = link_from_name(robot, 'panda_hand')
    arm_joints = get_ik_joints(robot, info, tool_link) #get_torso_arm_joints(robot, arm)
    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            # TODO: sort by distance from the current solution when attempting?
            if solutions:
                return select_solution(robot, arm_joints, solutions, nearby_conf=nearby_conf)
        except StopIteration:
            break
    return None

def panda_inverse_kinematics(robot, arm, gripper_pose, obstacles=[], custom_limits={}, **kwargs):
    arm_link = get_gripper_link(robot, arm)
    arm_joints = get_arm_joints(robot, arm)
    if is_ik_compiled():
        info = PANDA_INFO
        tool_link = link_from_name(robot, 'panda_hand')
        ik_joints = get_ik_joints(robot, info, tool_link) # get_torso_arm_joints(robot, arm)
        arm_conf = sample_tool_ik(robot, arm, gripper_pose, custom_limits=custom_limits, **kwargs)
        if arm_conf is None:
            return None
        set_joint_positions(robot, ik_joints, arm_conf)
    else:
        arm_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, gripper_pose, custom_limits=custom_limits)
        if arm_conf is None:
            return None
          
    if any(pairwise_collision(robot, b) for b in obstacles):
        return None
    # print(f"arm_joints = {arm_joints}")   
    # ans = get_joint_positions(robot, arm_joints)
    # print(f"ans = {ans}")
    return get_joint_positions(robot, arm_joints)


def panda_inverse_kinematics_background(robot, arm, gripper_pose, obstacles=[], custom_limits={}, **kwargs):
    arm_link = get_gripper_link(robot, arm)
    arm_joints = get_arm_joints(robot, arm)
    if is_ik_compiled():
        info = PANDA_INFO
        tool_link = link_from_name(robot, 'panda_hand')
        ik_joints = get_ik_joints(robot, info, tool_link) # get_torso_arm_joints(robot, arm)
        arm_conf = sample_tool_ik(robot, arm, gripper_pose, custom_limits=custom_limits, **kwargs)
        if arm_conf is None:
            return None
        print(f"arm_conf = {arm_conf}")
        # set_joint_positions(robot, ik_joints, arm_conf)
    else:
        arm_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, gripper_pose, custom_limits=custom_limits)
        if arm_conf is None:
            return None
          
    if any(pairwise_collision(robot, b) for b in obstacles):
        return None
    # print(f"arm_joints = {arm_joints}")   
    # ans = get_joint_positions(robot, arm_joints)
    # print(f"ans = {ans}")
    return arm_conf #get_joint_positions(robot, arm_joints)
