import numpy as np
from itertools import product


from .utils import create_box, set_base_values, set_point, set_pose, get_pose, \
    get_bodies, z_rotation, load_model, load_pybullet, HideOutput, create_body, \
    get_box_geometry, get_cylinder_geometry, create_shape_array, unit_pose, Pose, \
    Point, LockRenderer, FLOOR_URDF, TABLE_URDF, BIN_URDF, add_data_path, TAN, set_color, BASE_LINK, remove_body

# For panda    
from .utils import add_data_path, connect, dump_body, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, INF

from .panda_utils import set_arm_conf, open_arm, close_arm, create_gripper

from .ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF, SAFE_FRANKA_URDF
from .ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver

LIGHT_GREY = (0.7, 0.7, 0.7, 1.)

class Problem(object):
    def __init__(self, robot, bin=tuple(), movable=tuple(), grasp_types=tuple(),
                 surfaces=tuple(), sinks=tuple(), stoves=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(), goal_on=tuple(),
                 goal_cleaned=tuple(), goal_cooked=tuple(), costs=False,
                 body_names={}, body_types=[], base_limits=None):
        self.robot = robot
        # self.arms = arms
        self.bin = bin
        self.movable = movable
        self.grasp_types = grasp_types
        self.surfaces = surfaces
        self.sinks = sinks
        self.stoves = stoves
        self.buttons = buttons
        self.goal_conf = goal_conf
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cleaned = goal_cleaned
        self.goal_cooked = goal_cooked
        self.costs = costs
        self.body_names = body_names
        self.body_types = body_types
        self.base_limits = base_limits
        all_movable = [self.robot] + list(self.movable)
        self.fixed = list(filter(lambda b: b not in all_movable, get_bodies()))
        self.gripper = None
    def get_gripper(self, visual=True):
        # upper = get_max_limit(problem.robot, get_gripper_joints(problem.robot, 'left')[0])
        # set_configuration(gripper, [0]*4)
        # dump_body(gripper)
        if self.gripper is None:
            self.gripper = create_gripper(self.robot, visual=visual)
        return self.gripper
    def remove_gripper(self):
        if self.gripper is not None:
            remove_body(self.gripper)
            self.gripper = None
    def __repr__(self):
        return repr(self.__dict__)

#######################################################

def get_fixed_bodies(problem): # TODO: move to problem?
    return problem.fixed

def create_panda(fixed_base=True, torso=0.2):
    panda_path = FRANKA_URDF 
    # with LockRenderer():
        # with HideOutput():
    panda = load_model(panda_path, fixed_base=fixed_base)
    assign_link_colors(panda, max_colors=2, s=0.5, v=1.) 
    return panda

def create_safe_panda(fixed_base=True, torso=0.2):
    panda_path = SAFE_FRANKA_URDF 
    # with LockRenderer():
        # with HideOutput():
    panda = load_model(panda_path, fixed_base=fixed_base)
    assign_link_colors(panda, max_colors=2, s=0.5, v=1.)
        # set_group_conf(pr2, 'torso', [torso])
    return panda

def create_floor(**kwargs):
    add_data_path()
    return load_pybullet(FLOOR_URDF, **kwargs)

