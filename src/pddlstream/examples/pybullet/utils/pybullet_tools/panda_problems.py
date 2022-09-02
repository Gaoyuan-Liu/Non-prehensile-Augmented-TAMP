import numpy as np
from itertools import product

from .pr2_utils import REST_LEFT_ARM, \
      get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF
from .utils import create_box, set_base_values, set_point, set_pose, get_pose, \
    get_bodies, z_rotation, load_model, load_pybullet, HideOutput, create_body, \
    get_box_geometry, get_cylinder_geometry, create_shape_array, unit_pose, Pose, \
    Point, LockRenderer, FLOOR_URDF, TABLE_URDF, BIN_URDF, add_data_path, TAN, set_color, BASE_LINK, remove_body

# For panda    
from examples.pybullet.utils.pybullet_tools.utils import add_data_path, connect, dump_body, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, INF

from .panda_utils import set_arm_conf, open_arm, close_arm, create_gripper

from examples.pybullet.utils.pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF, SAFE_FRANKA_URDF
from examples.pybullet.utils.pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver

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
        # set_group_conf(pr2, 'torso', [torso])
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



def create_table(width=0.6, length=1.2, height=0.73, thickness=0.03, radius=0.015,
                 top_color=LIGHT_GREY, leg_color=TAN, cylinder=True, **kwargs):
    # TODO: table URDF
    surface = get_box_geometry(width, length, thickness)
    surface_pose = Pose(Point(z=height - thickness/2.))

    leg_height = height-thickness
    if cylinder:
        leg_geometry = get_cylinder_geometry(radius, leg_height)
    else:
        leg_geometry = get_box_geometry(width=2*radius, length=2*radius, height=leg_height)
    legs = [leg_geometry for _ in range(4)]
    leg_center = np.array([width, length])/2. - radius*np.ones(2)
    leg_xys = [np.multiply(leg_center, np.array(signs))
               for signs in product([-1, +1], repeat=len(leg_center))]
    leg_poses = [Pose(point=[x, y, leg_height/2.]) for x, y in leg_xys]

    geoms = [surface] + legs
    poses = [surface_pose] + leg_poses
    colors = [top_color] + len(legs)*[leg_color]

    collision_id, visual_id = create_shape_array(geoms, poses, colors)
    body = create_body(collision_id, visual_id, **kwargs)

    # TODO: unable to use several colors
    #for idx, color in enumerate(geoms):
    #    set_color(body, shape_index=idx, color=color)
    return body

def create_door():
    return load_pybullet("data/door.urdf")

def create_bin():
    return load_pybullet(BIN_URDF)

#######################################################

# https://github.com/bulletphysics/bullet3/search?l=XML&q=.urdf&type=&utf8=%E2%9C%93

TABLE_MAX_Z = 0.6265 # TODO: the table legs don't seem to be included for collisions?

def holding_problem(arm='left', grasp_type='side'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    plane = create_floor()
    table = load_pybullet(TABLE_URDF)
    #table = load_pybullet("table_square/table_square.urdf")
    box = create_box(.07, .05, .15)
    set_point(box, (0, 0, TABLE_MAX_Z + .15/2))

    return Problem(robot=pr2, movable=[box], arms=[arm], grasp_types=[grasp_type], surfaces=[table],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, box)])

def stacking_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    plane = create_floor()
    table1 = load_pybullet(TABLE_URDF)
    #table = load_pybullet("table_square/table_square.urdf")

    block = create_box(.07, .05, .15)
    set_point(block, (0, 0, TABLE_MAX_Z + .15/2))

    table2 = load_pybullet(TABLE_URDF)
    set_base_values(table2, (2, 0, 0))

    return Problem(robot=pr2, movable=[block], arms=[arm],
                   grasp_types=[grasp_type], surfaces=[table1, table2],
                   #goal_on=[(block, table1)])
                   goal_on=[(block, table2)])

#######################################################

def create_kitchen(w=.5, h=.7):
    floor = create_floor()

    table = create_box(w, w, h, color=(.75, .75, .75, 1))
    set_point(table, (2, 0, h/2))

    mass = 1
    #mass = 0.01
    #mass = 1e-6
    cabbage = create_box(.07, .07, .1, mass=mass, color=(0, 1, 0, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(cabbage, (2, 0, h + .1/2))

    sink = create_box(w, w, h, color=(.25, .25, .75, 1))
    set_point(sink, (0, 2, h/2))

    stove = create_box(w, w, h, color=(.75, .25, .25, 1))
    set_point(stove, (0, -2, h/2))

    return table, cabbage, sink, stove

#######################################################

def cleaning_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    #door = create_door()
    #set_point(door, (2, 0, 0))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cleaned=[cabbage])

def cooking_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cooked=[cabbage])

def cleaning_button_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    d = 0.1
    sink_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(sink_button, ((0, 2-(.5+d)/2, .7-d/2), z_rotation(np.pi/2)))

    stove_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(stove_button, ((0, -2+(.5+d)/2, .7-d/2), z_rotation(-np.pi/2)))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   buttons=[(sink_button, sink), (stove_button, stove)],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, cabbage)], goal_cleaned=[cabbage])

def cooking_button_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    d = 0.1
    sink_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(sink_button, ((0, 2-(.5+d)/2, .7-d/2), z_rotation(np.pi/2)))

    stove_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_pose(stove_button, ((0, -2+(.5+d)/2, .7-d/2), z_rotation(-np.pi/2)))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   buttons=[(sink_button, sink), (stove_button, stove)],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, cabbage)], goal_cooked=[cabbage])

PROBLEMS = [
    holding_problem,
    stacking_problem,
    cleaning_problem,
    cooking_problem,
    cleaning_button_problem,
    cooking_button_problem,
]
