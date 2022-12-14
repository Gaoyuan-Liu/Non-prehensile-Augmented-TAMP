#!/usr/bin/env python

from __future__ import print_function

from examples.pybullet.panda.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_cfree_traj_grasp_pose_test, BASE_CONSTANT, distance_fn, move_cost_fn

from examples.pybullet.utils.pybullet_tools.panda_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, control_commands
from examples.pybullet.utils.pybullet_tools.panda_utils import get_arm_joints, ARM_NAMES, get_group_joints, \
    get_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, pairwise_collision,\
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, set_joint_positions
from examples.pybullet.namo.stream import get_custom_limits

from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, from_test
from pddlstream.language.constants import Equal, And, print_solution, Exists, get_args, is_parameter, \
    get_parameter_name, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, DEBUG

from examples.pybullet.utils.pybullet_tools.panda_primitives import apply_commands, State
from examples.pybullet.panda.post_process import post_process
from examples.pybullet.utils.pybullet_tools.utils import draw_base_limits, WorldSaver, has_gui, str_from_object

from examples.pybullet.panda.problems import PROBLEMS





# TODO: collapse similar streams into a single stream when reodering

def get_bodies_from_type(problem):
    bodies_from_type = {}
    for body, ty in problem.body_types:
        bodies_from_type.setdefault(ty, set()).add(body)
    return bodies_from_type

def pddlstream_from_problem(problem, base_limits=None, collisions=True, teleport=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {
        '@sink': 'sink',
        '@stove': 'stove',
    }

    #initial_bq = Pose(robot, get_pose(robot))
    # initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    init = [
        ('CanMove',),
        # ('BConf', initial_bq),
        # ('AtBConf', initial_bq),
        Equal(('PickCost',), 1),
        Equal(('PlaceCost',), 1),
    ] + [('Sink', s) for s in problem.sinks] + \
           [('Stove', s) for s in problem.stoves] + \
           [('Connected', b, d) for b, d in problem.buttons] + \
           [('Button', b) for b, _ in problem.buttons]
    # for arm in ARM_NAMES:
    #for arm in problem.arms:
    arm = 'panda'
    joints = get_arm_joints(robot, arm)
    conf = Conf(robot, joints, get_joint_positions(robot, joints))
    init += [('Arm', arm), ('AConf', arm, conf), ('HandEmpty', arm), ('AtAConf', arm, conf)]
    # if arm in problem.arms:
    init += [('Controllable', arm)]

    for body in problem.movable:
        pose = Pose(body, get_pose(body), init=True) # TODO: supported here
        init += [('Graspable', body), ('Pose', body, pose),
                 ('AtPose', body, pose), ('Stackable', body, None)]
        for surface in problem.surfaces:
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]
    for body, ty in problem.body_types:
        init += [('Type', body, ty)]

    bodies_from_type = get_bodies_from_type(problem)
    goal_literals = []
    if problem.goal_conf is not None:
        goal_conf = Conf(robot, get_group_joints(robot, 'base'), problem.goal_conf)
        init += [('BConf', goal_conf)]
        goal_literals += [('AtBConf', goal_conf)]
    for ty, s in problem.goal_on:
        bodies = bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
        init += [('Stackable', b, s) for b in bodies]
        goal_literals += [('On', ty, s)]
    goal_literals += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked]
    goal_formula = []
    for literal in goal_literals:
        parameters = [a for a in get_args(literal) if is_parameter(a)]
        if parameters:
            type_literals = [('Type', p, get_parameter_name(p)) for p in parameters]
            goal_formula.append(Exists(parameters, And(literal, *type_literals)))
        else:
            goal_formula.append(literal)
    goal_formula = And(*goal_formula)

    custom_limits = {}
    if base_limits is not None:
        custom_limits.update(get_custom_limits(robot, problem.base_limits))

    stream_map = {
        # from_gen_fn basically just put the samples into a list.
        'sample-pose': from_gen_fn(get_stable_gen(problem, collisions=collisions)), 
        'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=collisions)),
    
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, custom_limits=custom_limits,
                                                        collisions=collisions, teleport=teleport)),
        

        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=collisions)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(problem, collisions=collisions)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(robot, collisions=collisions)),
        

        
        'Distance': distance_fn,
    }
    #stream_map = DEBUG

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)

#######################################################

def main(verbose=True):
    # TODO: could work just on postprocessing
    # TODO: try the other reachability database
    # TODO: option to only consider costs during local optimization

    parser = create_parser()
    parser.add_argument('-problem', default='packed', help='The name of the problem to solve')
    parser.add_argument('-n', '--number', default=1, type=int, help='The number of objects')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=120, type=int, help='The max time')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    parser.add_argument('-viewer', action='store_true', help='Enable the viewer and visualizes the plan')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_fn_from_name:
        raise ValueError(args.problem)
    problem_fn = problem_fn_from_name[args.problem]

    connect(use_gui=args.viewer)
    with HideOutput():
        problem = problem_fn(num=args.number)
    # draw_base_limits(problem.base_limits, color=(1, 0, 0))
    saver = WorldSaver()

    #handles = []
    #for link in get_group_joints(problem.robot, 'left_arm'):
    #    handles.append(draw_link_name(problem.robot, link))
    #wait_for_user()

    pddlstream_problem = pddlstream_from_problem(problem, collisions=not args.cfree, teleport=args.teleport)
    stream_info = {
        'inverse-kinematics': StreamInfo(),
        # 'plan-base-motion': StreamInfo(overhead=1e1),

        'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=verbose),
        'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=verbose),
        'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=verbose), # TODO: apply to arm and base trajs
        #'test-cfree-traj-grasp-pose': StreamInfo(verbose=verbose),

        'Distance': FunctionInfo(p_success=0.99, opt_fn=lambda q1, q2: BASE_CONSTANT),
        #'MoveCost': FunctionInfo(lambda t: BASE_CONSTANT),
    }
    #stream_info = {}

    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', str_from_object(set(stream_map)))


    # from examples.pybullet.utils.pybullet_tools.ikfast.utils import IKFastInfo
    # from examples.pybullet.utils.pybullet_tools.utils import link_from_name
    # from examples.pybullet.utils.pybullet_tools.ikfast.ikfast import get_ik_joints

    # PANDA_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
    #                     ee_link='panda_link8', free_joints=['panda_joint7'])
    # tool_link = link_from_name(problem.robot, 'panda_hand')
    # ik_joints = get_ik_joints(problem.robot, PANDA_INFO, tool_link)
    # conf = (2.369797298171423, -1.3632637481970176, -2.7285610952295007, -1.995832148337751, 1.1371716329641703, 0.4476344536474688, -0.5781410331698456)
    # set_joint_positions(problem.robot, ik_joints, conf)

    # obstacles = problem.fixed
    # print(obstacles)
    # print(f"collision = {pairwise_collision(problem.robot, obstacles[0])}")
    # print(f"collision = {pairwise_collision(problem.robot, obstacles[1])}")
    # print(f"collision = {pairwise_collision(problem.robot, obstacles[2])}")

    # Test the stream
    # a = stream_map['sample-grasp']
    # b = a(problem.movable[1])
    # print(next(b))
    wait_for_user()

    success_cost = 0 if args.optimal else INF
    planner = 'ff-astar' if args.optimal else 'ff-wastar3'
    search_sample_ratio = 2 #2
    max_planner_time = 10
    # effort_weight = 0 if args.optimal else 1
    effort_weight = 1e-3 if args.optimal else 1

    with Profiler(field='tottime', num=25): # cumtime | tottime
        with LockRenderer(lock=not args.enable):
            solution = solve(pddlstream_problem, algorithm=args.algorithm, stream_info=stream_info,
                             planner=planner, max_planner_time=max_planner_time,
                             unit_costs=args.unit, success_cost=success_cost,
                             max_time=args.max_time, verbose=True, debug=False,
                             unit_efforts=True, effort_weight=effort_weight,
                             search_sample_ratio=search_sample_ratio)
            saver.restore()


    cost_over_time = [(s.cost, s.time) for s in SOLUTIONS]
    for i, (cost, runtime) in enumerate(cost_over_time):
        print('Plan: {} | Cost: {:.3f} | Time: {:.3f}'.format(i, cost, runtime))
    #print(SOLUTIONS)
    print_solution(solution)
    plan, cost, evaluations = solution
    if (plan is None) or not has_gui():
        disconnect()
        return

    with LockRenderer(lock=not args.enable):
        commands = post_process(problem, plan, teleport=args.teleport)
        saver.restore()

 
    wait_for_user()

    print(f'commands = {commands}, type = {commands[0].path[0].values}.')
    print(f'{type(repr(commands[0]))}')
    if args.simulate:
        control_commands(commands)
    else:
        time_step = None if args.teleport else 0.01
        apply_commands(State(), commands, time_step)
    wait_for_user()
    disconnect()

    



if __name__ == '__main__':
    main()