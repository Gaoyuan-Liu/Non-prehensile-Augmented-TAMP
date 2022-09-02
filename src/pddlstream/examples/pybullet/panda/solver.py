#!/usr/bin/env python

import sys, os

file_path = os.path.dirname(os.path.realpath(__file__))

sys.path.insert(0, file_path + '/../../../')


from examples.pybullet.panda.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_cfree_traj_grasp_pose_test, BASE_CONSTANT, distance_fn, move_cost_fn

from examples.pybullet.utils.pybullet_tools.panda_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, control_commands
from examples.pybullet.utils.pybullet_tools.panda_utils import get_arm_joints, ARM_NAMES, get_group_joints, open_arm, \
    get_group_conf, set_arm_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, get_euler, set_euler, is_placement, disconnect, pairwise_collision,\
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, set_joint_positions, get_movable_joints
from examples.pybullet.namo.stream import get_custom_limits

from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, from_test
from pddlstream.language.constants import Equal, And, Or, print_solution, Exists, get_args, is_parameter, \
    get_parameter_name, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, DEBUG

from examples.pybullet.utils.pybullet_tools.panda_primitives import apply_commands, State
from examples.pybullet.panda.post_process import post_process
from examples.pybullet.utils.pybullet_tools.utils import draw_base_limits, WorldSaver, has_gui, str_from_object, set_pose, \
    create_box, set_point, Point, TAN

# from examples.pybullet.panda.problems_combined import PROBLEMS
from examples.pybullet.utils.pybullet_tools.panda_problems import create_panda, create_table, Problem

# Push
# from examples.pybullet.panda.pushing import push
from examples.pybullet.panda.scenario import Scenario #, reset_obj_solvable
from examples.pybullet.panda.camera import Camera
import pybullet as p
import moveit_commander
import cv2 as cv
import random
import itertools
from pushing import Pusher

class pddlstream_solver():
    def __init__(self, robot, bin):
        self.algorithm = 'adaptive'
        self.verbose = False
        self.stream_info = {
        'inverse-kinematics': StreamInfo(),

        # 'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=self.verbose),
        'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=self.verbose),
        # 'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=self.verbose), # TODO: apply to arm and base trajs
        }
        self.render = False
        self.optimal = False
        self.max_time = 20
        self.initial_complexity = 3 #3
        self.max_complexity = 9
        self.success_cost = 0 if self.optimal else INF
        self.planner = 'ff-astar' if self.optimal else 'ff-wastar3'
        self.search_sample_ratio = 2 #2
        self.max_planner_time = 10 #5
        # effort_weight = 0 if args.optimal else 1
        self.effort_weight = 1e-3 if self.optimal else 1

        self.robot = robot
        self.bin = bin
        self.problem = Problem(robot=self.robot, bin=self.bin)


        # Static objects
        plate_width = 0.2
        plate_length = 0.5
        plate_height = 0.002
        self.plate = create_box(plate_width, plate_length, plate_height, color=TAN)
        set_point(self.plate, Point(x=0.0, y=0.5, z=plate_height/2))



    def problem_formation(self, obj_poses, n_pick=None):
        num = len(obj_poses)
        if n_pick == None:
            n_pick = num

        self.flip_checking(obj_poses)
        # print(f'obj_poses = {obj_poses}')
        floor = 0#load_pybullet("plane.urdf")

    

        # panda = robot #create_panda() # Here panda is a int(1)

        joints = get_movable_joints(self.robot)
        

        # sample_fn = get_sample_fn(panda, joints)
        # initial_conf = sample_fn()
        # Config is (joint1, joint2, ..., joint7, )
        initial_conf = (0.0, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 0.02, 0.02)
        
        
        arm = 'panda'
        # set_arm_conf(self.robot, joints, initial_conf)
        open_arm(self.robot, arm)


        # Block
        # blocks = [create_box(block_width, block_width, block_height, color=GREEN) for _ in range(num)]
        grasp_type = 'top'
        
        # # set_euler(blocks[2], Euler(roll=0, pitch=0, yaw=3.14/2))
        blocks = []
        for i in obj_poses:
            # set_point(i, Point(x=obj_poses[i][0], y=obj_poses[i][1], z=obj_poses[i][2]))
            blocks.append(i)

        # pick_blocks = random.choices(blocks, k=n_pick)
        all_combination_n_pick = list(itertools.combinations(blocks, n_pick))
        
        # plate_width = 0.5
        # plate_length = 0.8
        # plate_height = 0.002
        # plate = create_box(plate_width, plate_length, plate_height, color=TAN)
        # set_point(plate, Point(x=-0.6, y=0.0, z=0))

        surfaces = [floor, self.plate]


        self.problem.robot = self.robot
        self.problem.movable = blocks
        self.problem.bin = self.bin
        print(f'\nbin = {self.problem.bin}\n')
        self.problem.grasp_types = [grasp_type]
        self.problem.surfaces = surfaces
        # self.problem.goal_holding=[(arm, block) for block in blocks]

        self.problem.goal_on = []
        

        # print(f'\n goal_on = {self.problem.goal_on} \n')
        if n_pick == num:
            self.problem.goal_on=[(block, self.plate) for block in blocks]

        else:
            for combination in all_combination_n_pick:
                goal = []
                for i in combination:
                    goal.append(i)
                goal_tuple = tuple(goal + [self.plate])
                self.problem.goal_on.append(goal_tuple)





    def get_bodies_from_type(self, problem):
        bodies_from_type = {}
        for body, ty in problem.body_types:
            bodies_from_type.setdefault(ty, set()).add(body)
        return bodies_from_type

    def pddlstream_from_problem(self, problem, n_pick, collisions=True, teleport=False):
        
        robot = problem.robot
        tool = problem.get_gripper()


        domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
        stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
        constant_map = {
            '@sink': 'sink',
            '@stove': 'stove',
        }


        init = [('CanMove',)]
            
        # for arm in ARM_NAMES:
        #for arm in problem.arms:
        arm = 'panda'
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('Arm', arm), ('AConf', arm, conf), ('HandEmpty', arm), ('AtAConf', arm, conf)]
        # if arm in problem.arms:
        init += [('Controllable', arm)]

        for body in problem.bin:
            pose = Pose(body, get_pose(body), init=True) # TODO: supported here
            init += [('Pose', body, pose),
                    ('AtPose', body, pose)]

        for body in problem.movable:
            pose = Pose(body, get_pose(body), init=True) # TODO: supported here
            init += [('Graspable', body), ('Pose', body, pose),
                    ('AtPose', body, pose), ('Stackable', body, None)]
            for surface in problem.surfaces:
                if is_placement(body, surface):
                    init += [('Supported', body, pose, surface)]




        for body, ty in problem.body_types:
            init += [('Type', body, ty)]



        bodies_from_type = self.get_bodies_from_type(problem)
        goal_literals = []
     
        n_goal_on_param = len(problem.goal_on[0])
        for one_goal in problem.goal_on:
            # on_number = str(n_goal_on_param-1) if n_goal_on_param-1 != 1 else None
            # one_goal_literal = ['On'+str(n_goal_on_param-1)] 
            one_goal_literal = ['On'+str(n_goal_on_param-1)] if n_goal_on_param-1 != 1 else ['On']
            for ty in one_goal[:n_goal_on_param-1]:
                bodies = bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
                init += [('Stackable', b, one_goal[-1]) for b in bodies]
                
                one_goal_literal.append(ty)
            one_goal_literal.append(one_goal[-1])
            goal_literals += [tuple(one_goal_literal)]


                
                  

            # bodies = bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
            # init += [('Stackable', b, s) for b in bodies]
            # goal_literals += [('On', ty, s)]

        # for ty, s in problem.goal_on:
        #     bodies = bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
        #     init += [('Stackable', b, s) for b in bodies]
        #     goal_literals += [('On', ty, s)]
      


        goal_formula = []

        for literal in goal_literals:
            parameters = [a for a in get_args(literal) if is_parameter(a)]
            if parameters:
                type_literals = [('Type', p, get_parameter_name(p)) for p in parameters]
                goal_formula.append(Exists(parameters, And(literal, *type_literals)))
            else:
                goal_formula.append(literal)
        # print(f'\nn_goal_on_param = {n_goal_on_param}\n')
        if n_pick == n_goal_on_param-1:
            goal_formula = Or(*goal_formula) # This mean partial picking
        else:
            goal_formula = And(*goal_formula) # This means total picking



        custom_limits = {}
        # if base_limits is not None:
        #     custom_limits.update(get_custom_limits(robot, problem.base_limits))

        stream_map = {
            # from_gen_fn basically just put the samples into a list.
            'sample-pose': from_gen_fn(get_stable_gen(problem, collisions=collisions)), 
            'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=collisions)),
        
            'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, custom_limits=custom_limits,
                                                            collisions=False, teleport=teleport)),
            # 'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=collisions)),
            'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(problem, collisions=collisions)),
            # 'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(robot, collisions=collisions)),

            # 'Distance': distance_fn,
        }

        
        return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)


 

    def problem_solve(self, obj_poses, n_pick):

      

        with HideOutput():
            self.problem_formation(obj_poses, n_pick)
            
        
        saver = WorldSaver()
        
        pddlstream_problem = self.pddlstream_from_problem(self.problem, n_pick=n_pick ,collisions=True)
        
        


     

        _, _, _, stream_map, init, goal = pddlstream_problem
        print('Init:', init)
        print('Goal:', goal)
        print('Streams:', str_from_object(set(stream_map)))

        


        with Profiler(field='tottime', num=25): # cumtime | tottime
            with LockRenderer(lock=not self.render): # Lock the render
                solution = solve(pddlstream_problem, algorithm=self.algorithm, stream_info=self.stream_info,
                                planner=self.planner, max_planner_time=self.max_planner_time,
                                success_cost=self.success_cost, initial_complexity = self.initial_complexity, max_complexity = self.max_complexity,
                                max_time=self.max_time, verbose=True, debug=False, visualize=True,
                                unit_efforts=True, effort_weight=self.effort_weight,
                                search_sample_ratio=self.search_sample_ratio)
                saver.restore()
                
        
        # Put the ee tool aside
        gripper = self.problem.get_gripper()
        set_pose(gripper, ((10, 10, 0.2), (0,0,0,1)))  
        

         
        return solution

    def execution(self, problem, solution):
        plan, cost, evaluations = solution
        if (plan is None) or not has_gui():
            disconnect()
            
        with LockRenderer(lock=True):
            commands = post_process(problem, plan, teleport=False)
       
            
        
        simulate = False
        if simulate:
            control_commands(commands)
        else:
            time_step = 0.01 #None if args.teleport else 0.01
            apply_commands(State(), commands, time_step)

    def solve_and_execute(self, obj_poses, n_pick):
        solution = self.problem_solve(obj_poses, n_pick)
        plan, _, _ = solution
        
        if plan!=None:
            solved = True
            self.execution(self.problem, solution)
            
        else:
            print('No solution found.')
            solved = False

        return solved

    def flip_checking(self, obj_poses):
        for i in obj_poses:
            eulers = get_euler(i)
            new_eulers = list(eulers)
            if abs(eulers[0]) >= 0.5:
                new_eulers = [0.0, eulers[1], eulers[2]]
                
            if abs(eulers[1]) >= 0.5:
                new_eulers = [eulers[0], 0.0, eulers[2]]
            set_euler(i, new_eulers)
            






    

if __name__ == '__main__':
    ##########################
    #       Start Sim        #
    ##########################
    connect(use_gui=True)
    # p.connect(p.GUI)
    scenario = Scenario()
    robot, obj_poses = scenario.scenario(4)
    bin = scenario.bin
    obj_poses = scenario.reset_obj_unsolvable(obj_poses)

    print(f'\nThe obj_poses in solver is\n{obj_poses} \n')

    wait_for_user()

    ##########################
    #       Here Pushes      #
    ##########################
    # camera = camera()
    # img1 = camera.observe()
    pusher = Pusher()
    move_group = moveit_commander.MoveGroupCommander("panda_arm")

    # -------------
    # Deadend push
    # -------------

    pusher.push(robot, obj_poses, [0.5, 0.2], [0.5, -0.08])
    pusher.push(robot, obj_poses, [0.6, -0.17], [0.5, -0.17])

    # ---------------
    # Collision push
    # ---------------
    obj_poses = scenario.reset_obj_unsolvable(obj_poses)
    scenario.reset_bin()
    wait_for_user()
    pusher.push(robot, obj_poses, [0.47, 0.2], [0.47, -0.12])

    # -------------
    # Untouch push
    # -------------
    obj_poses = scenario.reset_obj_unsolvable(obj_poses)
    scenario.reset_bin()
    wait_for_user()
    pusher.push(robot, obj_poses, [0.5, 0.15], [0.4, 0.15])



    # img2 = camera.observe()
    # ans = camera.compare_images(img1, img2)

    # cv.imshow('img1', img1)
    # cv.imshow('img2', img2)
    # cv.waitKey(0)


    # print(ans)


    ##########################
    # Here build the problem #
    ##########################

    # solver = pddlstream_solver(robot,bin)
    # solver.solve_and_execute(obj_poses, 5)
    


    ##########################
    #        Execution       #
    ##########################
    # solver.execution(problem, solution)
    wait_for_user()

    ##########################
    #       Push again       #
    ##########################
    # obj_poses = push(robot, obj_poses, [0.6, -0.3], [0.6, 0])