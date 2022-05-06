import rospy
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage

import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter as clock

from sl1m.generic_solver import solve_L1_combinatorial, solve_MIP
from sl1m.problem_definition import Problem
import sl1m.tools.plot_tools as plot

from sl1m_ros.sl1m_params import Sl1mParameters

# TODO remove me
from sl1m_ros.sl1m_params_talos import Sl1mParametersTalos


class Sl1mNode:

    def __init__(self):
        self.new_results = False

        self.params = Sl1mParameters()
        self.params.get_ros_param()
        print(self.params)
        self.params = Sl1mParametersTalos()
        print(self.params)

        self.polygon_client = rospy.Subscriber("/sl1m_wrapper_node/polygons",
                                               MarkerArray, self.polygon_callback)
        self.current_contact = rospy.Subscriber("/sl1m_wrapper_node/current_contact",
                                                TFMessage, self.current_contact_callback)

    def current_contact_callback(self, msg):
        pass

    def polygon_callback(self, msg):
        # Variables declaration
        surfaces = []
        allPlan = []

        rospy.loginfo('New message')
        rospy.loginfo('Number of surfaces : {}'.format(len(msg.markers)))

        t_init = clock()

        # Getting the list of surfaces
        for surface in msg.markers:
            point_list = []
            for point in surface.points:
                point_list.append([point.x, point.y, point.z])
            allPlan.append(np.array(point_list).T)

        surfaces = self.params.nb_steps * [[allPlan]]

        print(surfaces)

        # Using solver
        R = [np.identity(3)] * self.params.nb_steps
        initial_contacts = [
            np.array([-2,  0.1,  0]), np.array([-2,  -0.1,  0])]

        t_surface = clock()

        pb = Problem(limb_names=self.params.limbs, constraint_paths=self.params.paths,
                     suffix_com=self.params.suffix_com, suffix_feet=self.params.suffix_feet)
        pb.generate_problem(R, surfaces, self.params.gait, initial_contacts)

        t_problem = clock()

        result = solve_MIP(pb, costs=self.params.cost,
                           com=self.params.optimize_com,
                           solver=self.params.get_solver_type())
        t_end = clock()
        print(result)

        print("Optimized number of steps:              ", pb.n_phases)
        print("Total time is:                          ", 1000. * (t_end-t_init))
        print("Computing the surfaces takes            ",
              1000. * (t_surface - t_init))
        print("Generating the problem dictionary takes ",
              1000. * (t_problem - t_surface))
        print("Solving the problem takes               ",
              1000. * (t_end - t_problem))
        print("The LP and QP optimizations take        ", result.time)

        self.surfaces = surfaces
        self.result = result
        self.initial_contacts = initial_contacts
        self.new_results = True

    def plot_if_results(self):
        if self.new_results and self.params.plot:
            self.new_results = False

            ax = plot.draw_scene(self.surfaces, self.params.gait)
            plot.plot_initial_contacts(self.initial_contacts, ax=ax)
            if(self.result.success):
                plot.plot_planner_result(
                    self.result.all_feet_pos, coms=self.result.coms, ax=ax, show=True)
            else:
                plt.show()
