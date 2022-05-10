import rospy
import threading
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage

import time
from copy import deepcopy
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
        self.params = Sl1mParameters()
        self.params.get_ros_param()
        rospy.loginfo(str(self.params))

        # Handle the update of the input
        self.initial_contacts_mutex = threading.Lock()
        self.initial_contacts = [np.zeros(3), np.zeros(3)]
        self.all_polygons_mutex = threading.Lock()
        self.all_polygons = []

        self.polygon_client = rospy.Subscriber(
            "/sl1m_wrapper_node/polygons", MarkerArray, self.polygon_callback
        )
        self.current_contact = rospy.Subscriber(
            "/sl1m_wrapper_node/current_contact",
            TFMessage,
            self.current_contact_callback,
        )

        # This is valid because we loaded the parameter upon starting the node.
        self.problem = Problem(
            limb_names=self.params.limbs,
            constraint_paths=self.params.paths,
            suffix_com=self.params.suffix_com,
            suffix_feet=self.params.suffix_feet,
        )

    def current_contact_callback(self, msg):
        rospy.loginfo(
            "New contact list. Number of contact : {}".format(
                len(msg.transforms)
            )
        )
        assert len(self.params.limbs) == len(msg.transforms)
        self.initial_contacts_mutex.acquire()
        for i in range(len(self.initial_contacts)):
            self.initial_contacts[i] = [
                msg.transforms[i].transform.translation.x,
                msg.transforms[i].transform.translation.y,
                msg.transforms[i].transform.translation.z,
            ]
        rospy.loginfo("Current contacts: {}".format(self.initial_contacts))
        self.initial_contacts_mutex.release()

    def polygon_callback(self, msg):
        rospy.loginfo(
            "New polygon list. Number of surfaces : {}".format(len(msg.markers))
        )

        # tmp var
        all_polygons = []

        # Getting the list of surfaces
        for surface in msg.markers:
            point_list = []
            for point in surface.points:
                point_list.append([point.x, point.y, point.z])
            all_polygons.append(np.array(point_list).T)

        self.all_polygons_mutex.acquire()
        self.all_polygons = all_polygons
        self.all_polygons_mutex.release()

    def plot_if_results(self):
        if self.params.plot:
            ax = plot.draw_scene(self.surfaces, self.params.gait)
            plot.plot_initial_contacts(self.initial_contacts, ax=ax)
            if self.result.success:
                plot.plot_planner_result(
                    self.result.all_feet_pos,
                    coms=self.result.coms,
                    ax=ax,
                    show=True,
                )
            else:
                plt.show()

    def run(self):
        all_polygons = deepcopy(self.all_polygons)
        initial_contacts = deepcopy(self.initial_contacts)

        r = rospy.Rate(self.params.rate)
        while not rospy.is_shutdown():
            # Get the time at the beginning of the loop
            t_start = clock()

            rospy.loginfo("Wait for valid inputs...")
            no_valid_input = True
            while no_valid_input:
                # Work with local copies
                self.initial_contacts_mutex.acquire()
                all_polygons = deepcopy(self.all_polygons)
                self.initial_contacts_mutex.release()

                self.initial_contacts_mutex.acquire()
                initial_contacts = deepcopy(self.initial_contacts)
                self.initial_contacts_mutex.release()

                if all_polygons == [] and np.all(
                    np.array(initial_contacts)
                    == np.array([np.zeros(3), np.zeros(3)])
                ):
                    time.sleep(0.05)
                else:
                    no_valid_input = False

            rospy.loginfo("Input acquired.")
            rospy.loginfo("all_polygons: {}".format(all_polygons))
            rospy.loginfo("initial_contacts: {}".format(initial_contacts))

            t_acquiring_data = clock()

            # Getting the list of surfaces
            surfaces = self.params.nb_steps * [[all_polygons]]
            R = [np.identity(3)] * self.params.nb_steps

            t_surface = clock()

            self.problem.generate_problem(
                R, surfaces, self.params.gait, initial_contacts
            )

            t_problem = clock()

            if self.params.solver_method == "L1":
                result = solve_L1_combinatorial(
                    self.problem,
                    costs=self.params.cost,
                    com=self.params.optimize_com,
                    solver=self.params.get_solver_type(),
                )
            else:
                result = solve_MIP(
                    self.problem,
                    costs=self.params.cost,
                    com=self.params.optimize_com,
                    solver=self.params.get_solver_type(),
                )

            t_end = clock()

            rospy.loginfo(
                "Optimized number of steps: {}".format(self.problem.n_phases)
            )
            rospy.loginfo(
                "Total time is: {}".format(1000.0 * (t_end - t_start))
            )
            rospy.loginfo(
                "Acquiring the data takes: {}".format(
                    1000.0 * (t_acquiring_data - t_start)
                )
            )
            rospy.loginfo(
                "Computing the surfaces takes : {}".format(
                    1000.0 * (t_surface - t_acquiring_data)
                )
            )
            rospy.loginfo(
                "Generating the problem dictionary takes: {}".format(
                    1000.0 * (t_problem - t_surface)
                )
            )
            rospy.loginfo(
                "Solving the problem takes: {}".format(
                    1000.0 * (t_end - t_problem)
                )
            )
            rospy.loginfo(
                "The LP and QP optimizations take: {}".format(result.time)
            )
            self.surfaces = surfaces
            self.result = result
            self.initial_contacts = initial_contacts
            self.plot_if_results()
            r.sleep()
