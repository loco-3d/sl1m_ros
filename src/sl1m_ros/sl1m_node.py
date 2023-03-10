# standard imports
import colorsys
import threading
import time
from copy import deepcopy
import numpy as np
from pinocchio import Quaternion
from pinocchio.rpy import matrixToRpy

# imports from ROS
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage

# local debug information
import matplotlib.pyplot as plt

plt.ion()
try:
    from time import perf_counter as clock
except:
    from time import clock
import sl1m.tools.plot_tools as sl1m_plot

# local imports.
from sl1m.generic_solver import solve_L1_combinatorial, solve_MIP
from sl1m.problem_definition import Problem
from sl1m_ros.sl1m_params import Sl1mParameters
from sl1m_ros.average_quaternion import average_quaternion


class Sl1mNode:
    def __init__(self):
        self.params = Sl1mParameters()
        self.params.get_ros_param()
        rospy.loginfo(str(self.params))

        # Handle the update of the input
        self.initial_contacts_mutex = threading.Lock()
        self.initial_contacts = [np.zeros(3) for _ in self.params.limbs]
        self.initial_orientations = [np.zeros(4) for _ in self.params.limbs]
        self.initial_orientation = Quaternion()
        self.initial_contacts_frame_id = ""
        self.destination_contacts_mutex = threading.Lock()
        self.destination_contacts = [np.zeros(3) for _ in self.params.limbs]
        self.destination_orientations = [np.zeros(4) for _ in self.params.limbs]
        self.destination_orientation = Quaternion()
        self.destination_contacts_frame_id = ""
        self.all_polygons_mutex = threading.Lock()
        self.all_polygons = []
        self.all_polygons_frame_id = ""

        self.polygon_client = rospy.Subscriber(
            "/sl1m_ros/polygons",
            MarkerArray,
            self.polygon_callback,
            queue_size=5,
        )
        self.current_contact = rospy.Subscriber(
            "/sl1m_ros/initial_contact",
            TFMessage,
            self.current_contact_callback,
            queue_size=5,
        )
        self.destination_contact = rospy.Subscriber(
            "/sl1m_ros/destination_contact",
            TFMessage,
            self.destination_contact_callback,
            queue_size=5,
        )
        self.result_publisher = rospy.Publisher(
            "/sl1m_ros/solution",
            MarkerArray,
            queue_size=5,
        )

        # This is valid because we loaded the parameter upon starting the node.
        self.problem = Problem(
            limb_names=self.params.limbs,
            constraint_paths=self.params.paths,
            suffix_com=self.params.suffix_com,
            suffix_feet=self.params.suffix_feet,
        )

        # Colors
        self.colors = [
            (31, 119, 180),
            (255, 127, 14),
            (44, 160, 44),
            (214, 39, 40),
            (227, 119, 194),
            (127, 127, 127),
            (188, 189, 34),
            (23, 190, 207),
        ]

        if self.params.plot:
            self.fig = plt.figure("sl1m_ros results")
            self.ax = self.fig.add_subplot(111, projection="3d")

    def current_contact_callback(self, msg):
        assert len(self.params.limbs) == len(msg.transforms)
        self.initial_contacts_mutex.acquire()
        for i in range(len(self.initial_contacts)):
            self.initial_contacts[i][:] = [
                msg.transforms[i].transform.translation.x,
                msg.transforms[i].transform.translation.y,
                msg.transforms[i].transform.translation.z,
            ]
            self.initial_orientations[i][:] += [
                msg.transforms[i].transform.rotation.x,
                msg.transforms[i].transform.rotation.y,
                msg.transforms[i].transform.rotation.z,
                msg.transforms[i].transform.rotation.w,
            ]
        ori = average_quaternion(self.initial_orientations)
        self.initial_orientation = Quaternion(ori[3], ori[0], ori[1], ori[2])
        self.initial_contacts_frame_id = msg.transforms[0].header.frame_id
        self.initial_contacts_mutex.release()

    def destination_contact_callback(self, msg):
        assert len(self.params.limbs) == len(msg.transforms)
        self.destination_contacts_mutex.acquire()
        for i in range(len(self.destination_contacts)):
            self.destination_contacts[i][:] = [
                msg.transforms[i].transform.translation.x,
                msg.transforms[i].transform.translation.y,
                msg.transforms[i].transform.translation.z,
            ]
            self.destination_orientations[i][:] += [
                msg.transforms[i].transform.rotation.x,
                msg.transforms[i].transform.rotation.y,
                msg.transforms[i].transform.rotation.z,
                msg.transforms[i].transform.rotation.w,
            ]
        ori = average_quaternion(self.destination_orientations)
        self.destination_orientation = Quaternion(ori[3], ori[0], ori[1], ori[2])
        self.destination_contacts_frame_id = msg.transforms[0].header.frame_id
        self.destination_contacts_mutex.release()

    def polygon_callback(self, msg):
        all_polygons = []
        # Getting the list of surfaces
        for surface in msg.markers:
            point_list = []
            for point in surface.points:
                point_list.append([point.x, point.y, point.z])
            all_polygons.append(np.array(point_list).T)
            self.all_polygons_frame_id = surface.header.frame_id

        self.all_polygons_mutex.acquire()
        self.all_polygons = deepcopy(all_polygons)
        self.all_polygons_mutex.release()

    def plot_if_results(self, surfaces, initial_contacts, result):
        if self.params.plot:
            self.ax = sl1m_plot.draw_scene(surfaces, self.params.gait, ax=self.ax)
            sl1m_plot.plot_initial_contacts(initial_contacts, ax=self.ax)
            if result.success:
                sl1m_plot.plot_planner_result(
                    result.all_feet_pos,
                    coms=result.coms,
                    ax=self.ax,
                    show=False,
                )
            plt.draw()
            plt.pause(0.001)

    def get_input_copies(self):
        self.all_polygons_mutex.acquire()
        all_polygons = deepcopy(self.all_polygons)
        self.all_polygons_mutex.release()

        self.initial_contacts_mutex.acquire()
        initial_orientation = Quaternion(self.initial_orientation)
        initial_contacts = deepcopy(self.initial_contacts)
        self.initial_contacts_mutex.release()

        self.destination_contacts_mutex.acquire()
        destination_orientation = Quaternion(self.destination_orientation)
        destination_contacts = deepcopy(self.destination_contacts)
        self.destination_contacts_mutex.release()
        return (
            all_polygons,
            initial_contacts,
            initial_orientation,
            destination_contacts,
            destination_orientation,
        )

    def run(self):
        no_valid_input = True
        ros_rate = rospy.Rate(self.params.rate)
        while not rospy.is_shutdown():
            # Get the time at the beginning of the loop
            t_start = clock()
            self.params.get_ros_param()
            while no_valid_input:
                # Work with local copies
                (
                    all_polygons,
                    initial_contacts,
                    _,
                    destination_contacts,
                    _,
                ) = self.get_input_copies()

                if (
                    not all_polygons
                    or np.all(
                        np.array(initial_contacts)
                        == np.array([np.zeros(3), np.zeros(3)])
                    )
                    or np.all(
                        np.array(destination_contacts)
                        == np.array([np.zeros(3), np.zeros(3)])
                    )
                ):
                    time.sleep(0.05)
                else:
                    no_valid_input = False

            # Work with local copies
            (
                all_polygons,
                initial_contacts,
                initial_orientation,
                destination_contacts,
                destination_orientation,
            ) = self.get_input_copies()

            t_acquiring_data = clock()

            # Get all polygons for all phases.
            all_polygons = self.add_start_polygon(all_polygons, initial_contacts)

            # Compute the number of steps needed.
            flying_distance = np.linalg.norm(np.average(self.initial_contacts))
            yaw = abs(
                matrixToRpy(
                    (destination_orientation * initial_orientation.inverse()).matrix()
                )[2]
            )
            print("flying_distance = ", flying_distance)
            print("yaw = ", yaw)
            print("self.params.step_length = ", self.params.step_length)
            nb_step = int(
                min(
                    max(
                        flying_distance / self.params.step_length[0],
                        yaw / self.params.step_length[1],
                        2,
                    )
                    * 2,
                    self.params.nb_steps_max,
                )
            )
            print("nb_step = ", nb_step)

            # Getting the list of surfaces
            surfaces = nb_step * [[all_polygons]]

            # Update the orientation and cost:
            self.params.costs["end_effector_positions"] = [
                1.0,
                [v.tolist() for v in destination_contacts],
            ]
            # Slerp between initial and final orientation
            slerp_dt = np.arange(0.0, 1.0, 1.0 / (nb_step - 1)).tolist() + [1.0]
            base_orientations = [
                self.initial_orientation.slerp(
                    dt, self.destination_orientation
                ).matrix()
                for dt in slerp_dt
            ]

            t_surface = clock()

            self.problem.generate_problem(
                base_orientations, surfaces, self.params.gait, initial_contacts
            )

            t_problem = clock()

            if self.params.use_sl1m:
                result = solve_L1_combinatorial(
                    self.problem,
                    costs=self.params.costs,
                    com=self.params.optimize_com,
                    lp_solver=self.params.get_solver_type(),
                    qp_solver=self.params.get_solver_type(),
                )
            else:
                result = solve_MIP(
                    self.problem,
                    costs=self.params.costs,
                    com=self.params.optimize_com,
                    solver=self.params.get_solver_type(),
                )

            if not result.success:
                rospy.loginfo("Result.succeded: {}".format(result.success))
                rospy.loginfo("Result.time: {}".format(result.time))
                rospy.loginfo(
                    "Optimized number of steps: {}".format(self.problem.n_phases)
                )
                rospy.loginfo(
                    "The LP and QP optimizations take: {}".format(result.time)
                )
                rospy.loginfo("result.coms = {}".format(result.coms))
                rospy.loginfo(
                    "result.moving_feet_pos = {}".format(result.moving_feet_pos)
                )
                rospy.loginfo("result.all_feet_pos = {}".format(result.all_feet_pos))
                rospy.loginfo(
                    "result.surface_indices = {}".format(result.surface_indices)
                )

            self.plot_if_results(all_polygons, initial_contacts, result)

            result_ros = self.result_to_ros_msg(result, all_polygons)
            self.result_publisher.publish(result_ros)

            t_end = clock()
            rospy.loginfo("Total time is: {}".format((t_end - t_start)))
            rospy.loginfo(
                "Acquiring the data takes: {}".format((t_acquiring_data - t_start))
            )
            rospy.loginfo(
                "Computing the surfaces takes : {}".format(
                    (t_surface - t_acquiring_data)
                )
            )
            rospy.loginfo(
                "Generating the problem dictionary takes: {}".format(
                    (t_problem - t_surface)
                )
            )
            rospy.loginfo("Solving the problem takes: {}".format((t_end - t_problem)))
            ros_rate.sleep()

    def result_to_ros_msg(self, result, polygons):
        ret = MarkerArray()
        commun_header = Header()
        coms = Marker()
        if result.coms:
            for pose in result.coms:
                p = Point()
                p.x = pose[0]
                p.y = pose[1]
                p.z = pose[2]
                coms.points.append(p)
            r, g, b = self.colors[len(result.all_feet_pos) + 1]
            coms.color.r = r
            coms.color.g = g
            coms.color.b = b
            coms.header = commun_header
            coms.header.frame_id = self.all_polygons_frame_id
            coms.type = Marker.LINE_STRIP
            coms.action = Marker.ADD
            coms.id = 0
            coms.pose.orientation.w = 1.0
            coms.scale.x = 0.05
            coms.scale.y = 0.05
            coms.scale.z = 0.05
            coms.color.a = 1.0
            coms.frame_locked = True
            coms.ns = "/sl1m_ros/solution/coms"
            ret.markers.append(coms)

        moving_feet_pos = Marker()
        if result.moving_feet_pos:
            for pose in result.moving_feet_pos:
                p = Point()
                p.x = pose[0][0]
                p.y = pose[0][1]
                p.z = pose[0][2]
                moving_feet_pos.points.append(p)
            r, g, b = self.colors[len(result.all_feet_pos) + 2]
            moving_feet_pos.color.r = r
            moving_feet_pos.color.g = g
            moving_feet_pos.color.b = b
            moving_feet_pos.header = commun_header
            moving_feet_pos.header.frame_id = self.all_polygons_frame_id
            moving_feet_pos.type = Marker.SPHERE_LIST
            moving_feet_pos.action = Marker.ADD
            moving_feet_pos.pose.orientation.w = 1.0
            moving_feet_pos.scale.x = 0.05
            moving_feet_pos.scale.y = 0.05
            moving_feet_pos.scale.z = 0.05
            moving_feet_pos.color.a = 1.0
            moving_feet_pos.frame_locked = True
            moving_feet_pos.ns = "/sl1m_ros/solution/moving_feet_pos"
            ret.markers.append(moving_feet_pos)

        for i, (limb, end_eff_pose) in enumerate(
            zip(self.params.limbs, result.all_feet_pos)
        ):
            marker = Marker()
            if result.all_feet_pos:
                for pose in end_eff_pose:
                    if pose is not None:
                        p = Point()
                        p.x = pose[0]
                        p.y = pose[1]
                        p.z = pose[2]
                        marker.points.append(p)
            r, g, b = self.colors[i]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.header = commun_header
            marker.header.frame_id = self.all_polygons_frame_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.frame_locked = True
            marker.ns = "/sl1m_ros/solution/" + limb + "_pos"
            ret.markers.append(marker)

        for i, polygon in enumerate(polygons):
            marker = Marker()
            marker.points = []
            for j in range(polygon.shape[1]):
                p = Point()
                p.x = polygon[0, j]
                p.y = polygon[1, j]
                p.z = polygon[2, j]
                marker.points.append(p)
            marker.points.append(marker.points[0])
            r, g, b = self.color(i, len(polygons))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.header = Header()
            marker.header.frame_id = self.all_polygons_frame_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.id = i
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.frame_locked = True
            marker.ns = "/sl1m/polygon_used"
            ret.markers.append(marker)
        return ret

    def add_start_polygon(self, all_polygons, initial_contacts, margin=0.2):
        np_init_c = np.array(initial_contacts).T
        x_min = np.min(np_init_c[0, :]) - margin
        x_max = np.max(np_init_c[0, :]) + margin
        y_min = np.min(np_init_c[1, :]) - margin
        y_max = np.max(np_init_c[1, :]) + margin
        z = np.average(np_init_c[2, :])
        start_polygon = np.array(
            [
                [x_max, y_max, z],
                [x_min, y_max, z],
                [x_min, y_min, z],
                [x_max, y_min, z],
            ]
        ).T
        all_polygons.append(start_polygon)
        return all_polygons

    def color(self, i, i_max):
        hue = float(i) / float(i_max)
        print(i, i_max, hue)
        (r, g, b) = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        return int(255 * r), int(255 * g), int(255 * b)
