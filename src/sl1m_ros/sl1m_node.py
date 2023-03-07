# standard imports
import colorsys
import threading
import math
from copy import deepcopy
import numpy as np
from pinocchio import SE3
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

        # Startup polygon is the area around the robot that is not perceived
        # yet and assumed flat.
        self.startup_polygon = None

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
        self.destination_orientation = Quaternion(
            ori[3], ori[0], ori[1], ori[2]
        )
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
            self.ax = sl1m_plot.draw_scene(
                surfaces, self.params.gait, ax=self.ax
            )
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

    def is_environment_valid(self):
        # Work with local copies
        (
            all_polygons,
            _,
            _,
            _,
            _,
        ) = self.get_input_copies()
        return bool(all_polygons)

    def is_initial_state_valid(self):
        # Work with local copies
        (
            _,
            initial_contacts,
            _,
            _,
            _,
        ) = self.get_input_copies()
        return not np.all(
            np.array(initial_contacts) == np.array([np.zeros(3), np.zeros(3)])
        )

    def is_goal_valid(self):
        # Work with local copies
        (
            _,
            _,
            _,
            destination_contacts,
            _,
        ) = self.get_input_copies()
        return not np.all(
            np.array(destination_contacts)
            == np.array([np.zeros(3), np.zeros(3)])
        )

    def valid_input_available(self):
        # Work with local copies
        (
            all_polygons,
            initial_contacts,
            _,
            destination_contacts,
            _,
        ) = self.get_input_copies()
        return not (
            not all_polygons
            or np.all(
                np.array(initial_contacts)
                == np.array([np.zeros(3), np.zeros(3)])
            )
            or np.all(
                np.array(destination_contacts)
                == np.array([np.zeros(3), np.zeros(3)])
            )
        )

    def compute_nb_steps(self, destination):
        """Compute the distance from the initial state to the goal to reach."""

        self.initial_contacts_mutex.acquire()
        initial_position = sum(self.initial_contacts)
        initial_position /= len(self.initial_contacts)
        initial_orientation = self.initial_orientation
        self.initial_contacts_mutex.release()

        self.destination_contacts_mutex.acquire()
        if type(destination) == list:
            destination_position = sum(self.destination_contacts)
            destination_position /= len(self.destination_contacts)
        else:
            destination_position = np.array(
                [
                    destination.target_pose.position.x,
                    destination.target_pose.position.y,
                    destination.target_pose.position.z,
                ]
            )
        destination_orientation = self.destination_orientation
        self.destination_contacts_mutex.release()

        distance = np.linalg.norm(initial_position - destination_position)
        distance_yaw = abs(
            matrixToRpy(
                (
                    initial_orientation * destination_orientation.inverse()
                ).matrix()
            )
        )[2]

        print("Initial position ", initial_position)
        print("Initial orientation ", initial_orientation)
        print("Destination position ", destination_position)
        print("Destination orientation ", destination_orientation)
        print("distance ", distance)
        print("distance_yaw ", distance_yaw)
        print("self.params.step_length = ", self.params.step_length)
        nb_step = int(
            math.ceil(
                min(
                    max(
                        distance / abs(self.params.step_length[0]),
                        distance_yaw / abs(self.params.step_length[1]),
                        2,
                    ),
                    self.params.nb_steps_max,
                )
            )
        ) + 2
        return nb_step

    def run_once(self, nb_step=0, costs={}):
        # Get the time at the beginning of the loop
        self.params.get_ros_param()
        t_start = clock()
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
        all_polygons = self.add_start_polygon()

        if nb_step == 0:
            nb_step = self.compute_nb_steps(destination_contacts)
        print("nb_step = ", nb_step)

        # Getting the list of surfaces
        surfaces = nb_step * [[all_polygons]]

        if len(costs) == 0:
            costs = self.params.costs
            # Update the orientation and cost:
            costs["end_effector_positions"] = [
                10.0,
                [v.tolist() for v in destination_contacts],
            ]
        # Slerp between initial and final orientation
        slerp_dt = np.arange(0.0, 1.0, 1.0 / (nb_step - 1)).tolist() + [1.0]
        base_orientations = [
            initial_orientation.slerp(dt, destination_orientation).matrix()
            for dt in slerp_dt
        ]

        t_surface = clock()

        self.problem.generate_problem(
            base_orientations, surfaces, self.params.gait, initial_contacts
        )

        # Add target orientation at the end of the base_orientation list
        # it is required to have matching size between feet pose results and orientation list
        base_orientations += [destination_orientation.matrix()]

        t_problem = clock()

        if self.params.use_sl1m:
            result = solve_L1_combinatorial(
                self.problem,
                costs=costs,
                com=self.params.optimize_com,
                lp_solver=self.params.get_solver_type(),
                qp_solver=self.params.get_solver_type(),
            )
        else:
            result = solve_MIP(
                self.problem,
                costs=costs,
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
            rospy.loginfo(
                "result.all_feet_pos = {}".format(result.all_feet_pos)
            )
            rospy.loginfo(
                "result.surface_indices = {}".format(result.surface_indices)
            )

        self.plot_if_results(all_polygons, initial_contacts, result)

        result_ros = self.result_to_ros_msg(result, all_polygons)
        rospy.loginfo("Solution found, publishing it on /sl1m_ros/solution")
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
        rospy.loginfo(
            "Solving the problem takes: {}".format((t_end - t_problem))
        )
        return result, base_orientations

    def run(self):
        ros_rate = rospy.Rate(self.params.rate)
        while not rospy.is_shutdown():
            if self.valid_input_available():
                rospy.loginfo("Valid inputs received")
                self.run_once()
                ros_rate.sleep()
            else:
                rospy.loginfo_throttle(1, "No valid input found yet")

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

    def add_start_polygon(
        self,
        min_x_margin=0.2,
        max_x_margin=0.6,
        min_y_margin=0.2,
        max_y_margin=0.2,
    ):
        # Work with local copies
        (
            all_polygons,
            initial_contacts,
            initial_orientation,
            _,
            _,
        ) = self.get_input_copies()
        # we only create once the initial polygon
        max_iter = 50
        iter = 0
        while (
            self.startup_polygon is None or np.isnan(self.startup_polygon).any()
        ) and iter < 50:
            # Work with local copies
            (
                all_polygons,
                initial_contacts,
                initial_orientation,
                _,
                _,
            ) = self.get_input_copies()
            average_pose = np.zeros(3)
            for pose in initial_contacts:
                average_pose += pose
            average_pose /= len(initial_contacts)
            print(average_pose)
            polygon_se3 = SE3(initial_orientation.matrix(), average_pose)
            x_min = -min_x_margin
            x_max = +max_x_margin
            y_min = -min_y_margin
            y_max = +max_y_margin
            z = np.average(polygon_se3.translation[2])
            self.startup_polygon = np.array(
                [
                    polygon_se3.act(np.array([x_max, y_max, z])),
                    polygon_se3.act(np.array([x_min, y_max, z])),
                    polygon_se3.act(np.array([x_min, y_min, z])),
                    polygon_se3.act(np.array([x_max, y_min, z])),
                ]
            ).T
            iter += 1
        if iter == max_iter:
            raise RuntimeError(
                "Failed to add the startup polygon, nan detected"
            )
        all_polygons.append(self.startup_polygon)
        return all_polygons

    def color(self, i, i_max):
        hue = float(i) / float(i_max)
        print(i, i_max, hue)
        (r, g, b) = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        return int(255 * r), int(255 * g), int(255 * b)
