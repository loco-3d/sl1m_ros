# Standard dependencies.
import numpy as np
import colorsys
try:
    from time import perf_counter as clock
except:
    from time import clock

# Ros imports.
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

# Surface processing dep.
from sl1m_ros.third_party.walkgen_surface_processing.surface_processing import SurfaceProcessing
from sl1m_ros.third_party.walkgen_surface_processing.params import SurfaceProcessingParams


class SurfaceProcessingNode:
    def __init__(self):

        # Parameters of the environment
        self.params = SurfaceProcessingParams()
        self.params.n_points = rospy.get_param("surface_processing/n_points", 10)  # Maximum Number of points for for each convex surface.
        self.params.margin = rospy.get_param("surface_processing/margin",0.14)  # Margin in [m] inside the convex surfaces.
        self.params.method_id = rospy.get_param("surface_processing/method_id",3)  # Method for convex decomposition.
        self.params.poly_size = rospy.get_param("surface_processing/poly_size",50)  # Maximum size of the polygon for the convex decomposition.
        self.params.min_area = rospy.get_param("surface_processing/min_area",0.003)   # Area under which the remaining surfaces is deleted.
        self.params.max_height = rospy.get_param("surface_processing/max_height",0.5)   # surfaces above this height will be ignored.

        # Initial height
        initial_height = 0.0
        # Create the surface processing objects.
        self.surface_processing = SurfaceProcessing(
            initial_height=initial_height, params=self.params)

        self.staircase_location_client = rospy.Subscriber(
            "surface_to_process",
            MarkerArray,
            self.process_surface,
            queue_size=5
        )
        self.surface_publisher = rospy.Publisher(
            "surface_processed",
            MarkerArray,
            queue_size=5
        )

    def process_surface(self, msg):
        
        if len(msg.markers) == 0:
            return

        t0 = clock()
        position = np.array([0.,0.,0.])
        all_surfaces = self.surface_processing.run(position, msg)
        t1 = clock()
        # print("Time took to process the surfaces [ms] : ", 1000 * (t1 - t0))

        # print(all_surfaces)

        frame_id = msg.markers[0].header.frame_id
        
        marker_array = MarkerArray()
        for i, key in enumerate(all_surfaces):
            marker = Marker()
            marker.points = []
            for sp in all_surfaces[key]:
                p = Point()
                p.x = sp[0]
                p.y = sp[1]
                p.z = sp[2]
                marker.points.append(p)
            marker.points.append(marker.points[0])
            r, g, b = self.color(i, len(all_surfaces))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.header = Header()
            marker.header.frame_id = frame_id
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
            marker.ns = "/surface_processed"
            marker_array.markers.append(marker)

        self.surface_publisher.publish(marker_array)

    def color(self, i, i_max):
        hue = float(i) / float(i_max)
        #print(i, i_max, hue)
        (r, g, b) = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        return int(255 * r), int(255 * g), int(255 * b)

    def run(self):
        from time import sleep
        while not rospy.is_shutdown():
            sleep(1)