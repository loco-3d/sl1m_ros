from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from pinocchio import Quaternion

def footpose_to_marker(position, orientation, color, size, ns, id):
    marker = Marker()
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.MODIFY
    marker.ns = ns

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.x = orientation.x
    marker.pose.orientation.y = orientation.y
    marker.pose.orientation.z = orientation.z
    marker.pose.orientation.w = orientation.w

    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.7

    marker.frame_locked = True

    return marker

def foosteps_to_marker_array(feet_positions, orientations, time, feet_size, colors):
    array = MarkerArray()
    header = Header()
    header.frame_id = "map"
    header.stamp = time
    namespaces = ["left_foot", "right_foot"]
    id = 0
    for i in range(1, len(feet_positions[0]) - 1): 
        marker = footpose_to_marker(feet_positions[i%2][i+1], Quaternion(orientations[i+1]),
                                    colors[i%2], feet_size, namespaces[i%2], id)
        id += 1
        marker.header = header
        array.markers.append(marker)
    return array