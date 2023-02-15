## Sl1m ROS

This is a ROS wrapper around the sl1m package.

### Build the package to install it

Clone the repository in your catkin workspace and then do `catkin build`

### Run demo

There is a demo self contained roslaunch allowing you to optimize a 
trajectory for a simple biped robot from:
- Left foot pose: [-0.2, 0.1, 0.0]
- Right foot pose: [-0.2, -0.1, 0.0]
to:
- Left foot pose: [0.3, 0.1, 0.0]
- Right foot pose: [0.3, -0.1, 0.0]
In 4 steps.
```
source install/setup.bash
roslaunch sl1m_ros demo_sl1m_ros.launch
```

Then publish the desired goal:
```
rostopic pub /sl1m_ros/destination_contact tf2_msgs/TFMessage "[ {transform: { translation: { x: 0.3, y: 0.1, z: 0} } } ,{transform: { translation: { x: 0.3, y: -0.1, z: 0}}}]" 
```
