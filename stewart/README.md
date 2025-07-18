# My notes:

``` gz sim ```
``` ros2 launch gz_attach_links stewart_spawn.launch.py```
``` gz topic -t servo1/cmd -m gz.msgs.Double -p 'data:0.1' ```
``` ros2 launch gz_attach_links stewart_gazebo_interface.launch.py```

``` ros2 topic pub --once /servo1/cmd std_msgs/msg/Float64 '{data: -0.1}' ```
``` export GZ_SIM_RESOURCE_PATH=~/$ROS_WORKSPACE$/src/gz_attach_links/models:$GZ_SIM_RESOURCE_PATH ```

Tested on Ubuntu 22 LTS, ROS2 Humble, Gazebo Harmonic