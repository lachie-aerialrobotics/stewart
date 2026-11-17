# My notes:

``` gz sim ```
``` ros2 launch stewart stewart_spawn.launch.py```
``` gz topic -t servo1/cmd -m gz.msgs.Double -p 'data:0.1' ```
``` ros2 launch stewart stewart_gazebo_interface.launch.py```

``` ros2 topic pub --once /servo1/cmd std_msgs/msg/Float64 '{data: -0.1}' ```
``` export GZ_SIM_RESOURCE_PATH=~/$ROS_WORKSPACE$/src/stewart/stewart/models:$GZ_SIM_RESOURCE_PATH ```

Tested on Ubuntu 22 LTS, ROS2 Humble, Gazebo Harmonic

## Using the model with PX4 SITL (tricopter)

`tricopter` in PX4-gazebo-models includes `model://stewart_platform`, which lives in this package.
PX4's gz launcher only adds `Tools/simulation/gz/models` to the resource path, so export this
package's models before `make px4_sitl gz_tricopter`:

``` export GZ_SIM_RESOURCE_PATH=<path to this repo>/stewart/models:$GZ_SIM_RESOURCE_PATH ```

(after `colcon build` the models are also installed to `install/stewart/share/stewart/models`)

Then start the ROS side:

``` ros2 launch stewart stewart_gazebo_interface.launch.py ```

This loads `config/stewart_geometry.yaml` into `kinematics_node` and bridges:
- `/servoN/cmd` (ROS -> gz). `msg_converter` converts `/servo_setpoint/positions` from degrees to radians.
- `/stewart/servo_state` (gz -> ROS). `msg_converter` republishes this as `/servo_detected/positions` in degrees, matching the real Dynamixel driver.

Servo speed in sim is capped at 6 rad/s (`cmd_max` in `stewart_platform.sdf`, about an XL430-W250 at no load). Change it to match your servos.
