import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import (Node, SetParameter)

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # ROS-Gazebo Bridge
    rosgz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="rosgz_bridge",
        output="screen",
        arguments=[
            # "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            # "/world/default/model/x500_stewart_0/joint_state" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
            "servo1/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            "servo2/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            "servo3/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            "servo4/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            "servo5/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            "servo6/cmd" + "@std_msgs/msg/Float64" + "]gz.msgs.Double",
            # servo feedback (second JointStatePublisher in stewart_platform.sdf)
            "/stewart/servo_state" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
        ])

    geometry_config = os.path.join(
        get_package_share_directory("stewart"), "config", "stewart_geometry.yaml")

    kinematics = Node(
        package="stewart",
        executable="kinematics_node",
        name="kinematics_node",
        output="screen",
        parameters=[geometry_config],
        arguments=[],
    )

    msg_converter = Node(
        package="stewart",
        executable="msg_converter_node",
        name="msg_converter",
        output="screen",
        arguments=[],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            rosgz_bridge,
            kinematics,
            msg_converter,
        ]
    )