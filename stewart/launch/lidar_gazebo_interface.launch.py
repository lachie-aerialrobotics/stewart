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
            "/livox/lidar" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
            "/livox/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
        ],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            rosgz_bridge,
        ]
    )