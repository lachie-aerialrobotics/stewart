import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
from stewart_msgs.msg import ServoAnglesStamped

import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

import kinematics
import conversions as cvs

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # Get parameters from ROS2 parameter server
        self.declare_parameter('base_radius', 0.1)
        self.declare_parameter('platform_radius', 0.1)
        self.declare_parameter('base_joint_spacing', 0.1)
        self.declare_parameter('platform_joint_spacing', 0.1)
        self.declare_parameter('proximal_link_length', 0.2)
        self.declare_parameter('distal_link_length', 0.3)
        self.declare_parameter('translation_limit', 0.05)
        self.declare_parameter('rotation_limit', 0.1)

        rb = self.get_parameter('base_radius').value
        rp = self.get_parameter('platform_radius').value
        sb = self.get_parameter('base_joint_spacing').value
        sp = self.get_parameter('platform_joint_spacing').value
        ra = self.get_parameter('proximal_link_length').value
        rs = self.get_parameter('distal_link_length').value

        self.k = kinematics.Kinematics(rb, rp, sb, sp, ra, rs)
        self.Theta = np.zeros(6)
        self.time = self.get_clock().now().to_msg()
        
        translation_limit = self.get_parameter('translation_limit').value
        rotation_limit = self.get_parameter('rotation_limit').value

        self.Qhome = self.k.DefineHomePos()
        self.Q0 = self.Qhome
        self.Q_sp = self.Qhome

        self.translation_limit = np.ones(3) * translation_limit + self.Qhome[:3]
        self.rotation_limit = np.ones(3) * rotation_limit + self.Qhome[3:6]

        # TF2 Broadcasters
        self.br = tf2_ros.TransformBroadcaster(self)
        self.br_static = tf2_ros.StaticTransformBroadcaster(self)

        tf_workspace = cvs.Array2TransformStamped(self.Qhome, self.time, frame_id='stewart_base', child_frame_id='workspace_center')
        self.br_static.sendTransform([tf_workspace])

        # Publishers
        self.pub_servo_angles = self.create_publisher(ServoAnglesStamped, '/servo_setpoint/positions', 10)
        self.pub_platform = self.create_publisher(PoseStamped, '/platform_detected/pose', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/platform_setpoint/pose', self.ipk_callback, 10)
        self.create_subscription(ServoAnglesStamped, '/servo_detected/positions', self.fpk_callback, 10)

    def ipk_callback(self, platform_pos: PoseStamped):
        self.Q_sp = cvs.PoseStamped2Array(platform_pos)
        if self.k.CheckLims(self.Q_sp, self.translation_limit, self.rotation_limit):
            Theta = self.k.IPK(self.Q_sp)
        else:
            Theta = np.nan * np.ones(6)
            self.get_logger().warn('Manipulator workspace exceeded!')

        if not np.any(np.isnan(Theta)):
            msg = cvs.Array2ServoAnglesStamped(Theta, platform_pos.header.stamp)
            self.pub_servo_angles.publish(msg)

    def fpk_callback(self, servo_angles: ServoAnglesStamped):
        self.Theta = cvs.ServoAnglesStamped2Array(servo_angles)
        Q0 = self.k.FPK(self.Theta, self.Q0 + np.asarray([-0.001, -0.001, -0.001, 0.001, 0.001, 0.001]))
        platform_tf = cvs.Array2TransformStamped(Q0, servo_angles.header.stamp, frame_id='stewart_base', child_frame_id='platform')
        platform_pose = cvs.Array2PoseStamped(Q0, servo_angles.header.stamp)
        self.br.sendTransform(platform_tf)
        self.pub_platform.publish(platform_pose)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
