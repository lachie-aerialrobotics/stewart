#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import conversions as cvs
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.msg import State

class Stabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        tfBroadcasterStatic = tf2_ros.StaticTransformBroadcaster()

        tooltip_len = rospy.get_param("nozzle_length")
        tf_tip_to_platform = cvs.Array2TransformStamped(np.asarray([0,0,-tooltip_len,0,0,0]), rospy.Time.now(), frame_id='tooltip_sp', child_frame_id='platform_sp')
        tf_platform_to_tip = cvs.Array2TransformStamped(np.asarray([0,0,tooltip_len,0,0,0]), rospy.Time.now(), frame_id='platform', child_frame_id='tooltip')
        tf_workspace_center_to_tooltip_init = cvs.Array2TransformStamped(np.asarray([0,0,tooltip_len,0,0,0]), rospy.Time.now(), frame_id='workspace_center', child_frame_id='tooltip_init')
        self.manip_mode = 'STATIC'

        # make sure drone is online
        rospy.loginfo("Stabiliser: waiting for mavros...")
        rospy.wait_for_message('/mavros/state', State)
        rospy.loginfo("Stabiliser: waiting for manipulator to init...")
        rospy.wait_for_message('/manipulator/state', String)

        # send static tfs
        try:
            self.t = self.tfBuffer.lookup_transform('stewart_base', 'workspace_center', time=rospy.Time(0), timeout=rospy.Duration(10))
            tf_base_2_manip = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Tf frames not initialised :(")

        t_flat = self.t
        t_flat.header.frame_id = 'stewart_base_flat'
        t_flat.child_frame_id = 'workspace_center_flat'
        tfBroadcasterStatic.sendTransform(t_flat)

        tf_base_2_manip_flat = tf_base_2_manip
        tf_base_2_manip_flat.header.frame_id = 'base_link_flat'
        tf_base_2_manip_flat.child_frame_id = 'stewart_base_flat'
        tfBroadcasterStatic.sendTransform(tf_base_2_manip_flat)

        tfBroadcasterStatic.sendTransform(tf_tip_to_platform)
        tfBroadcasterStatic.sendTransform(tf_platform_to_tip)
        tfBroadcasterStatic.sendTransform(tf_workspace_center_to_tooltip_init)

        #init publishers and subscribers
        self.pub_platform_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/manipulator/state', String, self.state_callback, queue_size=1, tcp_nodelay=True) 
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/tooltip_setpoint/pose', PoseStamped, self.tooltip_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/tooltip_setpoint/velocity', TwistStamped, self.tooltip_twist_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def state_callback(self, state):
        self.manip_mode = state.data

    def drone_pose_callback(self, drone_pose_msg=PoseStamped()):
        drone_pose_flat = make_flat(drone_pose_msg)
        drone_transform_flat = cvs.PoseStamped2TransformStamped(drone_pose_flat, child_frame_id='base_link_flat')
        self.tfBroadcaster.sendTransform(drone_transform_flat)

        if self.manip_mode == "STAB_6DOF":
            target = 'platform_sp'
        elif self.manip_mode == "STAB_3DOF":
            target = 'workspace_center_flat'
        elif self.manip_mode == "STATIC":
            target = 'workspace_center'
        else:
            rospy.logwarn('Invalid manipulator state!')
            target = 'workspace_center'

        try:
            self.t = self.tfBuffer.lookup_transform('stewart_base', target, time=drone_pose_msg.header.stamp, timeout=rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Manipulator setpoint tf dropped')

        self.pub_platform_pose.publish(cvs.TransformStamped2PoseStamped(self.t))

    def tooltip_pose_callback(self, tip_pose_msg=PoseStamped()):
        if tip_pose_msg.pose.orientation == Quaternion(0,0,0,0):
            tip_pose_msg.pose.orientation = Quaternion(0,0,0,1)

        # vel_time = self.tip_vel_msg.header.stamp.to_sec() + self.tip_vel_msg.header.stamp.to_nsec()

        if self.tip_vel_msg.header.stamp.to_nsec() >= tip_pose_msg.header.stamp.to_nsec():
            
            dt = self.tip_vel_msg.header.stamp.to_nsec() - tip_pose_msg.header.stamp.to_nsec()
            print(dt)
            tip_pose_msg.pose.position.x += self.tip_vel_msg.twist.linear.x * dt
            tip_pose_msg.pose.position.y += self.tip_vel_msg.twist.linear.y * dt
            tip_pose_msg.pose.position.z += self.tip_vel_msg.twist.linear.z * dt

        tip_tf_msg = cvs.PoseStamped2TransformStamped(tip_pose_msg, child_frame_id='tooltip_sp')
        self.tfBroadcaster.sendTransform(tip_tf_msg)

    def tooltip_twist_callback(self, tip_vel_msg=TwistStamped()):
        self.tip_vel_msg = tip_vel_msg


def make_flat(pose=PoseStamped()):
    quaternion = np.asarray([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
    (R, P, Y) = euler_from_quaternion(quaternion) 
    quaternion = quaternion_from_euler(0, 0, Y)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('stabilisation')
    S = Stabilisation()