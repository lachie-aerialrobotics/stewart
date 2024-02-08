#!/usr/bin/env python3

#-------------------------------------------------------------------------------------------------
# INVERSE KINEMATICS
# This script computes the inverse kinematics solution for the stewart platform. Basically
# it takes position/velocity/acceleration setpoints for the end-effector and works out what
# the motors need to do. A nice perk of using a parallel manipulator is that these equations
# can be solved directly (so no faffing about with MoveIt! needed)
# The callbacks all operate independently so the publishing rates depend on the rate of the input
# setpoints and are not dependent on eachother.
#-------------------------------------------------------------------------------------------------

import rospy
import numpy as np
import tf2_ros
import kinematics
import conversions as cvs
from geometry_msgs.msg import PoseStamped
from delta_2.msg import ServoAnglesStamped

#code to determine servo positions from end effector pose setpoints
class KinematicsNode:
    def __init__(self):
        #get geometry values form parameter server
        rb = rospy.get_param('/base_radius')
        rp = rospy.get_param('/platform_radius')
        sb = rospy.get_param('/base_joint_spacing')
        sp = rospy.get_param('/platform_joint_spacing')
        ra = rospy.get_param('/proximal_link_length')
        rs = rospy.get_param('/distal_link_length')
        self.k = kinematics.Kinematics(rb, rp, sb, sp, ra, rs)

        translation_limit = rospy.get_param('/translation_limit')
        rotation_limit = rospy.get_param('/rotation_limit')
        self.solve = False

        self.Qhome = self.k.DefineHomePos()
        self.Q0 = self.Qhome
        self.translation_limit = np.ones(3) * translation_limit + self.Qhome[0:3]
        self.rotation_limit = np.ones(3) * rotation_limit + self.Qhome[3:6]

        self.br = tf2_ros.TransformBroadcaster()
        br_static = tf2_ros.StaticTransformBroadcaster()

        tf_workspace = cvs.Array2TransformStamped(self.Qhome, rospy.Time.now(), frame_id='stewart_base', child_frame_id='workspace_center')
        br_static.sendTransform(tf_workspace)

        #init publishers and subscribers
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.ipk_callback, queue_size=1, tcp_nodelay=True) #target pose subscriber
        rospy.Subscriber('/servo_detected/positions', ServoAnglesStamped, self.fpk_callback, queue_size=1, tcp_nodelay=True)

    def ipk_callback(self, platform_pos=PoseStamped()): 
        Q = cvs.PoseStamped2Array(platform_pos)
        if self.k.CheckLims(Q, self.translation_limit, self.rotation_limit):
            Theta = self.k.IPK(Q)
        else:
            Theta = np.nan * np.ones(6)
            rospy.logwarn('Manipulator workspace exceeded!')

        if not np.any(np.isnan(Theta)):
            self.pub_servo_angles.publish(cvs.Array2ServoAnglesStamped(Theta, platform_pos.header.stamp)) 

    def fpk_callback(self, servo_angles=ServoAnglesStamped()):
        Theta = cvs.ServoAnglesStamped2Array(servo_angles)
        self.Q0 = self.k.FPK(Theta, self.Q0+np.asarray([-0.001, -0.001, -0.001, 0.001, 0.001, 0.001])) #fsolve seems to get confused if solution is exactly right
        platform_tf = cvs.Array2TransformStamped(self.Q0, servo_angles.header.stamp, frame_id='stewart_base', child_frame_id='platform')
        self.br.sendTransform(platform_tf)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('kinematics_node')
    kN = KinematicsNode()
    rospy.spin()