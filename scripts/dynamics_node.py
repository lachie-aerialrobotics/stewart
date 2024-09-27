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
from geometry_msgs.msg import PoseStamped, WrenchStamped
from stewart.msg import ServoAnglesStamped
from tf.transformations import euler_matrix

#code to determine servo positions from end effector pose setpoints
class DynamicsNode:
    def __init__(self):
        # Server(AdmittanceConfig, self.config_callback)

        #get geometry values form parameter server
        rb = rospy.get_param('/base_radius')
        rp = rospy.get_param('/platform_radius')
        sb = rospy.get_param('/base_joint_spacing')
        sp = rospy.get_param('/platform_joint_spacing')
        ra = rospy.get_param('/proximal_link_length')
        rs = rospy.get_param('/distal_link_length')
        self.k = kinematics.Kinematics(rb, rp, sb, sp, ra, rs)

        self.EE_CoM_state = np.zeros(6)
        self.EE_CoM_dot = np.zeros(6)
        self.proximal_CoM_state = np.zeros((6,6))
        self.proximal_CoM_dot = np.zeros((6,6))
        self.distal_CoM_state = np.zeros((6,6))
        self.distal_CoM_dot = np.zeros((6,6))
        self.t = rospy.Time.now()

        self.br = tf2_ros.TransformBroadcaster()
        br_static = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.dynamics_callback, queue_size=1, tcp_nodelay=True) #target pose subscriber

    def dynamics_callback(self, pose_msg:PoseStamped): 
        Q = cvs.PoseStamped2Array(pose_msg)
        Theta = self.k.IPK(Q)
        EE_CoM_state, proximal_CoM_state, distal_CoM_state = self.k.CoMs(Q, Theta)
        
        

        if not np.any(np.isnan(Theta)):
            EE_tf = cvs.Array2TransformStamped(EE_CoM_state, pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='ee_CoM')
            self.br.sendTransform(EE_tf)

            # EE_CoM_state_no_rot = np.hstack((EE_CoM_state[0:3], np.zeros(3)))
            # EE_tf_no_rot = cvs.Array2TransformStamped(EE_CoM_state_no_rot, pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='ee_CoM_no_rot')
            # self.br.sendTransform(EE_tf_no_rot)
            for i in range(6):
                
                # proximal_CoM_state_no_rot = np.hstack((proximal_CoM_state[i][0:3], np.zeros(3)))
                # distal_CoM_state_no_rot = np.hstack((distal_CoM_state[i][0:3], np.zeros(3)))

                proximal_CoM_tf = cvs.Array2TransformStamped(proximal_CoM_state[i], pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='proximal_link_'+str(i)+'_CoM')
                self.br.sendTransform(proximal_CoM_tf)
                distal_CoM_tf = cvs.Array2TransformStamped(distal_CoM_state[i], pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='distal_link_'+str(i)+'_CoM')
                self.br.sendTransform(distal_CoM_tf)

                # proximal_CoM_tf_no_rot = cvs.Array2TransformStamped(proximal_CoM_state_no_rot, pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='proximal_link_'+str(i)+'_CoM_no_rot')
                # self.br.sendTransform(proximal_CoM_tf_no_rot)
                # distal_CoM_tf_no_rot = cvs.Array2TransformStamped(distal_CoM_state_no_rot, pose_msg.header.stamp, frame_id='stewart_base', child_frame_id='distal_link_'+str(i)+'_CoM_no_rot')
                # self.br.sendTransform(distal_CoM_tf_no_rot)


            EE_mass = 1.
            EE_I = np.eye(3)
            EE_R = matrix_from_euler(EE_CoM_state)
            EE_I = rotate_tensor(EE_I, EE_R)
            
            

            proximal_mass = 1.
            proximal_I_ = np.eye(3)

            distal_mass = 1.
            distal_I_ = np.eye(3)

            

            dt = (rospy.Time.now() - self.t).to_sec()
            self.t = rospy.Time.now()

            EE_CoM_dot = differentiate(EE_CoM_state, self.EE_CoM_state, dt)
            EE_CoM_ddot = differentiate(EE_CoM_dot, self.EE_CoM_dot, dt)
            self.EE_CoM_dot = EE_CoM_dot
            self.EE_CoM_state = EE_CoM_state
        
            EE_F = self.k.inertial(EE_mass, EE_I, EE_CoM_ddot)

            proximal_F = np.zeros((6,6))
            distal_F = np.zeros((6,6))
            for i in range(6):
                proximal_R = matrix_from_euler(proximal_CoM_state[i])
                proximal_I = rotate_tensor(proximal_I_, proximal_R)
                distal_R = matrix_from_euler(distal_CoM_state[i])
                distal_I = rotate_tensor(distal_I_, distal_R)


                proximal_CoM_dot = differentiate(proximal_CoM_state, self.proximal_CoM_state, dt)
                proximal_CoM_ddot = differentiate(proximal_CoM_dot, self.proximal_CoM_dot, dt)
                self.proximal_CoM_dot = proximal_CoM_dot
                self.proximal_CoM_state = proximal_CoM_state

                distal_CoM_dot = differentiate(distal_CoM_state, self.distal_CoM_state, dt)
                distal_CoM_ddot = differentiate(distal_CoM_dot, self.distal_CoM_dot, dt)
                self.distal_CoM_dot = distal_CoM_dot
                self.distal_CoM_state = distal_CoM_state

                proximal_F[i,:] = self.k.inertial(proximal_mass, proximal_I, proximal_CoM_ddot[i])
                distal_F[i,:] = self.k.inertial(distal_mass, distal_I, distal_CoM_ddot[i])
            print(proximal_mass)
            print("###########")
            print(proximal_I)
            print('-----------')
            print(proximal_CoM_ddot)


def differentiate(x1,x0,dt):
    xdot = (x1 - x0) / dt
    return xdot

def matrix_from_euler(angles):
    R = euler_matrix(angles[0], angles[1], angles[2])
    return R[0:3,0:3]

def rotate_tensor(tensor, R):
    return np.matmul(np.matmul(R, tensor), np.linalg.inv(R))


if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('dynamics_node')
    dN = DynamicsNode()
    rospy.spin()