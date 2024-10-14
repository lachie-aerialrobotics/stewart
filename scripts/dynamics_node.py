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
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_slerp

#code to determine servo positions from end effector pose setpoints
class DynamicsNode:
    def __init__(self):
        #get geometry values form parameter server
        rb = rospy.get_param('/base_radius')
        rp = rospy.get_param('/platform_radius')
        sb = rospy.get_param('/base_joint_spacing')
        sp = rospy.get_param('/platform_joint_spacing')
        ra = rospy.get_param('/proximal_link_length')
        rs = rospy.get_param('/distal_link_length')
        self.k = kinematics.Kinematics(rb, rp, sb, sp, ra, rs)

        EE_mass = 1.
        EE_I = np.eye(3)
        
        proximal_mass = 1.
        proximal_I = np.eye(3)

        distal_mass = 1.
        distal_I = np.eye(3)

        self.EE = linkage(EE_mass, EE_I)
        self.proximal = []
        self.distal = []
        for i in range(6):
            self.distal.append(linkage(distal_mass, distal_I))
            self.proximal.append(linkage(proximal_mass, proximal_I))

        self.br = tf2_ros.TransformBroadcaster()
        br_static = tf2_ros.StaticTransformBroadcaster()

        self.pub_ee_wrench = rospy.Publisher('/ee_wrench', WrenchStamped, queue_size=1, tcp_nodelay=True)
        self.pub_total_wrench = rospy.Publisher('/total_wrench', WrenchStamped, queue_size=1, tcp_nodelay=True)
        self.pub_proximal_wrench = []
        self.pub_distal_wrench = []
        for i in range(6):
            self.pub_proximal_wrench.append(rospy.Publisher('/proximal_wrench_'+str(i), WrenchStamped, queue_size=1, tcp_nodelay=True))
            self.pub_distal_wrench.append(rospy.Publisher('/distal_wrench_'+str(i), WrenchStamped, queue_size=1, tcp_nodelay=True))

        rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.dynamics_callback, queue_size=1, tcp_nodelay=True) #target pose subscriber

    def publish_state_vector_as_tf(self, state_vector:np.asarray, stamp:rospy.Time, frame_id:str, child_frame_id:str):
        tf = cvs.Array2TransformStamped(state_vector, stamp=stamp, frame_id=frame_id, child_frame_id=child_frame_id)
        self.br.sendTransform(tf)

    def publish_vector_as_wrench(self, vector:np.asarray, stamp:rospy.Time, frame_id:str, publisher:rospy.Publisher):
        wrench = cvs.Array2WrenchStamped(vector, stamp=stamp, frame_id=frame_id)
        publisher.publish(wrench)

    def dynamics_callback(self, pose_msg:PoseStamped): 
        Q = cvs.PoseStamped2Array(pose_msg)
        Theta = self.k.IPK(Q)
        EE_CoM_state, proximal_CoM_state, distal_CoM_state = self.k.CoMs(Q, Theta)
        if not np.any(np.isnan(Theta)):
            self.publish_state_vector_as_tf(EE_CoM_state, pose_msg.header.stamp, 'stewart_base', 'ee_CoM')
            for i in range(6):
                self.publish_state_vector_as_tf(proximal_CoM_state[i], pose_msg.header.stamp, 'stewart_base', 'proximal_link_'+str(i)+'_CoM')
                self.publish_state_vector_as_tf(distal_CoM_state[i], pose_msg.header.stamp, 'stewart_base', 'distal_link_'+str(i)+'_CoM')


            EE_F = self.EE.update(EE_CoM_state)
            self.publish_vector_as_wrench(EE_F, pose_msg.header.stamp, 'stewart_base', self.pub_ee_wrench)
            for i in range(6):
                proximal_F = self.proximal[i].update(proximal_CoM_state[i])
                self.publish_vector_as_wrench(proximal_F, pose_msg.header.stamp, 'stewart_base', self.pub_proximal_wrench[i])

                distal_F = self.distal[i].update(distal_CoM_state[i])
                self.publish_vector_as_wrench(distal_F, pose_msg.header.stamp, 'stewart_base', self.pub_distal_wrench[i])

            total_F = EE_F + proximal_F + distal_F
            self.publish_vector_as_wrench(total_F, pose_msg.header.stamp, 'stewart_base', self.pub_total_wrench)


        

    
class linkage:
    def __init__(self, mass, inertia_pa):
        self.mass = mass
        self.inertia_pa = inertia_pa
        self.t = rospy.Time.now()
        self.state = np.zeros(6)
        self.state_dot = np.zeros(6)
        self.Mom = np.zeros(6)
    
    def update(self, new_state):
        # get timestep
        dt = (rospy.Time.now() - self.t).to_sec()
        self.t = rospy.Time.now()

        # update speeds and accelerations
        state_dot = first_deriv(new_state, self.state, dt)
        state_ddot = second_deriv(state_dot, self.state_dot, new_state, dt)

        self.state = new_state
        self.state_dot = state_dot

        R = matrix_from_euler(self.state[3:6])
        I = rotate_tensor(self.inertia_pa, R)
        M = self.mass * np.eye(3)
        G = np.zeros((6,6))
        G[0:3,0:3] = I
        G[3:6,3:6] = M

        Vdot = np.zeros(6)
        Vdot[0:3] = state_ddot[3:6]
        Vdot[3:6] = state_ddot[0:3]

        V = np.zeros(6)
        V[0:3] = state_dot[3:6]
        V[3:6] = state_dot[0:3]

        advT = np.transpose(lie_bracket(V[0:3], V[3:6]))

        F_flipped = np.matmul(G,Vdot) - np.matmul(np.matmul(advT, G), V)

        F = np.zeros(6)
        F[0:3] = F_flipped[3:6]
        F[3:6] = F_flipped[0:3]

        return F


def first_deriv(x1,x0,dt):
    xdot = np.zeros(6)
    for i in range(3):
        xdot[i] = (x1[i] - x0[i]) / dt

    theta1  = x1[3]
    phi1    = x1[4]
    psi1    = x1[5]

    theta0  = x0[3]
    phi0    = x0[4]
    psi0    = x0[5]

    thetadot    = (theta1 - theta0) / dt
    phidot      = (phi1 - phi0)     / dt
    psidot      = (psi1 - psi0)     / dt

    # xdot[3] = phidot * np.sin(theta1) * np.sin(psi1) + thetadot * np.cos(psi1)
    # xdot[4] = phidot * np.sin(theta1) * np.cos(psi1) - thetadot * np.sin(psi1)
    # xdot[5] = phidot * np.cos(theta1) + psidot

    ctheta = np.cos(theta1)
    stheta = np.sin(theta1)
    cphi = np.cos(phi1)
    sphi = np.sin(phi1)
    # cpsi = np.cos(psi1)
    # spsi = np.sin(psi1)

    Rdot = np.asarray([[0, cphi, sphi*ctheta],
                            [0, sphi, -cphi * stheta],
                            [1, 0, ctheta]])
    
    xdot[3:6] = np.matmul(Rdot, np.asarray([phidot,  thetadot, psidot]))

    return xdot


def second_deriv(xdot1, xdot0, x1, dt):
    xddot = np.zeros(6)
    for i in range(3):
        xddot[i] = (xdot1[i] - xdot0[i]) / dt
    
    theta1  = x1[3]
    phi1    = x1[4]
    psi1    = x1[5]

    thetadot0 = xdot0[3]
    phidot0 = xdot0[4]
    psidot0 = xdot0[5]

    thetadot1 = xdot1[3]
    phidot1= xdot1[4]
    psidot1 = xdot1[5]

    thetaddot    = (thetadot1 - thetadot0) / dt
    phiddot      = (phidot1 - phidot0)     / dt
    psiddot      = (psidot1 - psidot0)     / dt


    ctheta = np.cos(theta1)
    stheta = np.sin(theta1)
    cphi = np.cos(phi1)
    sphi = np.sin(phi1)

    Rdot = np.asarray([[0, cphi, sphi*ctheta],
                            [0, sphi, -cphi * stheta],
                            [1, 0, ctheta]])
    
    Rddot = np.asarray([[0, -phidot1 *sphi, phidot1 * cphi * stheta + thetadot1 * sphi * ctheta],
                        [0, phidot1 * cphi, phidot1 * sphi * stheta - thetadot1 * cphi * ctheta],
                        [0, 0, -thetadot1 * stheta]])
    
    alpha = np.matmul(Rdot, np.asarray([thetaddot, phiddot, psiddot])) + np.matmul(Rddot, np.asarray([phidot1,  thetadot1, psidot1]))

    xddot[3:6] = alpha

    return xddot

def matrix_from_euler(angles):
    # R = euler_matrix(angles[0], angles[1], angles[2])
    
    phi = angles[1]
    psi = angles[2]
    theta = angles[0]

    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    Rpsi = np.asarray([[cpsi, -spsi, 0],
                            [spsi, cpsi, 0],
                            [0, 0, 1]])

    Rtheta = np.asarray([[1, 0, 0],
                        [0, ctheta, -stheta],
                        [0, stheta, ctheta]])

    Rphi = np.asarray([[cphi, 0, sphi],
                        [0, 1, 0],
                        [-sphi, 0, cphi]])

    wRp = np.matmul(np.matmul(Rpsi, Rtheta), Rphi)

    return wRp

def rotate_tensor(tensor, R):
    return np.matmul(np.matmul(np.transpose(R), tensor), R)

def x_product_matrix(vec):
    return np.cross(np.eye(3),vec)

def lie_bracket(omega, v):
    lie = np.zeros((6,6))
    lie[0:3,0:3] = x_product_matrix(omega)
    lie[3:6, 0:3] = x_product_matrix(v)
    lie[3:6, 3:6] = x_product_matrix(omega)
    return lie


if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('dynamics_node')
    dN = DynamicsNode()
    rospy.spin()