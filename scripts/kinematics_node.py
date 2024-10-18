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
from tf.transformations import quaternion_from_euler, quaternion_conjugate, quaternion_multiply
# from scipy import signal

# from dynamic_reconfigure.server import Server
# from hld.cfg import AdmittanceConfig

#code to determine servo positions from end effector pose setpoints
class KinematicsNode:
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

        self.Theta=np.zeros(6)

        self.fx = []
        self.fy = []
        self.fz = []
        self.tx = []
        self.ty = []
        self.tz = []

        self.time = rospy.Time.now()

        translation_limit = rospy.get_param('/translation_limit')
        rotation_limit = rospy.get_param('/rotation_limit')
        self.solve = False

        self.Qhome = self.k.DefineHomePos()
        self.Q0 = self.Qhome
        self.Q_sp = self.Qhome
        self.Q_sp_compliant = np.zeros(6)
        self.Q_dot = np.zeros(6)
        self.Q_ddot = np.zeros(6)

        self.translation_limit = np.ones(3) * translation_limit + self.Qhome[0:3]
        self.rotation_limit = np.ones(3) * rotation_limit + self.Qhome[3:6]

        self.br = tf2_ros.TransformBroadcaster()
        br_static = tf2_ros.StaticTransformBroadcaster()

        tf_workspace = cvs.Array2TransformStamped(self.Qhome, rospy.Time(0), frame_id='stewart_base', child_frame_id='workspace_center')
        br_static.sendTransform(tf_workspace)

        #init publishers and subscribers
        self.pub_servo_angles = rospy.Publisher('/servo_setpoint/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        self.pub_servo_vels = rospy.Publisher('/servo_setpoint/velocities', ServoAnglesStamped, queue_size=1, tcp_nodelay=True)
        self.pub_platform = rospy.Publisher('/platform_detected/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        # self.pub_force = rospy.Publisher('/platform_detected/wrench', WrenchStamped, queue_size=1, tcp_nodelay=True)
        self.pub_setpoint = rospy.Publisher('/platform_setpoint/compliant', PoseStamped, queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('/platform_setpoint/pose', PoseStamped, self.ipk_callback, queue_size=1, tcp_nodelay=True) #target pose subscriber
        rospy.Subscriber('/servo_detected/positions', ServoAnglesStamped, self.fpk_callback, queue_size=1, tcp_nodelay=True)
        # rospy.Subscriber('/servo_detected/torques', ServoAnglesStamped, self.fs_callback, queue_size=1, tcp_nodelay=True)

    def ipk_callback(self, platform_pos=PoseStamped()): 
        Q_sp = cvs.PoseStamped2Array(platform_pos)
        if self.k.CheckLims(Q_sp, self.translation_limit, self.rotation_limit):
            Theta = self.k.IPK(Q_sp)
        else:
            Theta = np.nan * np.ones(6)
            rospy.logwarn('Manipulator workspace exceeded!')

        if not np.any(np.isnan(Theta)):
            self.pub_servo_angles.publish(cvs.Array2ServoAnglesStamped(Theta, platform_pos.header.stamp)) 

        dt = (self.time - rospy.Time.now()).to_sec()
        self.time = rospy.Time.now()
        Qdot_sp = state_derivative(Q_sp, self.Q_sp, dt)
        self.Q_sp = Q_sp

        Theta_dot = self.k.IVK(Q_sp, Qdot_sp, Theta)

        if not np.any(np.isnan(Theta_dot)):
            self.pub_servo_vels.publish(cvs.Array2ServoAnglesStamped(Theta_dot, platform_pos.header.stamp))

    def fpk_callback(self, servo_angles=ServoAnglesStamped()):
        self.Theta = cvs.ServoAnglesStamped2Array(servo_angles)
        Q0 = self.k.FPK(self.Theta, self.Q0+np.asarray([-0.001, -0.001, -0.001, 0.001, 0.001, 0.001])) #fsolve seems to get confused if solution is exactly right
        platform_tf = cvs.Array2TransformStamped(Q0, servo_angles.header.stamp, frame_id='stewart_base', child_frame_id='platform')
        platform_pose = cvs.Array2PoseStamped(Q0, servo_angles.header.stamp)
        self.br.sendTransform(platform_tf)
        self.pub_platform.publish(platform_pose)

    # def fs_callback(self, servo_torques=ServoAnglesStamped()):
    #     T = cvs.ServoAnglesStamped2Array(servo_torques)
    #     F = self.k.IFS(self.Q0, self.Theta, T)
    #     F = self.low_pass_filter(F, self.cutoff_frequency)
    #     platform_forces = cvs.Array2WrenchStamped(F, servo_torques.header.stamp)
    #     self.pub_force.publish(platform_forces)


        

    #     Q_sp_compliant = np.zeros(6)
    #     for i in range(6):
    #         Q_sp_compliant[i] =  (F[i] - self.C[i]*self.Q_dot[i] - self.M[i]*self.Q_ddot[i]) / self.K[i]

    #     self.pub_setpoint.publish(cvs.Array2PoseStamped(self.Q_sp + Q_sp_compliant, servo_torques.header.stamp))

    #     if self.k.CheckLims(self.Q_sp + Q_sp_compliant, self.translation_limit, self.rotation_limit):
    #         Theta = self.k.IPK(self.Q_sp + Q_sp_compliant)

    #         dt = rospy.Time.now() - self.time
    #         dt = dt.to_sec() + dt.to_nsec() * 1e-9
    #         self.time = rospy.Time.now()
    #         Q_dot = (Q_sp_compliant - self.Q_sp_compliant) * (1/dt) 
    #         self.Qsp_compliant = Q_sp_compliant
    #         self.Q_ddot = (Q_dot - self.Q_dot) * (1/dt)
    #         self.Q_dot = Q_dot

    #     else:
    #         Theta = np.nan * np.ones(6)
    #         rospy.logwarn('Manipulator workspace exceeded!')


        

    #     if not np.any(np.isnan(Theta)):
    #         self.pub_servo_angles.publish(cvs.Array2ServoAnglesStamped(Theta, servo_torques.header.stamp)) 

    
    # def low_pass_filter(self, data, cutoff):
    #     fs = rospy.get_param('/servo/rate')
    #     fc = cutoff
    #     w = fc / (fs/2)
    #     b, a = signal.butter(5, w, 'low')
        
    #     self.fx.append(data[0])
    #     self.fy.append(data[1])
    #     self.fz.append(data[2])
    #     self.tx.append(data[3])
    #     self.ty.append(data[4])
    #     self.tz.append(data[5])

    #     for i in range(6):
    #         fx_low = signal.lfilter(b,a,self.fx)
    #         fy_low = signal.lfilter(b,a,self.fy)
    #         fz_low = signal.lfilter(b,a,self.fz)
    #         tx_low = signal.lfilter(b,a,self.tx)
    #         ty_low = signal.lfilter(b,a,self.ty)
    #         tz_low = signal.lfilter(b,a,self.tz)

        
    #     return np.asarray([fx_low[-1], fy_low[-1], fz_low[-1], tx_low[-1], ty_low[-1], tz_low[-1]])
    
    # def config_callback(self, config, level):
    #     self.M = np.asarray([config.M_t, config.M_t, config.M_t, config.M_r, config.M_r, config.M_r])
    #     self.C = np.asarray([config.C_t, config.C_t, config.C_t, config.C_r, config.C_r, config.C_r])
    #     self.K = np.asarray([config.K_t, config.K_t, config.K_t, config.K_r, config.K_r, config.K_r])
    #     self.cutoff_frequency = config.low_pass_cutoff
    #     return config


def state_derivative(x1, x0, dt):
    xdot = np.zeros(6)
    xdot[0:3] = (x1[0:3] - x0[0:3]) / dt
    q1 = _euler_to_np_quaternion(x1[3:6])
    q0 = _euler_to_np_quaternion(x0[3:6])
    qdot = _quaternion_derivative(q1, q0, dt)
    xdot[3:6] = _quaternions_to_angular_rate(qdot, q1)
    return xdot

def _euler_to_np_quaternion(angles):
    x, y, z = angles
    q = quaternion_from_euler(x, y, z) #yzx - pitch, yaw, roll
    return np.asarray(q)

def _quaternion_derivative(q1, q0, dt):
    return (q1 - q0)/dt

def _quaternions_to_angular_rate(dqdt, q):
    w = 2 * quaternion_multiply(dqdt, quaternion_conjugate(q))
    return np.asarray(w)[0:3]

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('kinematics_node')
    kN = KinematicsNode()
    rospy.spin()