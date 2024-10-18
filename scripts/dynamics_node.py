#!/usr/bin/env python3


import rospy
import numpy as np
import tf2_ros
import kinematics
import conversions as cvs
from scipy import signal
from geometry_msgs.msg import PoseStamped, WrenchStamped
from stewart.msg import ServoAnglesStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_conjugate, quaternion_multiply, quaternion_matrix

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

        EE_mass = 0.1
        proximal_mass = 0.1
        distal_mass = 0.1
        platform_height = 0.02
        distal_diameter = 0.01
        proximal_diameter = 0.03

        linear_force_limit = 1

        self.EE = cuboid(EE_mass, 2*rp, 2*rp, platform_height)
        self.proximal = []
        self.distal = []
        for i in range(6):
            self.distal.append(cuboid(distal_mass, distal_diameter, distal_diameter, rs))
            self.proximal.append(cuboid(proximal_mass, proximal_diameter, proximal_diameter, ra))

        self.br = tf2_ros.TransformBroadcaster()

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
        EE_CoM_state, proximal_CoM_state, distal_CoM_state, wrist_state, elbow_state  = self.k.CoMs(Q, Theta)

        if not np.any(np.isnan(Theta)):
            self.publish_state_vector_as_tf(EE_CoM_state, pose_msg.header.stamp, 'stewart_base', 'ee_CoM')
            for i in range(6):
                self.publish_state_vector_as_tf(proximal_CoM_state[i], pose_msg.header.stamp, 'stewart_base', 'proximal_link_'+str(i)+'_CoM')
                self.publish_state_vector_as_tf(distal_CoM_state[i], pose_msg.header.stamp, 'stewart_base', 'distal_link_'+str(i)+'_CoM')
                self.publish_state_vector_as_tf(wrist_state[i], pose_msg.header.stamp, 'stewart_base', 'wrist_joint_'+str(i)+'_CoM')
                self.publish_state_vector_as_tf(elbow_state[i], pose_msg.header.stamp, 'stewart_base', 'elbow_joint_'+str(i)+'_CoM')


            EE_F = self.EE.update(EE_CoM_state)
            self.publish_vector_as_wrench(EE_F, pose_msg.header.stamp, 'stewart_base', self.pub_ee_wrench)

            total_F = EE_F

            for i in range(6):
                proximal_F = self.proximal[i].update(proximal_CoM_state[i])
                self.publish_vector_as_wrench(proximal_F, pose_msg.header.stamp, 'stewart_base', self.pub_proximal_wrench[i])
                total_F += proximal_F

                distal_F = self.distal[i].update(distal_CoM_state[i])
                self.publish_vector_as_wrench(distal_F, pose_msg.header.stamp, 'stewart_base', self.pub_distal_wrench[i])
                total_F += distal_F


            self.publish_vector_as_wrench(total_F, pose_msg.header.stamp, 'stewart_base', self.pub_total_wrench)



    
class rigid_body:
    def __init__(self, mass, I_pa):
        self.mass = mass
        self.I_pa = I_pa
        self.t = rospy.Time.now()
        self.state = np.zeros(6)
        self.state_dot = np.zeros(6)
        self.Mom = np.zeros(6)
        self.Fhist= [[],[],[],[],[],[]]
    
    def update(self, new_state):
        # get timestep
        dt = (rospy.Time.now() - self.t).to_sec()
        self.t = rospy.Time.now()

        # update speeds and accelerations
        state_dot, state_ddot = self._derivatives(self.state, self.state_dot, new_state, dt)
        self.state = new_state
        self.state_dot = state_dot
        
        # rotate inertia tensor to local axes
        I = rotate_tensor(self.I_pa, self.state[3:6])

        # compute dynamics
        F = self._dynamics(self.mass, I, state_dot, state_ddot)

        # add moment due to translation from point of interest
        F[3:6] += np.cross(new_state[0:3], F[0:3])

        F = self._6dof_low_pass_filter(F, dt)
        return F
    
    def _6dof_low_pass_filter(self, F, dt):
        fs = 1/dt
        fc = fs/5
        w = fc / (fs/2)
        b, a = signal.butter(5, w, 'low')
        F_low = np.zeros(6)
        
        for i in range(6):
            self.Fhist[i].append(F[i])
            F_low[i] = signal.lfilter(b,a,self.Fhist[i])[-1]
            
        return F_low
    
    def _derivatives(self, x0, xdot0, x1, dt):
        xdot1 = state_deriv(x1, x0, dt)
        xddot = state_deriv(xdot1, xdot0, dt)
        return xdot1, xddot
    
    def _dynamics(self, mass, inertia, v, vdot):
        M = mass * np.eye(3)
        G = np.zeros((6,6))
        G[0:3,0:3] = inertia
        G[3:6,3:6] = M

        # annoyingly the angular and linear terms must be flipped (screw theory convention)
        Vdot = np.zeros(6)
        Vdot[0:3] = vdot[3:6]
        Vdot[3:6] = vdot[0:3]
        V = np.zeros(6)
        V[0:3] = v[3:6]
        V[3:6] = v[0:3]

        advT = np.transpose(lie_bracket(V[0:3], V[3:6]))
        F = np.matmul(G,Vdot) - np.matmul(np.matmul(advT, G), V)

        # flip back to usual convention
        F_flipped = np.zeros(6)
        F_flipped[0:3] = F[3:6]
        F_flipped[3:6] = F[0:3]
        return F_flipped
    
class cuboid(rigid_body):
    def __init__(self, mass, lx, ly, lz):
        I_pa = self._cuboid_inertia(mass, lx, ly, lz)
        super().__init__(mass, I_pa)
    
    def _cuboid_inertia(self, mass, lx, ly, lz):
        inertia = np.eye(3)
        inertia[0,0] = 1/12 * mass * (ly**2 + lz**2)
        inertia[1,1] = 1/12 * mass * (lx**2 + lz**2)
        inertia[2,2] = 1/12 * mass * (lx**2 + ly**2)
        return inertia
    
class shell_cylinder(rigid_body):
    def __init__(self, mass, rxy, lz):
        I_pa = self._shell_cylinder_inertia(mass, rxy, lz)
        super().__init__(mass, I_pa)
    
    def _shell_cylinder_inertia(self, mass, rxy, lz):
        inertia = np.eye(3)
        inertia[0,0] = 1/6 * mass * (3*rxy**2 + lz**2)
        inertia[1,1] = 1/6 * mass * (3*rxy**2 + lz**2)
        inertia[2,2] = mass * rxy**2
        return inertia
    
class solid_cylinder(rigid_body):
    def __init__(self, mass, rxy, lz):
        I_pa = self._solid_cylinder_inertia(mass, rxy, lz)
        super().__init__(mass, I_pa)
    
    def _solid_cylinder_inertia(self, mass, rxy, lz):
        inertia = np.eye(3)
        inertia[0,0] = 1/12 * mass * (3*rxy**2 + lz**2)
        inertia[1,1] = 1/12 * mass * (3*rxy**2 + lz**2)
        inertia[2,2] = 0.5 * mass * rxy**2
        return inertia
    
class point(rigid_body):
    def __init__(self, mass):
        I_pa = np.zeros((3,3))
        super().__init__(mass, I_pa)



def state_deriv(x1, x0, dt):
    xdot = np.zeros(6)
    xdot[0:3] = (x1[0:3] - x0[0:3]) / dt
    q1 = euler_to_np_quaternion(x1[3:6])
    q0 = euler_to_np_quaternion(x0[3:6])
    qdot = quaternion_derivative(q1, q0, dt)
    xdot[3:6] = quaternions_to_angular_rate(qdot, q1)
    return xdot


def euler_to_np_quaternion(angles):
    x, y, z = angles
    q = quaternion_from_euler(x, y, z) #yzx - pitch, yaw, roll
    return np.asarray(q)

def quaternion_to_np_euler(q):
    x, y, z = euler_from_quaternion(q)
    return np.asarray([x, y, z])

def quaternions_to_angular_rate(dqdt, q):
    w = 2 * quaternion_multiply(dqdt, quaternion_conjugate(q))
    return np.asarray(w)[0:3]

def quaternion_derivative(q1, q0, dt):
    return (q1 - q0)/dt

def rotate_tensor(tensor, angles):
    q = euler_to_np_quaternion(angles)
    R = quaternion_matrix(q)[0:3,0:3]
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