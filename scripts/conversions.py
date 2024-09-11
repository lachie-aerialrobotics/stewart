import numpy as np

from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped
from stewart.msg import ServoAnglesStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion    


def Array2PoseStamped(Q, stamp, frame_id = 'stewart_base'):
    q = quaternion_from_euler(Q[3], Q[4], Q[5])
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = Q[0]
    msg.pose.position.y = Q[1]
    msg.pose.position.z = Q[2]
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg

def Array2TransformStamped(Q, stamp, frame_id = 'stewart_base', child_frame_id='platform'):
    q = quaternion_from_euler(Q[3], Q[4], Q[5])
    msg = TransformStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = Q[0]
    msg.transform.translation.y = Q[1]
    msg.transform.translation.z = Q[2]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg

def Array2WrenchStamped(F, stamp, frame_id = 'stewart_base'):
    msg = WrenchStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.wrench.force.x = F[0]
    msg.wrench.force.y = F[1]
    msg.wrench.force.z = F[2]
    msg.wrench.torque.x = F[3]
    msg.wrench.torque.y = F[4]
    msg.wrench.torque.z = F[5]
    return msg

def Array2ServoAnglesStamped(Theta, stamp):
    msg = ServoAnglesStamped()
    msg.header.frame_id = "servo"
    msg.header.stamp = stamp
    for i in range(6):
        msg.Theta.append(np.rad2deg(Theta[i]))
    return msg

def ServoAnglesStamped2Array(msg:ServoAnglesStamped):
    Theta = np.deg2rad(np.asarray(msg.Theta))
    return Theta

def PoseStamped2Array(msg:PoseStamped):
    (theta, phi, psi) = euler_from_quaternion([msg.pose.orientation.x, 
                                                    msg.pose.orientation.y, 
                                                    msg.pose.orientation.z, 
                                                    msg.pose.orientation.w])

    return np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, theta, phi, psi]) 

def TransformStamped2PoseStamped(msg:TransformStamped):
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.pose.position = msg.transform.translation
    pose_msg.pose.orientation = msg.transform.rotation
    return pose_msg

def PoseStamped2TransformStamped(msg:PoseStamped, child_frame_id:str):
    tf_msg = TransformStamped()
    tf_msg.header = msg.header
    tf_msg.child_frame_id = child_frame_id
    tf_msg.transform.translation = msg.pose.position
    tf_msg.transform.rotation = msg.pose.orientation
    return tf_msg