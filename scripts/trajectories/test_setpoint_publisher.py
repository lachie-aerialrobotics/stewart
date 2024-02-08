#! /usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler

from dynamic_reconfigure.server import Server
from delta_2.cfg import TestSetpointConfig

def config_callback(config, level): 
    q = quaternion_from_euler(np.deg2rad(config.psi), np.deg2rad(config.theta), np.deg2rad(config.phi))

    setpoint = PoseStamped()
    setpoint.header.frame_id = "base"
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.orientation.x = q[0]
    setpoint.pose.orientation.y = q[1]
    setpoint.pose.orientation.z = q[2]
    setpoint.pose.orientation.w = q[3]
    setpoint.pose.position.x = config.x
    setpoint.pose.position.y = config.y
    setpoint.pose.position.z = config.z

    setpoint_pub.publish(setpoint)
    rospy.loginfo("Updated position!")

    vel_setpoint = TwistStamped()
    vel_setpoint.header.frame_id = "base"
    vel_setpoint.header.stamp = rospy.Time.now()
    vel_setpoint.twist.angular.x = np.deg2rad(config.psi_dot)
    vel_setpoint.twist.angular.y =  np.deg2rad(config.theta_dot)
    vel_setpoint.twist.angular.z =  np.deg2rad(config.phi_dot)
    vel_setpoint.twist.linear.x = config.x_dot
    vel_setpoint.twist.linear.y = config.y_dot
    vel_setpoint.twist.linear.z = config.z_dot

    vel_setpoint_pub.publish(vel_setpoint)
    rospy.loginfo("Updated velocity!")

    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('test_setpoint_publisher')
    setpoint_pub = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
    vel_setpoint_pub = rospy.Publisher('/platform_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
    srv = Server(TestSetpointConfig, config_callback)
    rospy.spin()