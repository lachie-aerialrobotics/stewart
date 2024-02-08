#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout
from delta_2.msg import ServoAnglesStamped

class SimConverter:
    def __init__(self):
        self.rate = rospy.get_param('/servo/rate')
        self.pos_sp = np.zeros(rospy.get_param('/servo/num'))

        #init publisher and subscriber
        model_name = rospy.get_param('model_name')
        self.pub_servo_angles = rospy.Publisher('/' + model_name + '/position_cmd', Float32MultiArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/servo_setpoint/positions', ServoAnglesStamped, self.pos_callback, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0/self.rate), self.servo_callback)

    def servo_callback(self, event):
        angles = np.deg2rad(self.pos_sp).tolist()
        self.pub_servo_angles.publish(Float32MultiArray(MultiArrayLayout(), angles))
    
    def pos_callback(self, platform_state): #callback calculates servo angles
        self.pos_sp = np.asarray(platform_state.Theta)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('publish_sim_joints')
    sc = SimConverter()
    rospy.spin()