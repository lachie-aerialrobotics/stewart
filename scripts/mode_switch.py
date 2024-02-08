#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
    
class ManipulatorState:
    def __init__(self):
        self.pub_tooltip_state = rospy.Publisher('/manipulator/state',  String, queue_size=1, tcp_nodelay=True)
        rospy.wait_for_message('/mavros/state', State)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5, tcp_nodelay=True)
        rospy.spin()

    def state_cb(self, state_msg):
        if state_msg.armed:
            if state_msg.mode == 'OFFBOARD':
                self.pub_tooltip_state.publish(String("STAB_6DOF"))
            elif state_msg.mode == 'POSCTL':
                self.pub_tooltip_state.publish(String("STAB_3DOF"))
            else:
                self.pub_tooltip_state.publish(String("STATIC"))
        else:
            self.pub_tooltip_state.publish(String("STATIC"))

if __name__ == "__main__":
    rospy.init_node('manipulator_state_publisher')
    mS = ManipulatorState()