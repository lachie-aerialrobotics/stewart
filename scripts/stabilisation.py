#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import conversions as cvs
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, TwistStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.msg import State

class Stabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(5.0))
        tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        tfBroadcasterStatic = tf2_ros.StaticTransformBroadcaster()

        tooltip_len = rospy.get_param("nozzle_length")
        tf_tip_to_platform = cvs.Array2TransformStamped(np.asarray([0,0,-tooltip_len,0,0,0]), rospy.Time.now(), frame_id='tooltip_sp', child_frame_id='platform_sp')
        tf_platform_to_tip = cvs.Array2TransformStamped(np.asarray([0,0,tooltip_len,0,0,0]), rospy.Time.now(), frame_id='platform', child_frame_id='tooltip')
        tf_workspace_center_to_tooltip_init = cvs.Array2TransformStamped(np.asarray([0,0,tooltip_len,0,0,0]), rospy.Time.now(), frame_id='workspace_center', child_frame_id='tooltip_init')

        self.mavros_state = 'POSCTL'
        self.tip_pose_msg = PoseStamped()

        # make sure drone is online
        rospy.loginfo("Stabiliser: waiting for mavros...")
        rospy.wait_for_message('/mavros/state', State)

        # send static tfs
        try:
            self.t = self.tfBuffer.lookup_transform('stewart_base', 'workspace_center', time=rospy.Time(0), timeout=rospy.Duration(10))
            t_flat_r = self.tfBuffer.lookup_transform('workspace_center', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
            tf_base_2_manip = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(10))
            tf_manip_2_base = self.tfBuffer.lookup_transform('stewart_base', 'base_link', time=rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Tf frames not initialised :(")

        t_flat = self.t
        t_flat.header.frame_id = 'stewart_base_flat'
        t_flat.child_frame_id = 'workspace_center_flat'
        tfBroadcasterStatic.sendTransform(t_flat)

        tb_sp = t_flat_r
        tb_sp.header.frame_id = 'platform_sp_flat'
        tb_sp.child_frame_id = 'stewart_base_sp'
        tfBroadcasterStatic.sendTransform(tb_sp)

        tf_base_2_manip_sp = tf_manip_2_base
        tf_base_2_manip_sp.header.frame_id = 'stewart_base_sp'
        tf_base_2_manip_sp.child_frame_id = 'base_link_sp'
        tfBroadcasterStatic.sendTransform(tf_base_2_manip_sp)

        tf_base_2_manip_flat = tf_base_2_manip
        tf_base_2_manip_flat.header.frame_id = 'base_link_flat'
        tf_base_2_manip_flat.child_frame_id = 'stewart_base_flat'
        tfBroadcasterStatic.sendTransform(tf_base_2_manip_flat)

        # map_tf = tf_base_2_manip
        # map_tf.transform.translation = Vector3(0,0,0)
        # map_tf.header.frame_id = 'map'
        # map_tf.child_frame_id = 'map_r'
        # tfBroadcasterStatic.sendTransform(map_tf)

        # tooltip_sp_init = TransformStamped()
        # tooltip_sp_init.header.stamp = rospy.Time.now()
        # tooltip_sp_init.child_frame_id = 'tooltip_sp'
        # tooltip_sp_init.header.frame_id = 'map'
        # tooltip_sp_init.transform.rotation = Quaternion(0,0,0,1)


        tfBroadcasterStatic.sendTransform(tf_tip_to_platform)
        tfBroadcasterStatic.sendTransform(tf_platform_to_tip)
        tfBroadcasterStatic.sendTransform(tf_workspace_center_to_tooltip_init)
        # self.tfBroadcaster.sendTransform(tooltip_sp_init)

        #init publishers and subscribers
        self.pub_platform_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_platform_state = rospy.Publisher('/manipulator/state', String, queue_size=1, tcp_nodelay=True)
        # rospy.Subscriber('/manipulator/state', String, self.state_callback, queue_size=1, tcp_nodelay=True) 
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/tooltip_setpoint/pose', PoseStamped, self.tooltip_pose_callback, queue_size=1, tcp_nodelay=True)
        # rospy.Subscriber('/tooltip_setpoint/velocity', TwistStamped, self.tooltip_twist_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=5, tcp_nodelay=True)
        rospy.spin()

    def state_callback(self, state_msg=State()):
        self.mavros_state = state_msg.mode

    def tooltip_pose_callback(self, tip_pose_msg=PoseStamped()):
        if tip_pose_msg.pose.orientation == Quaternion(0,0,0,0):
            tip_pose_msg.pose.orientation = Quaternion(0,0,0,1)

        tip_tf_msg = cvs.PoseStamped2TransformStamped(tip_pose_msg, child_frame_id='tooltip_sp_r')
        self.tfBroadcaster.sendTransform(tip_tf_msg)

        rot = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=tip_pose_msg.header.stamp, timeout=rospy.Duration(0.1))
        flat_ref = self.tfBuffer.lookup_transform('map', 'platform_sp', time=tip_pose_msg.header.stamp, timeout=rospy.Duration(0.1))      
        flat_ref = make_flat_tooltip(flat_ref, rot)
        self.tfBroadcaster.sendTransform(flat_ref)

    # def tooltip_twist_callback(self, tip_vel_msg=TwistStamped()):
    #     self.tip_vel_msg = tip_vel_msg
    #     # this is currently unused :(

    def drone_pose_callback(self, drone_pose_msg=PoseStamped()):
        drone_pose_flat = make_flat_drone(drone_pose_msg)
        drone_transform_flat = cvs.PoseStamped2TransformStamped(drone_pose_flat, child_frame_id='base_link_flat')
        self.tfBroadcaster.sendTransform(drone_transform_flat)


        


        if self.mavros_state == 'OFFBOARD':
            if self.tip_pose_msg.pose.position.z <= 0.0: #theres probably a better criterion for this 
                manip_mode = 'STAB_3DOF'
            else:
                manip_mode = 'STAB_6DOF'
        elif self.mavros_state == 'POSCTL':
            manip_mode = 'STAB_3DOF'
        else:
            manip_mode = 'STATIC'

        self.pub_platform_state.publish(String(manip_mode))

        if manip_mode == "STAB_6DOF":
            target = 'platform_sp'
        elif manip_mode == "STAB_3DOF":
            target = 'workspace_center_flat'
        elif manip_mode == "STATIC":
            target = 'workspace_center'
        else:
            rospy.logwarn('Invalid manipulator state!')
            target = 'workspace_center'

        try:
            self.t = self.tfBuffer.lookup_transform('stewart_base', target, time=drone_pose_msg.header.stamp, timeout=rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Manipulator setpoint tf dropped')

        self.pub_platform_pose.publish(cvs.TransformStamped2PoseStamped(self.t))

def make_flat_drone(pose=PoseStamped()):
    quaternion = np.asarray([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
    (R, P, Y) = euler_from_quaternion(quaternion) 
    quaternion = quaternion_from_euler(0, 0, Y)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose

def make_flat_tooltip(tip_tf, base2stew_tf):
    q1 = np.asarray([tip_tf.transform.rotation.x, tip_tf.transform.rotation.y, tip_tf.transform.rotation.z, tip_tf.transform.rotation.w])
    q2 = np.asarray([base2stew_tf.transform.rotation.x, base2stew_tf.transform.rotation.y, base2stew_tf.transform.rotation.z, base2stew_tf.transform.rotation.w])

    (R1, P1, Y1) = euler_from_quaternion(q1) 

    (R2, P2, Y2) = euler_from_quaternion(q2) 

    # q = quaternion_from_euler(R2,P2,Y2+Y1)

    q = quaternion_from_euler(R2,P2,Y1)

    tip_tf.transform.rotation.x = q[0]
    tip_tf.transform.rotation.y = q[1]
    tip_tf.transform.rotation.z = q[2]
    tip_tf.transform.rotation.w = q[3]

    tip_tf.child_frame_id = 'platform_sp_flat'

    return tip_tf

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('stabilisation')
    S = Stabilisation()