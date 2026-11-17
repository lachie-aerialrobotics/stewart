import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from stewart_msgs.msg import ServoAnglesStamped

# Actuated joints in stewart_platform.sdf, in servo order 1..6
SERVO_JOINTS = ['proximal_link_' + str(i + 1) + '_joint' for i in range(6)]


class ServoConverterNode(Node):
    def __init__(self):
        super().__init__('stewart_sim_msg_converter')

        # Setpoints: ServoAnglesStamped [deg] -> gz JointPositionController /servoN/cmd [rad]
        self.pub_servo = [None] * 6
        for i in range(6):
            self.pub_servo[i] = self.create_publisher(Float64, '/servo'+str(i+1)+'/cmd', 10)
        self.create_subscription(ServoAnglesStamped, '/servo_setpoint/positions', self.cb, 10)

        # Feedback: gz JointState [rad] -> ServoAnglesStamped [deg], same as the real servo driver
        self.pub_detected = self.create_publisher(ServoAnglesStamped, '/servo_detected/positions', 10)
        self.create_subscription(JointState, '/stewart/servo_state', self.state_cb, 10)

    def cb(self, msg: ServoAnglesStamped):
        # ServoAnglesStamped carries degrees (see conversions.Array2ServoAnglesStamped);
        # gz joint position controllers expect radians.
        for i in range(6):
            servo_angle = float(np.deg2rad(msg.angles[i]))
            self.pub_servo[i].publish(Float64(data=servo_angle))

    def state_cb(self, msg: JointState):
        # Joint names may be scoped (e.g. 'stewart_platform::proximal_link_1_joint') when nested
        angles = []
        for joint in SERVO_JOINTS:
            idx = next((k for k, name in enumerate(msg.name)
                        if name == joint or name.endswith('::' + joint)), None)
            if idx is None or idx >= len(msg.position):
                return
            angles.append(float(np.rad2deg(msg.position[idx])))
        out = ServoAnglesStamped()
        out.header = msg.header
        out.angles = angles
        self.pub_detected.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ServoConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
