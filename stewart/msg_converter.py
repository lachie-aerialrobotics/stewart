import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from stewart_msgs.msg import ServoAnglesStamped

class ServoConverterNode(Node):
    def __init__(self):
        super().__init__('stewart_sim_msg_converter')

        self.pub_servo = [None] * 6
        
        for i in range(6):
            self.pub_servo[i] = self.create_publisher(Float64, '/servo'+str(i+1)+'/cmd', 10)
            
        self.create_subscription(ServoAnglesStamped, '/servo_setpoint/positions', self.cb, 10)

    def cb(self, msg:ServoAnglesStamped):
        for i in range(6):
            servo_angle = msg.angles[i]
            self.pub_servo[i].publish(Float64(data=servo_angle))

def main(args=None):
    rclpy.init(args=args)
    node = ServoConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
