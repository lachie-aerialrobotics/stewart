import rclpy
from rclpy.node import Node
import numpy as np
from dynamixel_sdk import *
from stewart_msgs.msg import ServoAnglesStamped
# from dynamic_reconfigure.server import Server
# from dynamixel_interfaces.cfg import ServoConfig

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        self.declare_parameter('servo.baud', 4000000)
        self.declare_parameter('servo.port', '/dev/ttyUSB0')
        self.declare_parameter('servo.num', 6)
        self.declare_parameter('servo.rate', 250)

        self.BAUDRATE = self.get_parameter('servo.baud').value
        self.SERIAL_PORT = self.get_parameter('servo.port').value
        self.NUM_SERVOS = self.get_parameter('servo.num').value
        self.RATE = self.get_parameter('servo.rate').value

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_GOAL_POSITION = 4
        self.LEN_PRESENT_POSITION = 4
        self.PROTOCOL_VERSION = 2.0
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.portHandler = PortHandler(self.SERIAL_PORT)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        self.init_dynamixels()

        self.pos_pub = self.create_publisher(ServoAnglesStamped, '/servo_detected/positions', 10)
        self.create_subscription(ServoAnglesStamped, '/servo_setpoint/positions', self.pos_sp_callback, 10)

        self.timer = self.create_timer(1.0 / self.RATE, self.servo_callback)

    def init_dynamixels(self):
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().info("Failed to open the port")
        
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().info("Failed to change the baudrate")

        for i in range(self.NUM_SERVOS):
            self.packetHandler.write1ByteTxRx(self.portHandler, i+1, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            self.groupSyncRead.addParam(i+1)

    def get_positions(self):
        # Fast Sync Read present position
        dxl_comm_result = groupSyncRead.fastSyncRead()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        dxl_present_position = np.zeros(NUM_SERVOS)
        for i in range(NUM_SERVOS):
            # Check if groupsyncread data of DYNAMIXEL is available
            dxl_getdata_result = groupSyncRead.isAvailable(
                int(i+1),
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                rospy.loginfo("[ID:%03d] groupSyncRead getdata failed" % int(i+1))

            # Get DYNAMIXEL present position value
            dxl_present_position[i] = groupSyncRead.getData(
                int(i+1),
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION)

        #publish measured position
        pos_measured = ServoAnglesStamped()
        pos_measured.header.stamp = rospy.Time.now()
        pos_measured.header.frame_id = "servo"
        for i in range(NUM_SERVOS):
            pos_measured.angles.append(bits2deg(dxl_present_position[i]))

        return pos_measured

    def set_positions(self, pos_msg):
        POS_SP = np.asarray(list(pos_msg.angles))
        for i in range(self.NUM_SERVOS):
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(self.deg2bits(POS_SP[i]))),
                DXL_HIBYTE(DXL_LOWORD(self.deg2bits(POS_SP[i]))),
                DXL_LOBYTE(DXL_HIWORD(self.deg2bits(POS_SP[i]))),
                DXL_HIBYTE(DXL_HIWORD(self.deg2bits(POS_SP[i])))
            ]
        # Add DYNAMIXEL#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(int(i+1), param_goal_position)
        if dxl_addparam_result != True:
            rospy.loginfo("[ID:%03d] groupSyncWrite addparam failed" % int(i+1))

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    def pos_sp_callback(self, pos_msg):
        self.set_positions(pos_msg)
        pos_measured = self.get_positions()
        self.pos_pub.publish(pos_measured)

    def servo_callback(self):
        pos_measured = self.get_positions()
        self.pos_pub.publish(pos_measured)

    @staticmethod
    def bits2deg(bits):
        return float(bits - 2048) * 0.0878906

    @staticmethod
    def deg2bits(deg):
        return int(deg / 0.0878906) + 2048


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()