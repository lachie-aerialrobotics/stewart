#!/usr/bin/env python3

import rospy
import numpy as np
from stewart.msg import ServoAnglesStamped
from dynamixel_sdk import *
from dynamic_reconfigure.server import Server
from stewart.cfg import ServoConfig

class ServoController:
    # Control table address
    ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_LED_RED            = 65
    ADDR_GOAL_POSITION      = 116
    ADDR_GOAL_VELOCITY      = 104
    ADDR_PRESENT_POSITION   = 132
    ADDR_POS_P_GAIN         = 84
    ADDR_POS_I_GAIN         = 82
    ADDR_POS_D_GAIN         = 80
    ADDR_VEL_LIMIT          = 44
    ADDR_PRESENT_CURRENT    = 126

    # Data Byte Length
    LEN_LED_RED             = 1
    LEN_GOAL_VELOCITY       = 4
    LEN_PRESENT_POSITION    = 4
    LEN_GOAL_POSITION       = 4
    LEN_PRESENT_CURRENT     = 2
    
    TORQUE_ENABLE           = 1                 # Value for enabling the torque
    TORQUE_DISABLE          = 0                 # Value for disabling the torque
    NUM_SERVOS              = 6
    
    def __init__(self):
        # Protocol version
        PROTOCOL_VERSION    = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        BAUDRATE            = rospy.get_param('/servo/baud')         # Dynamixel default baudrate : 57600
        SERIAL_PORT         = rospy.get_param('/servo/port')                # Check which port is being used on your controller
        
        self.POS_SP = np.zeros(self.NUM_SERVOS)
        
        self.initialise(SERIAL_PORT, PROTOCOL_VERSION, BAUDRATE, self.NUM_SERVOS)

        Server(ServoConfig, self.config_callback)
        
        self.pos_pub = rospy.Publisher('/servo_detected/positions', ServoAnglesStamped, queue_size=1, tcp_nodelay=True) # servo angle publisher
        self.torque_pub = rospy.Publisher('/servo_detected/torques', ServoAnglesStamped, queue_size=1, tcp_nodelay=True) # servo torque publisher
        rospy.Subscriber('/servo_setpoint/positions', ServoAnglesStamped, self.pos_sp_callback, queue_size=1, tcp_nodelay=True) # target angle subscriber

        RATE                = rospy.get_param('/servo/rate')
        rospy.Timer(rospy.Duration(1.0/RATE), self.servo_callback)

    def initialise(self, serial_port, protocol_version, baudrate, num):
        rospy.loginfo("INITIALISING DYNAMIXELS.......")
        
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(serial_port)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(protocol_version)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            self.ADDR_GOAL_POSITION,
            self.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead_pos = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION)
        
        # Initialize GroupSyncRead instace for Present Current
        self.groupSyncRead_current = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            self.ADDR_PRESENT_CURRENT,
            self.LEN_PRESENT_CURRENT)
        
        # Open port
        if self.portHandler.openPort():
            rospy.loginfo("Succeeded to open the port")
        else:
            rospy.loginfo("Failed to open the port")

        # Set port baudrate
        if self.portHandler.setBaudRate(baudrate):
            rospy.loginfo("Succeeded to change the baudrate")
        else:
            rospy.loginfo("Failed to change the baudrate")

        for i in range(num):
            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, 
                int(i+1), 
                self.ADDR_TORQUE_ENABLE, 
                self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.loginfo("Dynamixel#%d has been successfully connected" % int(i+1))

            # Add parameter storage for Dynamixel present position
            dxl_addparam_result = self.groupSyncRead_pos.addParam(i+1)
            if dxl_addparam_result != True:
                rospy.loginfo("[ID:%03d] groupSyncRead addparam failed" % int(i+1))

            # Add parameter storage for Dynamixel present current
            dxl_addparam_result = self.groupSyncRead_current.addParam(i+1)
            if dxl_addparam_result != True:
                rospy.loginfo("[ID:%03d] groupSyncRead addparam failed" % int(i+1))

    def close(self):
        # Clear syncread parameter storage
        self.groupSyncRead_pos.clearParam()
        self.groupSyncRead_current.clearParam()

        # Disable Dynamixel Torque - commented to prevent propeller collisions
        # for i in range(self.NUM_SERVOS):
        #     dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, int(i+1), self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()

    def simple_write(self, value, addr, len, num, port_handler, packet_handler):
            for i in range(num):
                if len == 1:
                    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, int(i+1), addr, int(value))
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.loginfo("%s" % packet_handler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        rospy.loginfo("%s" % packet_handler.getRxPacketError(dxl_error))
                elif len == 2:
                    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, int(i+1), addr, int(value))
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.loginfo("%s" % packet_handler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        rospy.loginfo("%s" % packet_handler.getRxPacketError(dxl_error))
                elif len == 4:
                    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, int(i+1), addr, int(value))
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.loginfo("%s" % packet_handler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        rospy.loginfo("%s" % packet_handler.getRxPacketError(dxl_error))
                else:
                    rospy.logwarn("Invalid packet length!")

    def config_callback(self, config, level): 
        #update config values and trigger sending values to servos in main timer callback
        self.settings_changed = True
        self.config = config
        # self.no_load_current = ...
        # self.torque_per_amp = ...
        return config

    def sync_write_data(self, value, group_sync_write, packet_handler):
        if not np.any(np.isnan(value)):
            for i in range(self.NUM_SERVOS):
                # Allocate goal position value into byte array
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(value[i])),
                    DXL_HIBYTE(DXL_LOWORD(value[i])),
                    DXL_LOBYTE(DXL_HIWORD(value[i])),
                    DXL_HIBYTE(DXL_HIWORD(value[i]))]

                # Add DYNAMIXEL#i goal position value to the Syncwrite parameter storage
                dxl_addparam_result = group_sync_write.addParam(int(i+1), param_goal_position)
                if dxl_addparam_result != True:
                    rospy.loginfo("[ID:%03d] groupSyncWrite addparam failed" % int(i+1))

                # Syncwrite goal position
                dxl_comm_result = group_sync_write.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.loginfo("%s" % packet_handler.getTxRxResult(dxl_comm_result))

                # Clear syncwrite parameter storage
                group_sync_write.clearParam()
        else:
            rospy.logwarn("Invalid setpoint received")

    def sync_read_data(self, addr, len, num, group_sync_read, packet_handler):
        # Fast Sync Read
        dxl_comm_result = group_sync_read.fastSyncRead()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packet_handler.getTxRxResult(dxl_comm_result))
        data = []
        for i in range(num):
            # Check if groupsyncread data of DYNAMIXEL is available
            dxl_getdata_result = group_sync_read.isAvailable(int(i+1), addr, len)
            if dxl_getdata_result != True:
                rospy.loginfo("[ID:%03d] groupSyncRead getdata failed" % int(i+1))
            # Get DYNAMIXEL present value
            data.append(group_sync_read.getData(int(i+1), addr, len))
        return data

    def pos_sp_callback(self, msg):
        #update position setpoint and save as array
        self.POS_SP = np.asarray(list(msg.Theta))
              
    def servo_callback(self, event):
        # #read measured position
        dxl_present_position = self.sync_read_data(self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION, 6, self.groupSyncRead_pos, self.packetHandler)
        # publish measured position
        self.pos_pub.publish(self._vector2ServoAnglesStamped(self._bits2deg(dxl_present_position)))

        # read measured current
        dxl_present_current = self.sync_read_data(self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT, 6, self.groupSyncRead_current, self.packetHandler)
        # two's complement fix
        for i in range(self.NUM_SERVOS):
            if dxl_present_current[i] > 0x7fff:
                dxl_present_current[i] = dxl_present_current[i] - 65536
        #publish measured current
        self.torque_pub.publish(self._vector2ServoAnglesStamped(self._bits2torque(dxl_present_current)))

        # print(self.POS_SP)
        
        #write position setpoint to servos
        self.sync_write_data(self._deg2bits(self.POS_SP), self.groupSyncWrite, self.packetHandler)

        #change servo settings if needed
        if self.settings_changed:
            self.settings_changed = False
            self.simple_write(self.TORQUE_DISABLE, self.ADDR_TORQUE_ENABLE, 1, 6, self.portHandler, self.packetHandler)
            self.simple_write(self.config.P,         self.ADDR_POS_P_GAIN, 2, 6, self.portHandler, self.packetHandler)
            self.simple_write(self.config.I,         self.ADDR_POS_I_GAIN, 2, 6, self.portHandler, self.packetHandler)
            self.simple_write(self.config.D,         self.ADDR_POS_D_GAIN, 2, 6, self.portHandler, self.packetHandler)
            self.simple_write(self.config.vel_limit, self.ADDR_VEL_LIMIT, 4, 6, self.portHandler, self.packetHandler)
            self.simple_write(self.TORQUE_ENABLE, self.ADDR_TORQUE_ENABLE, 1, 6, self.portHandler, self.packetHandler)

        

    def _vector2ServoAnglesStamped(self, vector):
        msg = ServoAnglesStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "servo"
        for i in range(len(vector)):
            msg.Theta.append(vector[i])
        return msg

    def _bits2deg(self, bits):
        deg = []
        for i in range(len(bits)):
            deg.append(float(bits[i] - 2048) * 0.0878906)
        return deg

    def _deg2bits(self, deg):
        bits = []
        for i in range(len(deg)):
            bits.append(int(deg[i] / 0.0878906) + 2048)
        return bits

    def _bits2torque(self, bits):
        torque = []
        torque_per_amp = 1.0
        no_load_current = 0.04
        for i in range(len(bits)):
            # convert current in bits to amps
            amps = -float(bits[i]) * 0.001

            # convert amps to torque
            if amps > no_load_current:
                torque.append(-no_load_current + torque_per_amp * amps)
            elif amps < no_load_current:
                torque.append(no_load_current + torque_per_amp * amps)
            else: 
                torque.append(0.0) 
            #todo: update this model experimentally
        return torque
    
if __name__ == '__main__':
    rospy.init_node('servo_controller', anonymous=True)
    sc = ServoController()
    rospy.spin()
    sc.close()