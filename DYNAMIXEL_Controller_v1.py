import rclpy
from rclpy.node import Node
from dynamixel_sdk import *                    
from std_msgs.msg import Int32
from dynamixel_controller_pkg.msg import MotorSpeeds

# Protocol version
PROTOCOL_VERSION = 1.0      

# Control table address
ADDR_MX_TORQUE_ENABLE = 24                     
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_MOVING_SPEED = 32                      # Address for moving speed

# Default setting
DXL_ID1 = 1
DXL_ID2 = 2  
BAUDRATE = 1000000                               
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 100               
DXL_MAXIMUM_POSITION_VALUE = 4000              
DXL_MOVING_STATUS_THRESHOLD = 20               
DXL_MOVING_SPEED_VALUE = 1000                   # Speed for the motor

'''=======================================================================================================
For controlling a single motor
======================================================================================================='''
class SingleMotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()
        self.speed_subscriber = self.create_subscription(Int32, '/motor_speed', self.speed_callback, 10)

    def init_dynamixel(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_moving_speed(self, speed):
        if speed<0:
            speed = int((-speed/255)*1023 + 1023)
        else:
            speed = int((speed/255)*1023)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, speed)
    
    def pin(self):
        m1_model_number, m1_comm_result, m1_error = self.packetHandler.ping(self.portHandler, DXL_ID1)
        if m1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m1_comm_result))
        elif m1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m1_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID1, m1_model_number))

    def run(self):
        # Set initial moving speed
        self.set_moving_speed(DXL_MOVING_SPEED_VALUE)

        # while rclpy.ok():
        #     # Write goal position
        #     self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, DXL_MINIMUM_POSITION_VALUE)
        #     rclpy.spin_once(self)
    
    def speed_callback(self, msg):
        # Set the received speed
        self.set_moving_speed(msg.data)



'''=======================================================================================================
For controlling two motors
======================================================================================================='''
class DualMotorController(Node):
    def __init__(self):
        super().__init__('dual_motor_controller')
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()
        self.speed_subscriber = self.create_subscription(MotorSpeeds, '/dual_motor_speed', self.speed_callback, 10)


    def init_dynamixel(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Enable torque for both motors
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_moving_speed(self, motor_id, speed):
        # Set moving speed for both motors
        if speed<0:
            speed = int((-speed/255)*1023 + 1023)
        else:
            speed = int((speed/255)*1023)
        print(motor_id, speed)
        self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, ADDR_MX_MOVING_SPEED, speed)
    
    def speed_callback(self, msg):
        # Set the received speed for both motors
        self.set_moving_speed(DXL_ID1, msg.motor_speed1)
        self.set_moving_speed(DXL_ID2, msg.motor_speed2)

    def pin(self):
        m1_model_number, m1_comm_result, m1_error = self.packetHandler.ping(self.portHandler, DXL_ID1)
        if m1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m1_comm_result))
        elif m1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m1_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID1, m1_model_number))

        m2_model_number, m2_comm_result, m2_error = self.packetHandler.ping(self.portHandler, DXL_ID2)
        if m2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m2_comm_result))
        elif m2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m2_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID2, m1_model_number))



def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = DualMotorController()
    dynamixel_controller.pin()
    rclpy.spin(dynamixel_controller)  # 使用 spin 而非 run
    dynamixel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
