import rclpy
from rclpy.node import Node
from dynamixel_controller_v1.msg import MotorSpeeds

class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('speed_publisher')
        self.publisher = self.create_publisher(MotorSpeeds, '/dual_motor_speed', 10)

    def publish_speed(self, speed1, speed2):
        msg = MotorSpeeds()
        msg.motor_speed1 = speed1
        msg.motor_speed2 = speed2
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: Motor1 Speed: "%d", Motor2 Speed: "%d"' % (speed1, speed2))

def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()

    while rclpy.ok():
        try:
            speed_input1 = input("Enter motor 1 speed (integer): ")
            speed1 = int(speed_input1)
            speed_input2 = input("Enter motor 2 speed (integer): ")
            speed2 = int(speed_input2)
            speed_publisher.publish_speed(speed1, speed2)
        except ValueError:
            print("Please enter valid integers.")
        except KeyboardInterrupt:
            break

    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
