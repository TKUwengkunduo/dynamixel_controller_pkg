import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('speed_publisher')
        self.publisher = self.create_publisher(Int32, '/motor_speed', 10)

    def publish_speed(self, speed):
        msg = Int32()
        msg.data = speed
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()

    while rclpy.ok():
        try:
            speed_input = input("Enter motor speed (integer): ")
            speed = int(speed_input)
            speed_publisher.publish_speed(speed)
        except ValueError:
            print("Please enter a valid integer.")
        except KeyboardInterrupt:
            break

    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
