import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

import math


class Driver(Node):
    def __init__(self):
        super().__init__("Driver")
        self.get_cmd = self.create_subscription(Twist, 'Driver', self.send, qos_profile_sensor_data)
        self.cmd_publishers = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.prev_dir = None
        self.dir_stack = 0

    def stack_twist(self, direction: Twist):
        if self.dir_stack == 0:
            return direction
        direction.linear.x *= self.dir_stack
        direction.angular.z *= self.dir_stack
        return direction

    def send(self, direction):
        if self.prev_dir == direction:
            self.dir_stack += 1
        else:
            self.dir_stack = 0
        self.prev_dir = direction
        #TOOD: add security limits here and further Driver usage
        self.cmd_publishers.publish(self.stack_twist(direction))


def main(args=None):
    try:
        rclpy.init(args=args)
        driver = Driver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        print("cleaning node")
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
