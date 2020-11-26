import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import asin, atan2, sin, cos


class Robot(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info("initializing AutoMapper")
        self.call = None
        self.create_timer(1, self.get_map)
        self.map_data = None
        self.width = 0
        self.height = 0
        self.map_origin = Point()
        self.resolution = 0
        self.robot_yaw = 0
        self.robot_pose = Point()
        self.tf_buffer = Buffer()
        self.tf_timer = None
        self.listener = TransformListener(self.tf_buffer, self)
        try:
            self.map_srv = self.create_client(GetMap, "/map_server/map")
            self.get_logger().info("Service client created")
            while not self.map_srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
        except Exception as e:
            self.get_logger().info(e)

    @staticmethod
    def quaternion_to_euler(quaternion):
        t0 = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        t1 = +1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = atan2(t0, t1)
        t2 = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)
        t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        t4 = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = atan2(t3, t4)
        return [yaw, pitch, roll]

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        return [qx, qy, qz, qw]

    def get_robot_pose(self):
        try:
            ((robot_x, robot_y, z), robot_rot) = self.tf_buffer.lookup_transform("/map", "base_link", Time())
            robot_yaw = self.quaternion_to_euler(robot_rot)[2]
            self.robot_pose = {'x': robot_x, 'y': robot_y, 'yaw': robot_yaw}
        except LookupException:
            self.get_logger().info('transform not ready')

    def get_map(self):
        self.get_logger().info("Requesting GetMap data")
        self.call = self.map_srv.call(GetMap.Request())
        if self.call.done():
            try:
                response: GetMap.Response = self.call.result()
                self.get_logger().info("GetMap data received")
                response_map = response.map()
                self.width = response_map.info.width
                self.height = response_map.info.height
                self.resolution = response_map.info.resolution
                origin_yaw = self.quaternion_to_euler(response_map.info.origin.orientation)[2]
                origin_x = response_map.info.origin.position.x
                origin_y = response_map.info.origin.position.y
                self.map_origin = {'x': origin_x, 'y': origin_y, 'yaw': origin_yaw}
                self.tf_timer = self.create_timer(1.0, self.get_robot_pose)
                self.map_data = list(response_map.data)
                for i in range(self.height):
                    for j in range(self.width):
                        self.map_data[i * self.width + j] = response_map.data[(self.height - 1 - i) * self.width + j]
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))

    def pose_to_pix(self):
        pix_x = (self.map_origin.y - self.robot_pose.y) / self.resolution + self.height
        pix_y = (self.robot_pose.x - self.map_origin.x) / self.resolution
        pix_yaw = self.robot_pose.z - self.map_origin.z
        return int(pix_x), int(pix_y), pix_yaw
