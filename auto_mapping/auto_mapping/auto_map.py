import rclpy
from rclpy.time import Time

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point

from rclpy.action import ActionClient
from auto_mapping.robot import Robot
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import asin, atan2, sin, cos
from threading import Event

# ros2 launch maze_gazebo maze_launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
# ros2 launch slam_toolbox online_async_launch.py


class AutoMap(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.client = ActionClient(self, NavigateToPose, "NavGoal")
        self.goal_nbr = 0
        self.goal = None
        self.robot_pose_img = Point()
        self.target = Point()
        self.call = None
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
        self.get_logger().info("robot initialization finished")

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
            tf = self.tf_buffer.lookup_transform("base_link", "base_link", Time())
            self.robot_pose.x = tf.transform.translation.x
            self.robot_pose.y = tf.transform.translation.y
            robot_rot = tf.transform.rotation
            self.robot_pose.z = self.quaternion_to_euler(robot_rot)[2]
        except LookupException:
            self.get_logger().info('transform not ready')

    def request_map(self):
        self.get_logger().info("Requesting GetMap data")
        self.call = self.map_srv.call_async(GetMap.Request())
        self.call.add_done_callback(self.get_map)

    def get_map(self, response):
        try:
            # response: GetMap.Response = self.call.result()
            self.get_logger().info("GetMap data received")
            response_map = response.result().map
            self.width = response_map.info.width
            self.height = response_map.info.height
            self.resolution = response_map.info.resolution
            origin_yaw = self.quaternion_to_euler(response_map.info.origin.orientation)[2]
            origin_x = response_map.info.origin.position.x
            origin_y = response_map.info.origin.position.y
            self.map_origin = Point(x=origin_x, y=origin_y, z=origin_yaw)
            self.tf_timer = self.create_timer(1.0, self.get_robot_pose)
            self.map_data = list(response_map.data)
            for i in range(self.height):
                for j in range(self.width):
                    self.map_data[i * self.width + j] = response_map.data[(self.height - 1 - i) * self.width + j]
            self.pose_to_pix()
            self.check_directions()
            if not self.find_next_goal():
                self.get_logger().info("No new goal found")
                return
            self.pix_to_pose()
            self.send_target()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))

    def pose_to_pix(self):
        self.robot_pose_img.x = float(round((self.map_origin.y - self.robot_pose.y) / self.resolution + self.height))
        self.robot_pose_img.y = float(round((self.robot_pose.x - self.map_origin.x) / self.resolution))
        self.robot_pose_img.z = float(round(self.robot_pose.z - self.map_origin.z))

    def pix_to_pose(self):
        self.target.x = self.robot_pose_img.y * self.resolution + self.map_origin.x
        self.target.y = -((self.robot_pose_img.x - self.height) * self.resolution - self.map_origin.y)
        self.target.z = self.robot_pose_img.z + self.map_origin.z

    def is_accessible(self, x, y, safe_dist=4):
        if self.map_data[int(x * self.width + y)] == 0:
            for m in range(-safe_dist, safe_dist + 1):
                for n in range(-safe_dist, safe_dist + 1):
                    if self.map_data[int((x + m) * self.width + (y + n))] == 100:
                        return False
            return True
        return False

    def is_free(self, x, y):
        if self.map_data[int(x * self.width + y)] == -2:
            for k in range(-1, 2):
                for l in range(-1, 2):  # On regarde les cellules adjacentes
                    if k != 0 or l != 0:
                        if self.map_data[int((x + k) * self.width + (y + l))] == -1:  # Si une de ces cellules adjacentes est inconnue
                            return True
                    return False
        else:
            return False

    def check_directions(self):
        pile = []
        for i in range(-3, 4):
            for j in range(-3, 4):
                pile.append([int(self.robot_pose_img.x) + i,
                             int(self.robot_pose_img.y) + j])
        while pile:
            [x, y] = pile.pop()
            self.map_data[int(x * self.width + y)] = -2
            for k in range(-1, 2):
                for l in range(-1, 2):
                    if k != 0 or l != 0:
                        if self.is_accessible(x + k, y + l):
                            pile.append([x + k, y + l])

    def find_next_goal(self, rayon=10):
        while 2 * rayon < max(self.height, self.width):
            for i in range(-rayon, rayon + 1):
                if i == -rayon or i == rayon:
                    for j in range(-rayon, rayon + 1):
                        if self.robot_pose_img.x + i < self.width and self.robot_pose_img.y + j < self.height:
                            if self.is_free(self.robot_pose_img.x + i, self.robot_pose_img.y + j):
                                self.target.x = self.robot_pose_img.x + i
                                self.target.y = self.robot_pose_img.y + j
                                return True
                else:
                    if self.robot_pose_img.x + i < self.width and self.robot_pose_img.y + j < self.height:
                        if self.is_free(self.robot_pose_img.x + i, self.robot_pose_img.y + rayon):
                            self.target.x = self.robot_pose_img.x + i
                            self.target.y = self.robot_pose_img.y + rayon
                            return True
                        if self.is_free(self.robot_pose_img.x + i, self.robot_pose_img.y - rayon):
                            self.target.x = self.robot_pose_img.x + i
                            self.target.y = self.robot_pose_img.y - rayon
                            return True
            rayon = rayon + 1
        return False

    def goal_reached(self, resp):
        goal_handle = resp.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.request_map()

    def send_target(self):
        quaternion = self.euler_to_quaternion(0, 0, self.target.z)
        goal = NavigateToPose.Goal()
        self.goal_nbr = self.goal_nbr + 1
        goal.pose.header.seq = self.goal_nbr
        goal.pose.header.stamp = Time()
        goal.pose.header.frame_id = "sendGoal"
        goal.pose.pose.position = Point(x=self.target.x, y=self.target.y, yaw=0)
        goal.pose.pose.orientation.x = quaternion[0]
        goal.pose.pose.orientation.y = quaternion[1]
        goal.pose.pose.orientation.z = quaternion[2]
        goal.pose.pose.orientation.w = quaternion[3]
        self.goal = self.client.send_goal_async(goal)
        self.goal.add_done_callback(self.goal_reached)


def main(args=None):
    rclpy.init(args=args)
    mapper = AutoMap("AutoMapper")
    try:
        mapper.request_map()
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        print("cleaning node")
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
