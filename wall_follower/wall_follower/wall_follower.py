# Ros import
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.time import Time

# Turtlebot import
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# python import
import numpy as np
from numpy import inf
import math
import time
import _thread

# local import
from src.wall_follower.wall_follower.Directions.directions import Directions
from src.wall_follower.wall_follower.Location.location import Location
from src.wall_follower.wall_follower.Distances.distances import Distances


# GOTO 9.5 8.5


class WallFollower(Node):
    def __init__(self):
        super().__init__("WallFollower")
        self.lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.get_lidar,
            qos_profile_sensor_data)
        self.odom = self.create_subscription(Odometry, 'odom', self.set_current_pos, qos_profile_sensor_data)
        self.driver = self.create_publisher(Twist, 'Driver', QoSProfile(depth=10))
        self.current_location = Location()
        self.wall_safe_dist = 1.0
        self.wall_critical_dist = self.wall_safe_dist / 2
        self.dist = Distances(self.wall_safe_dist, self.wall_critical_dist)
        self.nbSection = 8
        self.ranges = []
        self.range = []
        self.tx = 9.5  # maze exit x
        self.ty = 8.5  # maze exit y
        self.closest_point = (None, None)
        self.origin = (None, None)
        self.circumnavigated = False
        self.closest_distance = None  # not sure of init value
        self.left_origin_point = None
        self.is_following_left = None
        self.is_set = False

    def get_lidar(self, msg):
        self.range = np.asarray(msg.ranges, dtype=float)
        self.wall_critical_dist = msg.range_min * 5
        self.wall_safe_dist = self.wall_critical_dist * 2
        if not self.is_set:
            self.is_set = True
            print("critical: ", self.wall_critical_dist)
            print("safe: ", self.wall_safe_dist)
        self.range[~np.isfinite(self.range)] = msg.range_max
        self.range = np.roll(self.range, int(np.floor(360 / self.nbSection / 2)))
        self.ranges = np.split(self.range, self.nbSection)
        self.dist.ranges(self.ranges)

    @staticmethod
    def quaternion_to_euler(quaternion):
        t0 = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        t1 = +1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        t4 = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]

    def set_current_pos(self, msg):
        self.current_location.location = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'angle': self.quaternion_to_euler(msg.pose.pose.orientation)[0] * 180 / math.pi
        }

    def near(self, cx, cy, x, y):
        nearx = x - self.wall_critical_dist <= cx <= x + self.wall_critical_dist
        neary = y - self.wall_critical_dist <= cy <= y + self.wall_critical_dist
        return nearx and neary

    def go_until_obstacle(self):
        print("Going until destination or obstacle")
        is_correct_angle = self.current_location.is_correct_angle(self.tx, self.ty)
        while self.current_location.distance(self.tx, self.ty) > 0.1:
            if (self.dist.distances['front'] <= self.wall_safe_dist or
                    self.dist.distances['frontleft'] <= self.wall_safe_dist or
                    self.dist.distances['frontright'] <= self.wall_safe_dist):
                print("going to hit a wall at dist", min([self.dist.distances['front'], self.dist.distances['frontleft'], self.dist.distances['frontright']]))
                return True
            if self.current_location.is_correct_angle(self.tx, self.ty):
                self.forward()
            elif self.current_location.is_left(self.tx, self.ty):
                self.go_left()
            else:
                self.go_right()
            time.sleep(.01)
        return False

    def follow_wall(self):  # refactor using goal direction heuristic
        print("Following wall")
        while self.is_wall():
            dist = self.dist.distances
            if (self.dist.average(['frontleft', 'left', 'backleft'], asum=True) >
                    self.dist.average(['frontright', 'right', 'backright'], asum=True)):  # following right wall
                if self.is_following_left is None:
                    print("found wall at right side")
                    self.is_following_left = False
                if self.dist.average(['front', 'right'], asum=True) < 1.5:  # corner found
                    self.turn_left()
                elif dist['frontright'] > dist['backright']:
                    self.go_right()
                elif dist['frontright'] < dist['backright']:
                    self.go_left()
                else:
                    self.forward()
            elif (self.dist.average(['frontleft', 'left', 'backleft'], asum=True) <
                    self.dist.average(['frontright', 'right', 'backright'], asum=True)):  # following left wall
                if self.is_following_left is None:
                    print("found wall at left side")
                    self.is_following_left = True
                if self.dist.average(['front', 'frontleft', 'left'], asum=True) < 1.5:  # corner found
                    self.turn_right()
                elif dist['frontleft'] > dist['backleft']:
                    self.go_left()
                elif dist['frontright'] < dist['backright']:
                    self.go_right()
                else:
                    self.forward()
            time.sleep(.01)

    def search_lost_wall(self):
        print("looking for lost wall")
        self.stop()
        if self.is_following_left:
            while self.dist.average('frontleft') > self.dist.average('left'):
                if self.dist.is_critical('frontleft'):
                    self.forward()
                else:
                    self.turn_left()
            print("think wall found at left side, going toward this destination")
            self.forward()
            time.sleep(.1)
        elif not self.is_following_left:
            while self.dist.average('frontright') > self.dist.average('right'):
                if self.dist.is_critical('frontright'):
                    self.forward()
                else:
                    self.turn_right()

            print("think wall found at right side, going toward this destination")
            self.forward()
            time.sleep(.1)
        else:
            self.forward()

    def is_wall(self):
        if self.is_following_left:
            return self.dist.average(['frontleft', 'left', 'backleft'], asum=True) < 2.0
        elif not self.is_following_left:
            return self.dist.average(['frontright', 'right', 'backright'], asum=True) < 2.0
        else:
            print("no wall follow")
            self.forward()
            return False

    def turn_left(self):
        self.driver.publish(Directions.LEFT.value)

    def turn_right(self):
        self.driver.publish(Directions.RIGHT.value)

    def forward(self):
        self.driver.publish(Directions.FORWARD.value)

    def backward(self):
        self.driver.publish(Directions.BACKWARD.value)

    def stop(self):
        self.driver.publish(Directions.STOP.value)

    def go_left(self):
        self.driver.publish(Directions.FRONTLEFT.value)

    def go_right(self):
        self.driver.publish(Directions.FRONTRIGHT.value)


def main(args=None):
    rclpy.init(args=args)
    clk = False

    def setClk():
        nonlocal clk
        clk = True
    timer = Node.create_subscription(Time, 'clock', setClk)
    _thread.start_new_thread(rclpy.spin, (timer, None))
    while not clk:
        time.sleep(.1)
    follower = WallFollower()
    #rclpy.spin(follower)
    print("waiting for clock time to init")

    print("received clock init")
    _thread.start_new_thread(rclpy.spin, (follower, None))
    time.sleep(2)
    while follower.current_location.distance(9.5, 8.5) > .1:
        hit_wall = follower.go_until_obstacle()
        if hit_wall:
            follower.follow_wall()
            follower.search_lost_wall()
    print("Arrived at", (9.5, 8.5))
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
