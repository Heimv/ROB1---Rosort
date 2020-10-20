import threading
import math
import sys


class Location:
    def __init__(self):
        self.mutex = threading.Lock()
        self.location = {
            'x': sys.maxsize,
            'y': sys.maxsize,
            'angle': sys.maxsize
        }
        self.deltaAngle = 0.1  # Acceptable value to requested angle

    def distance(self, x, y):
        if self.location['x'] == sys.maxsize or self.location['y'] == sys.maxsize:
            # will be unset on the first iteration
            return sys.maxsize
        return math.sqrt((x - self.location['x']) ** 2 + (y - self.location['y']) ** 2)

    def is_correct_angle(self, x, y):
        if sys.maxsize in (self.location['x'], self.location['y'], self.location['angle']):
            # will be unset on the first iteration
            return False
        angle = self.to_angle(x, y)
        return angle - self.deltaAngle <= self.location['angle'] <= angle + self.deltaAngle

    def is_left(self, x, y):
        if sys.maxsize in (self.location['x'], self.location['y'], self.location['angle']):
            # will be unset on the first iteration
            return False
        return self.to_angle(x, y) < 0

    def global_to_local(self, angle):
        ans = angle - self.angle
        if ans < -math.pi:
            ans += 2 * math.pi
        return ans

    def to_angle(self, x, y):
        return math.atan2(y - self.location['y'], x - self.location['x'])
