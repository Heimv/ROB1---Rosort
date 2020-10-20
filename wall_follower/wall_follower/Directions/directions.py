from geometry_msgs.msg import Vector3, Twist
from enum import Enum

FORWARD_VEL = 0.25
FORWARD_VEL_TURN = 0.1
ROTATE_VEL = 0.1


class Directions(Enum):
    STOP = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    FORWARD = Twist(linear=Vector3(x=FORWARD_VEL, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    BACKWARD = Twist(linear=Vector3(x=-FORWARD_VEL, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    LEFT = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=ROTATE_VEL))
    RIGHT = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-ROTATE_VEL))
    FRONTLEFT = Twist(linear=Vector3(x=FORWARD_VEL_TURN, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=ROTATE_VEL))
    FRONTRIGHT = Twist(linear=Vector3(x=FORWARD_VEL, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-ROTATE_VEL))

