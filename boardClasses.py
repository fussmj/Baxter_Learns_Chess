
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class Piece:

    def __init__(self, piece_type, pose):
        self.piece_type = piece_type
        self.pose = pose


class Square:

    def __init__(self, x, y):
        self.x = x
        self.y = y


class Board:

    def __init__(self, x_ref, y_ref, board_len):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.board_len - board_len
        self. squares = []

        for i in range(8):
            self.squares.append([])
            new_x = self.x_ref + i*board_len/8
            for j in range(8):
                new_y = self.y_ref + j*board_len/8
                self.squares.append(Pose(position=Point(x=new_x, y=new_y, z=-0.129), orientation=overhead_orientation))

