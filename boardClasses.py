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

import moveClasses

class Piece:

    def __init__(self, pose, type):
        self.pose = pose


class Square:

    def __init__(self, pose, x, y):
        self.cal = int(pose[1])
        self.pose = pose
        self.x = x
        self.y = y
        self.z_pawn = -0.183 - self.cal
        self.z_king = -0.164 - self.cal
        self.z_queen = -0.171 - self.cal
        self.z_knight = -0.176 - self.cal
        self.z_bishop = -0.181 - self.cal
        self.z_rook = -0.176 - self.cal
        self.heights = {'pawn': self.z_pawn,
                        'king': self.z_king,
                        'queen': self.z_queen,
                        'knight': self.z_knight,
                        'bishop': self.z_bishop,
                        'rook': self.z_rook}

class Board:

    def __init__(self, x_ref, y_ref, board_length):

        #the x and y ref variables should pertain to the coordinates that the gripper has to be in to reach spot a1 on the board
        self.x_ref = x_ref
        self.y_ref = y_ref
        hover_distance = 0.15
        limbR = 'right'
        self.pnp = moveClasses.PickAndPlace(limbR, hover_distance)
        self.board_length = board_length
        self.square_length = self.board_length/8
        self.squares = []
        self.pieces = []
        self.overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
        self.camera_view_orientation = Quaternion(
            x=-0.994,
            y=-0.098,
            z=-0.037,
            w=-0.027
        )
        self.camera_view_x = 0.544
        self.camera_view_y = -0.438
        self.camera_view_z = 0.334
        self.camera_view_pose = Pose(position=Point(x=self.camera_view_x, y=self.camera_view_y, z=-self.camera_view_z), orientation=self.camera_view_orientation)
        self.dumpster_pose = Pose(position=Point(x=x_ref - 0.2, y=y_ref - 0.3, z=-0.17), orientation=self.overhead_orientation)

        rows = ['1', '2', '3', '4', '5', '6', '7', '8']
        cols = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

        for i in range(8):
            for j in range(8):
                self.squares.append(Square((cols[i] + rows[j]), x_ref - j * self.square_length, y_ref + i * self.square_length ))


        for i in range(8):
            self.pieces.append(Piece(cols[i] + '1'))
            self.pieces.append(Piece(cols[i] + '2'))
            self.pieces.append(Piece(cols[i] + '7'))
            self.pieces.append(Piece(cols[i] + '8'))

    def move_player_piece(self, old_pose, new_pose):
        for piece in self.pieces:
            if piece.pose == new_pose:
                del piece
        for piece in self.pieces:
            if piece.pose == old_pose:
                piece.pose = new_pose


    def move_robot_piece(self, old_pose, new_pose):
        for piece in self.pieces:
            if piece.pose == new_pose:
                for square in self.squares:
                    if square.pose == piece.pose:
                        x_pose = square.x
                        y_pose = square.y
                        z_pose = square.heights[piece.type]
                taken_piece_pose = Pose(
                    position=Point(x=x_pose, y=y_pose, z=z_pose),
                    orientation=self.overhead_orientation)
                self.pnp.pick(taken_piece_pose)
                self.pnp.place(self.dumpster_pose)
                del piece
        for piece in self.pieces:
            if piece.pose == old_pose:
                piece.pose = new_pose
                for square in self.squares:
                    if square.pose == old_pose:
                        old_x = square.x
                        old_y = square.y
                        old_z = square.heights[piece.type]
                    elif square.pose == new_pose:
                        new_x = square.x
                        new_y = square.y
                        new_z = square.heights[piece.type]
                pick_pose = Pose(position=Point(x=old_x, y=old_y, z=old_z), orientation=self.overhead_orientation)
                place_pose = Pose(position=Point(x=new_x, y=new_y, z=new_z), orientation=self.overhead_orientation)
                self.pnp.pick(pick_pose)
                self.pnp.place(place_pose)
                self.pnp._servo_to_pose(self.camera_view_pose)







