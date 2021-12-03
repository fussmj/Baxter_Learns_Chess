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

    def __init__(self, pose):
        self.pose = pose


class Square:

    def __init__(self, pose, x, y):
        self.pose = pose
        self.x = x
        self.y = y

class Board:

    def __init__(self, x_ref, y_ref, board_length):
        #the x and y ref variables should pertain to the coordinates that the gripper has to be in to reach spot a1 on the board
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_grab = -0.182
        hover_distance = 0.15
        limbR = 'right'
        print("pre init node")
        rospy.init_node("CHESS")
        print("pre pnp")
        self.pnp = moveClasses.PickAndPlace(limbR, hover_distance)
        print("pe gripper cal")
        self.pnp.gripper_calibrate()
        print("post gripper cal")
        self.board_length = board_length
        self.square_length = self.board_length/8
        self.squares = []
        self.pieces = []
        self.overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
        self.dumpster_pose = Pose(position=Point(x=x_ref - 0.2, y=y_ref - 0.3, z=self.z_grab), orientation=self.overhead_orientation)

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

    def move_piece(self, old_pose, new_pose):
        for piece in self.pieces:
            if piece.pose == new_pose:
                for square in self.squares:
                    if square.pose == piece.pose:
                        x_pose = square.x
                        y_pose = square.y
                taken_piece_pose = Pose(
                    position=Point(x=x_pose, y=y_pose, z=self.z_grab),
                    orientation=self.overhead_orientation)
                self.pnp.pick(taken_piece_pose)
                self.pnp.place(self.dumpster_pose)
                piece.pose = None
                del piece
        for piece in self.pieces:
            if piece.pose == old_pose:
                piece.pose = new_pose
                for square in self.squares:
                    if square.pose == old_pose:
                        old_x = square.x
                        old_y = square.y
                    elif square.pose == new_pose:
                        new_x = square.x
                        new_y = square.y
                pick_pose = Pose(position=Point(x=old_x, y=old_y, z=self.z_grab), orientation=self.overhead_orientation)
                place_pose = Pose(position=Point(x=new_x, y=new_y, z=self.z_grab), orientation=self.overhead_orientation)
                self.pnp.pick(pick_pose)
                self.pnp.place(place_pose)
                break

    def calibrate_board_postition(self):
        for square in self.squares:
            if square.pose == 'a1':
                x = square.x
                y = square.y
                a1_pose = Pose(position=Point(x=x, y=y, z=self.z_grab), orientation=self.overhead_orientation)
            if square.pose == 'h8':
                x = square.x
                y = square.y
                h8_pose = Pose(position=Point(x=x, y=y, z=self.z_grab), orientation=self.overhead_orientation)

        self.pnp.gripper_open()
        # servo above pose
        self.pnp._approach(a1_pose)
        # servo to pose
        self.pnp._servo_to_pose(a1_pose)
        #wait for user to adjust the board position, then move on to the next calibration position
        done = raw_input("Move the board such that the a1 position rook is between the gripper, then type done and press enter")
        self.pnp._retract()


        #now do the calibration on the h8 position of the board
        self.pnp.gripper_open()
        # servo above pose
        self.pnp._approach(h8_pose)
        # servo to pose
        self.pnp._servo_to_pose(h8_pose)
        #wait for user to adjust the board position, then move on to the next calibration position
        done = raw_input("Move the board such that the h8 position rook is between the gripper, then press type done and press enter")
        self.pnp._retract()









