import argparse
import struct
import sys
import copy
import time

import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import numpy as np
import os

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

    def __init__(self, pose, type, color):
        self.type = type
        self.pose = pose
        self.color = color


class Square:

    def __init__(self, pose, x, y):
        self.cal = int(pose[1])*0.001
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
        #rospy.init_node('Camera_Subscriber',anonymous=True) # Initialze ROS node
        # Subscribe to right_hand_camera image topic
        print("pre")
        self.save_picture = False
        rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.image_callback)
        #rospy.spin() # sleep
        print("post")
        cv2.destroyAllWindows() # Destroy CV image window on shut_down
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_grab = -0.182
        hover_distance = 0.1
        limbR = 'right'
        rospy.init_node("CHESS")
        self.pnp = moveClasses.PickAndPlace(limbR, hover_distance)
        self.pnp.gripper_calibrate()
        self.board_length = board_length
        self.square_length = self.board_length/8
        self.squares = []
        self.pieces = []
        self.overhead_orientation = Quaternion(
                             x=0.003,
                             y=0.999,
                             z=-0.002,
                             w=-0.009)
        self.overhead_x = 0.64
        self.overhead_y = -0.348
        self.overhead_z = 0.156
        self.overhead_pose = Pose(position=Point(x=self.overhead_x, y=self.overhead_y, z=-self.overhead_z),
                                     orientation=self.overhead_orientation)
        self.camera_view_orientation = Quaternion(
            x=0.006,
            y=0.995,
            z=0.020,
            w=-0.019
        )
        self.camera_view_x = 0.64
        self.camera_view_y = -0.293
        self.camera_view_z = 0.366
        self.camera_view_pose = Pose(position=Point(x=self.camera_view_x, y=self.camera_view_y, z=self.camera_view_z),
                                     orientation=self.camera_view_orientation)
        self.dumpster_pose = Pose(position=Point(x=x_ref - 0.2, y=y_ref - 0.3, z=self.z_grab), orientation=self.overhead_orientation)
        self.dir = None



        rows = ['1', '2', '3', '4', '5', '6', '7', '8']
        cols = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

        for i in range(8):
            for j in range(8):
                self.squares.append(Square((cols[i] + rows[j]), x_ref - j * self.square_length, y_ref + i * self.square_length ))

        types = ['rook', 'knight', 'bishop', 'queen', 'king', 'bishop', 'knight', 'rook']
        for i in range(8):
            self.pieces.append(Piece((cols[i] + '1'), types[i], 'w'))
            self.pieces.append(Piece((cols[i] + '2'), 'pawn', 'w'))
            self.pieces.append(Piece((cols[i] + '7'), 'pawn', 'w'))
            self.pieces.append(Piece((cols[i] + '8'), types[i], 'w'))

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
                print(x)
                print(y)
                print(self.z_grab)
                h8_pose = Pose(position=Point(x=x, y=y, z=-0.175), orientation=self.overhead_orientation)
        
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
        print("pre approach")
        self.pnp._approach(h8_pose)
        # servo to pose
        print("pre servo to pose")
        self.pnp._servo_to_pose(h8_pose)
        #wait for user to adjust the board position, then move on to the next calibration position
        done = raw_input("Move the board such that the h8 position rook is between the gripper, then press type done and press enter")
        self.pnp._retract()

    def go_to_camera_view(self):
        self.pnp._servo_to_pose(self.camera_view_pose)

    def move_to_overhead(self):
        self.pnp._servo_to_pose(self.overhead_pose)

    def image_callback(self, ros_img):
        if self.save_picture == True:
            bridge = cv_bridge.CvBridge()
            #Convert received image message to OpenCv image
            cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
            #print(str(self.save_picture))
            cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/real_before.png', cv_image)
            self.save_picture = False
            self.detect_board()

    def take_picture(self, dir):
        time.sleep(1)
        self.dir = dir
        self.save_picture = True
    
    def detect_board(self):
        image = cv2.imread('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/real_before.png')
        #print(image.shape) # Print image shape
        #cv2.imshow("original", image)
        height, width = image.shape[:2]
        center = (width/2, height/2)
        rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=-2, scale=1)
        rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))
        #cv2.imshow("rotated", rotated_image)
        # Cropping an image
        cropped_image = rotated_image[82:353, 210:491]
        # Display cropped image
        #cv2.imshow("cropped", cropped_image)
        # Save the cropped image
        cv2.imwrite('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/crop.png', cropped_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.dissect_image()

    def dissect_image(self):
        img =  cv2.imread("/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/crop.png")
        height, width = img.shape[:2]
        center = (width/2, height/2)
        rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=180, scale=1)
        img = cv2.warpAffine(src=img, M=rotate_matrix, dsize=(width, height))
        image_copy = img.copy() 
        imgheight=img.shape[0]
        imgwidth=img.shape[1]

        M = imgheight/8
        N = imgwidth/8
        x1 = 0
        y1 = 0
        y_range = range(0, imgheight, M)
        x_range = range(0, imgwidth, N)
        numbers = [8,7,6,5,4,3,2,1]
        letters = ['a','b','c','d','e','f','g','h']

        for y, num in zip(y_range,numbers):
            for x, let in zip(x_range,letters):
                if (imgheight - y) < M or (imgwidth - x) < N:
                    break
                y1 = y + M
                x1 = x + N

                # check whether the patch width or height exceeds the image width or height
                if x1 >= imgwidth and y1 >= imgheight:
                    x1 = imgwidth - 1
                    y1 = imgheight - 1
                    #Crop into patches of size MxN
                    tiles = image_copy[y:y+M, x:x+N]
                    #Save each patch into file directory
                    cv2.imwrite(self.dir+str(let)+str(num)+'.png', tiles)
                    cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
                elif y1 >= imgheight: # when patch height exceeds the image height
                    y1 = imgheight - 1
                    #Crop into patches of size MxN
                    tiles = image_copy[y:y+M, x:x+N]
                    #Save each patch into file directory
                    cv2.imwrite(self.dir+str(let)+str(num)+'.png', tiles)
                    cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
                elif x1 >= imgwidth: # when patch width exceeds the image width
                    x1 = imgwidth - 1
                    #Crop into patches of size MxN
                    tiles = image_copy[y:y+M, x:x+N]
                    #Save each patch into file directory
                    cv2.imwrite(self.dir+str(let)+str(num)+'.png', tiles)
                    cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)
                else:
                    #Crop into patches of size MxN
                    tiles = image_copy[y:y+M, x:x+N]
                    #Save each patch into file directory
                    cv2.imwrite(self.dir+str(let)+str(num)+'.png', tiles)
                    cv2.rectangle(img, (x, y), (x1, y1), (0, 255, 0), 1)


    def find_player_move(self):
        path1 = '/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/saved_patches/' 
        path2 = '/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/new_saved_patches/' 

        changed_places = []
        list = os.listdir('/home/gle-3271-nix01/baxter_ws/src/baxter_examples/scripts/baxter_chess/Images/new_saved_patches/')
        errors = []
        pieces = []
        for l in range(len(list)):
            before = cv2.cvtColor(cv2.imread(path1+str(list[l])), cv2.COLOR_BGR2GRAY)
            after = cv2.cvtColor(cv2.imread(path2+str(list[l])), cv2.COLOR_BGR2GRAY)
            diff = after.astype("float") - before.astype("float")
            err = np.sum(np.square(diff))
            string = list[l]
            piece = string.replace('.png','')
            errors.append(err)
            pieces.append(piece)
            
        max_index = errors.index(max(errors))
        changed_places.append(pieces.pop(max_index))
        errors.pop(max_index)
        max_index = errors.index(max(errors))
        changed_places.append(pieces.pop(max_index))
        move = 'no move found'
        print(changed_places)
        for piece in self.pieces:
            if piece.color == 'w':
                if piece.pose == changed_places[0]:
                    move = changed_places[0] + changed_places[1]
                elif piece.pose == changed_places[1]:
                    move = changed_places[1] + changed_places[0]
        print(move)
        return move
                

