#!/usr/bin/env python

from boardClasses import Board
from moveClasses import PickAndPlace
import socket

#for sending
UDP_IP = "192.168.1.10"
UDP_PORT = 5005
MESSAGE = b"Hello, World!"

#for receiving
UDP_IP2 = "192.168.1.10"
UDP_PORT2 = 5005
sock = socket.socket(socket.AF_INET,  # Internet
socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP2, UDP_PORT2))

#the board length in meters
board_length = 0.46
#the coordinates used for the gripper to reach the a1 spot on the chess board
x_ref = 0.688
y_ref = -0.652
z_ref = -0.170
x_orientation = 0.955
y_orientation = 0.292
z_orientation = 0.0

#initialize the robot interface and other classes
game = Board(x_ref, y_ref, board_length)

while(1):
    #white moves first. The text input will be replaced with camera move detection

    white_move = input("move white\n")
    '''
    white_move = input("make move\n")
    MESSAGE = white_move
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    '''

    old_pose = white_move[0:1]
    new_pose = white_move[2:3]
    game.move_piece(old_pose, new_pose)


    #black_move = stockfish.get_best_move_time(2000)
    '''
    while True:
        data, addr = sock.recvfrom(1024)
    black_move = addr
    '''
    black_move = input("move black\n")
    old_pose = black_move[0:1]
    new_pose = black_move[2:3]
    game.move_piece(old_pose, new_pose)

