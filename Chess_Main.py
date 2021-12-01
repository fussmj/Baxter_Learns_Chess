#!/usr/bin/env python
from stockfish import Stockfish
from boardClasses import Board, Square
from moveClasses import pick, place


#the board length in meters
board_length = 0.46
#the x and y coordinates used for the gripper to reach the a1 spot on the chess board
x_ref = 0.688
y_ref = -0.652
z_ref = -0.170
x_orientation = 0.955
y_orientation = 0.292
z_orientation = 0.0



#initialize the stockfish game
stockfish = Stockfish("C:/Users/Matthew/Desktop/Baxter_Learns_Chess/stockfish_14.1_win_x64_avx2.exe")
stockfish.set_position([])

#initialize the robot interface and other classes
game = Board(x_ref, y_ref, board_length)

print(stockfish.get_board_visual())

while(1):
    #white moves first. The text input will be replaced with camera move detection
    valid = False
    while not valid:
        white_move = input("enter white move: \n")
        valid = stockfish.is_move_correct(white_move)
        if not valid:
            print("Illegal move. please do something else")
        else:
            stockfish.make_moves_from_current_position([white_move])
            old_pose = white_move[0:1]
            new_pose = white_move[2:3]
            game.move_piece(old_pose, new_pose)

    print(stockfish.get_board_visual())


    black_move = stockfish.get_best_move_time(2000)
    stockfish.make_moves_from_current_position([black_move])
    old_pose = white_move[0:1]
    new_pose = white_move[2:3]
    game.move_piece(old_pose, new_pose)

    print(black_move)
    print(stockfish.get_board_visual())