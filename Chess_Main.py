from stockfish import Stockfish

import boardClasses
import modelClasses
import moveClasses

#initialize the stockfish game
stockfish = Stockfish("C:/Users/Matthew/Desktop/Baxter_Learns_Chess/stockfish_14.1_win_x64_avx2.exe")
stockfish.set_position([])

#initialize the robot interface classes


print(stockfish.get_board_visual())

while(1):

    #white moves first. The text input will be replaced with camera move detection
    white_move = input("enter white move: \n")
    stockfish.make_moves_from_current_position([white_move])
    print(stockfish.get_board_visual())


    black_move = stockfish.get_best_move_time(2000)
    stockfish.make_moves_from_current_position([black_move])
    print(black_move)
    print(stockfish.get_board_visual())