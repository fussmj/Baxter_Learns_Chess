from stockfish import Stockfish
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


stockfish = Stockfish("C:/Users/Matthew/Desktop/Baxter_Learns_Chess/stockfish_14.1_win_x64_avx2.exe")
stockfish.set_position([])
print(stockfish.get_board_visual())

while(1):

    while True:
        data, addr = sock.recvfrom(1024)
    print("received message: %s" % data)

    white_move = data
    valid = stockfish.is_move_correct(white_move)
    if not valid:
        print("illegal move, try again")
        continue
    stockfish.make_moves_from_current_position([white_move])
    print(stockfish.get_board_visual())


    black_move = stockfish.get_best_move_time(2000)
    MESSAGE = black_move
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    stockfish.make_moves_from_current_position([black_move])
    print(stockfish.get_board_visual())