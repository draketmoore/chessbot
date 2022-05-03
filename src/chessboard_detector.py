#!/usr/bin/env python3
import cv2
import chess
import chess.engine as engine
from gpg import Data
from apriltag_ros.srv import AnalyzeSingleImage
from sensor_msgs.msg import Image, CameraInfo
import rospy
from cv_bridge import CvBridge
import numpy as np
import random
from stockfish import Stockfish
from data_collector import Data_Collector
"""
roslaunch realsense2_camera rs_camera.launch 
roslaunch apriltag_ros continuous_detection.launch 
roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx300s
"""

class Chess_Detector:

    def __init__(self, start_im, isblack, collect_data = True):
        self.start_im = start_im
        self.isblack = isblack
        self.board = chess.Board()
        self.collect_data = collect_data
        if self.collect_data:
            self.collector = Data_Collector(self.isblack)
        cv2.imwrite("/home/river/Desktop/start.png", self.start_im)
        # self.engine = engine.SimpleEngine.popen_uci(r"/home/river/Downloads/stockfish_15_linux_x64_bmi2/stockfish_15_x64_bmi2")
        # self.stockfish = Stockfish(path=r"/home/river/Downloads/stockfish_15_linux_x64_bmi2/stockfish_15_x64_bmi2")
        # print("HELLO\n\n\n\n")

    def update_start_image(self, im):
        self.start_im = im

    def get_diffs(self, im):
        """
        Input: Image of the chessboard after a move has been made
        Function: Takes the difference between the current image and previous image.
        Returns an array of x positions, y positions, and the corresponding difference for that square from the original image
        """
        diff = cv2.absdiff(self.start_im, im)
        diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        # print(np.amax(diff))
        _, bdiff = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/home/river/Desktop/diff.png", bdiff)
        cv2.imshow("test", bdiff)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        xs = []
        ys = []
        diffs = []

        dy = int(len(bdiff)/8)
        dx = int(len(bdiff[0])/8)
        print(dx, dy)
        for y in range(8):
            for x in range(8):
                square = bdiff[y*dy: (y+1)*dy, x*dx:(x+1)*dx]
                totaldiff = np.sum(square)
                xs.append(x)
                ys.append(y)
                diffs.append(totaldiff)
        return diffs, xs, ys

    def get_move(self, im):
        """
        Input: Image of the chessboard after a move has been made
        Function: Takes the difference between the current image and previous image and uses it to compute what move was made
        After updating the board, a new engine move is calculated and the corresponding indicies of the squares are returned
        """
        cv2.imwrite("/home/river/Desktop/end.png", im)
        diffs, xs, ys = self.get_diffs(im)
        maxdiffi = diffs.index(max(diffs))
        diffs[maxdiffi] = 0
        y = ys[maxdiffi]
        x = xs[maxdiffi]
        if self.isblack:
            y = 7 - y
            x = x
            
        square1 = chr(x+97) + str(y + 1)
        int_square1 = [x, y]
        
        maxdiffi = diffs.index(max(diffs))
        y = ys[maxdiffi]
        x = xs[maxdiffi]
        if self.isblack:
            y = 7 - y
            x = x
        square2 = chr(x+97) + str(y + 1)
        int_square2 = [x, y]
        
        print(square1, square2)
        legal_moves = list(self.board.legal_moves)
        if chess.Move.from_uci(square1 + square2) in legal_moves:
            move = square1 + square2
            int_move = [int_square1, int_square2]
        elif chess.Move.from_uci(square2 + square1) in legal_moves:
            move = square2 + square1
            int_move = [int_square2, int_square1]
        else:
            input("Did not detect a legal move. Please reset the board to the previous state and press enter when finished")
            return 

        print(move)
        self.board.push_uci(move)
        print(self.board)

        if self.collect_data:
            self.collector.collect_data(self.board, im)


        int_move = self.get_engine_move()
        print(self.board)
        self.start_im = im

        return int_move

    def get_engine_move(self):
        """
        Given a board state, a new move is calculated and the corresponding x and y coordinates for each square are returned
        """
        legal_moves = list(self.board.legal_moves)
        # self.stockfish.set_fen_position(self.board.fen())
        # stockfish_move = self.stockfish.get_best_move_time(1000)
        # print(stockfish_move)
        rand_move = random.sample(legal_moves, 1)[0]
        print("\nRobot Move: ")
        print(rand_move)

        int_moves = []
        

        int_moves.append(self.square_to_ints(rand_move.from_square))
        int_moves.append(self.square_to_ints(rand_move.to_square))

        if self.board.is_capture(rand_move):
            int_moves.append(self.square_to_ints(rand_move.to_square))

        self.board.push_uci(rand_move.uci())
        print(int_moves)
        return int_moves

    def square_to_ints(self, square):
        """
        Input: A chess square in chess notation
        Function: Computes the integer coordinates of the square.
        """
        square = chess.square_name(square)
        letter = square[0]
        num = int(square[1]) - 1
        letter_num = ord(letter) - 97

        if self.isblack:
            letter_num = 7 - letter_num
            num = 7 - num
        return [letter_num, num]
        
 


# if __name__ == "__main__":
    # d = Chess_Detector(None, False)
    # print(d.get_engine_move())
    # e = chess.engine.SimpleEngine.popen_uci(r"/home/river/Downloads/stockfish_15_linux_x64_bmi2/stockfish_15_x64_bmi2")
    # e = Stockfish(path="/home/river/Downloads/stockfish/stockfish-15-64")