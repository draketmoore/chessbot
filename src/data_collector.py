import cv2
import chess
import numpy as np
import string
import random
import os

data_dir = '/home/river/interbotix_ws/src/chessbot/data'


class Data_Collector:

    def __init__(self, isblack=True):
        self.isblack = isblack
        
        board = chess.Board()
        print(board.piece_at(chess.parse_square("e3")))

    def collect_data(self, board, image):
        
        dy = int(len(image)/8)
        dx = int(len(image[0])/8)
        for y in range(8):
            for x in range(8):
                if self.isblack:
                    yindex = 7 - y
                    xindex = x
                else:
                    yindex = y
                    xindex = x
                
                square = chr(xindex+97) + str(yindex + 1)
                piece_name = str(board.piece_at(chess.parse_square(square)))
                
                if piece_name is not None:
                    square_im = cv2.resize(image[y*dy: (y+1)*dy, x*dx:(x+1)*dx], (29, 29))
                    im_name = piece_name + ''.join(random.choice(string.ascii_lowercase) for i in range(20))
                    dir = data_dir + "/" + piece_name
                    if not os.path.isdir(dir):
                        os.mkdir(dir)
                    cv2.imwrite(dir + "/" + im_name + ".jpg", square_im)
                
                

if __name__ == "__main__":
    collector = Data_Collector()