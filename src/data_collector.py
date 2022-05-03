import cv2
import chess
import numpy as np

data_dir = '/home/river/interbotix_ws/src/chessbot/data'


class Data_Collector:

    def __init__(self, isblack):
        self.isblack = isblack
        pass

    def collect_data(self, board, image):
        dy = int(len(image)/8)
        dx = int(len(image[0])/8)
        for y in range(8):
            for x in range(8):
                square = image[y*dy: (y+1)*dy, x*dx:(x+1)*dx]
                totaldiff = np.sum(square)
                xs.append(x)
                ys.append(y)
                image.append(totaldiff)