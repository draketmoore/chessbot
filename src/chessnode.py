#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import math
import time
import cv2
from cv_bridge import CvBridge
from chessboard_detector import Chess_Detector
from interbotix_xs_modules.arm import InterbotixManipulatorXS

class ChessBot:

    def __init__(self, blackleft = True):
        # rospy.init_node("chessnode")

        # Initialize state variables
        self.bridge = CvBridge()
        self.tags = {}
        self.corner_tag_ids = [0, 1]
        self.corners = []
        self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper")
        self.blackleft = blackleft

        self.x_offset = 0.14
        self.y_offset = -0.03#-0.06

        # Initialize the listeners
        self.tf_listener = tf.TransformListener()
        self.corner_listener = rospy.Subscriber("corner_detections", Int32MultiArray, self.corner_cb)
        
        time.sleep(0.5)
        self.process_tags()

        self.board_image = self.get_cropped_image()
        self.detector = Chess_Detector(self.board_image, self.blackleft)
 

    def get_cropped_image(self):
        """
        Observes and image, and crops the image based on the two apriltags in the corners of the chessboard.
        """
        image = rospy.wait_for_message("/camera/color/image_raw", Image)
        image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        # if self.blackleft:
        #     corner_tag_names = ['/tag_1', '/tag_0']
        # else:
        #     corner_tag_names = ['/tag_0', '/tag_1']
        # _, rot1 = self.tf_listener.lookupTransform('/camera_link', corner_tag_names[0], rospy.Time(0))
        # _, rot2 = self.tf_listener.lookupTransform('/camera_link', corner_tag_names[1], rospy.Time(0))

        # print(euler_from_quaternion(rot1))



        sx = min(self.corners[:, 0])
        ex = max(self.corners[:, 0])
        sy = min(self.corners[:, 1])
        ey = max(self.corners[:, 1])
        cropped = image[sy:ey, sx:ex]
        return cropped
        
    def corner_cb(self, msg):
        """
        Constantly updating the corner positions of the chessboard
        """
        data = np.array(msg.data)
        # Data size per node is 9
        data = np.reshape(data, (-1, 9))

        for tag in data:
            self.tags[tag[0]] = np.reshape(tag[1:], (-1, 2))


    def process_tags(self):
        """
        For the apriltags, figures out which corners are closest to each other and uses them as the boundary.
        """
        corner0 = self.tags[self.corner_tag_ids[0]]
        corner1 = self.tags[self.corner_tag_ids[1]]

        # The corners that are closest to each other are the corners of the chessboard
        min_dist = 10000
        for c0 in corner0:
            for c1 in corner1:
                if math.dist(c0, c1) < min_dist:
                    min_dist = math.dist(c0, c1)
                    self.corners = np.array([c0, c1])

    def get_move(self):
        """
        Waits for the user to input a chess move, passes the cropped into the chess detector.
        Once the chess detector has figured out what move to make, the squares are converted into the
        robot's frame and the move is made.
        """

        input("Press enter for a move")
        im = self.get_cropped_image()
        # Move is formatted [y, x], where y is the letter and x is the value
        # April tag has the y axis to be the length of the chessboard
        move = self.detector.get_move(im)
        if move is None:
            return

        from_pose = self.square_to_cartesian(move[0])
        to_pose = self.square_to_cartesian(move[1])
        capture_pose = None
        if len(move) == 3:
            capture_pose = self.square_to_cartesian(move[2])

        self.move_arm(from_pose, to_pose, capture_pose=capture_pose)
        time.sleep(1)
        new_im = self.get_cropped_image()
        self.detector.update_start_image(new_im)

    def square_to_cartesian(self, square):
        """
        Input: A chess square in chess notation
        Function: Computes the cartesian position relative to the robot apriltag.
        """

        # Figures out which apriltag to base the chess coordinates off of
        if self.blackleft:
            corner_tag_names = ['/tag_1', '/tag_0']
        else:
            corner_tag_names = ['/tag_0', '/tag_1']
        # Converts all relative coordinates to be realtive to the robot's apriltag
        corner_tag, _ = self.tf_listener.lookupTransform('/tag_2', corner_tag_names[0], rospy.Time(0))
        opposite_tag, _ = self.tf_listener.lookupTransform('/tag_2', corner_tag_names[1], rospy.Time(0))

        corner_tag = np.array(corner_tag)
        opposite_tag = np.array(opposite_tag)


        tag_diff = (opposite_tag - corner_tag) / 8

        # Current poses are relative to the robot's apriltag
        cart_pose = [corner_tag[0] + tag_diff[0] * square[0],
                          corner_tag[1] + tag_diff[1] * square[1],
                          corner_tag[2]]


        # Convert them to be relative to the robot's frame
        pose = [cart_pose[1] + self.x_offset, -cart_pose[0] + self.y_offset, cart_pose[2]]

        return pose

    
    def move_arm(self, from_pose, to_pose, capture_pose = None):
        """
        Moves the arm to perform the given moves.
        """
        self.bot.arm.go_to_home_pose()
        if capture_pose is not None:
            self.bot.arm.set_ee_pose_components(x=capture_pose[0], y=capture_pose[1], z=0.15, roll=0.0, pitch=1.5)
            self.bot.gripper.close(0)
            self.bot.arm.set_ee_pose_components(x=capture_pose[0], y=capture_pose[1], z=0.06, roll=0, pitch=1.5)
            self.bot.arm.set_ee_pose_components(x=capture_pose[0], y=capture_pose[1], z=0.15, roll=0.0, pitch=1.5)
            # Remove the piece
            self.bot.arm.set_ee_pose_components(x=0.3, y=0.3, z=0.15, roll=0.0, pitch=1.5)
            self.bot.gripper.open(0.3)
            



        self.bot.arm.set_ee_pose_components(x=from_pose[0], y=from_pose[1], z=0.15, roll=0.0, pitch=1.5)
        self.bot.gripper.close(0)
        self.bot.arm.set_ee_pose_components(x=from_pose[0], y=from_pose[1], z=0.06, roll=0, pitch=1.5)
        self.bot.arm.set_ee_pose_components(x=from_pose[0], y=from_pose[1], z=0.15, roll=0.0, pitch=1.5)


        # self.bot.arm.go_to_home_pose()
        self.bot.arm.set_ee_pose_components(x=to_pose[0], y=to_pose[1], z=0.15, roll=0.0, pitch=1.5)
        self.bot.arm.set_ee_pose_components(x=to_pose[0], y=to_pose[1], z=0.06, roll=0, pitch=1.5)
        self.bot.gripper.open(0.3)
        self.bot.arm.set_ee_pose_components(x=to_pose[0], y=to_pose[1], z=0.15, roll=0.0, pitch=1.5)
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()

         

        
if __name__ == "__main__":

    bot = ChessBot()
    
    while not bot.detector.board.is_game_over():
        bot.get_move()
    
    
    rospy.spin()