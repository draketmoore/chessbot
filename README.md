# Chessbot
## Background
Today, anyone, anywhere, at any time can play against a chess bot through digital interfaces such as websites or apps, 
but with the advancing integration of engineering and computer science it's a wonder why we have yet to see chess world champions 
compete against chess robots. Current chess bots preside in digital environments, 
and implementing chess bots in a real world space relies on a human to “make a move” for the computer. 
What if, however, the computer had the autonomy to “make a move” in a real world space, no humans require.  
What if we gave a computer an arm? The basis of my project is work towards just that, 
a transfer of chess bots from the digital interface to a real world environment where humans 
can play chess against a robot.

## Environment Setup
### Hardware Used
1. Intel Realsense Camera for RGBD Data
2. Apriltags at the corners of the chessboard and at the base of the robot
3. Interbotix 6DOF Arm
4. Chessboard

## Perception Pipeline
For the basic model of my chessbot, I obtained images of the chessboard before and after a move was made.
By Cropping the image by the apriltags present in the corners of the chessboard, the precise indices of the piece moved can be obtained,
and the resulting legal chess move can be registered into the chess backend.

This system only works when the game begins from a known board state. In order to gain more flexibility,
I saved the image of each chess square after every move and used it to train an image classifier in order to systematically detect what piece resides on each
chess square for any board state.

## Manipulation
By controlling the robot arm to operate in a crane-like movement, I was able to avoid unwanted contact with any surrounding pieces and perform each move correctly.

Below is a video of the final result.

https://drive.google.com/file/d/1708KWPqq1-FwvPJjhGd5C4F5uY9Sb2bj/view?usp=sharing
