# Python file to define constants

import numpy as np

# DIRECTIONS
NORTH = "Up"
EAST = "Right"
SOUTH = "Down"
WEST = "Left"

# COMMANDS


FORWARDFAST = "F" # Move forward quickly by 10cm


#SENSOR = "L" # Get sensor readings from Arduino

#PICTURE = "I"

REACHEDSTART= "G"

ENDEXPLORATIONALIGNSOUTH = "V"
ENDEXPLORATIONWEST = "M"

BACKWARDSFAST = "b" # Move backwards quickly by 10cm
LEFT = "l" # Rotate counter-clockwise by 90 degrees
RIGHT = "r" # Rotate clockwise by 90 degrees
ROTATE180 = "rr"
BACKWARDS = "b" # Move backwards by 10cm
#Forward Commands x10cm
FORWARD = "f"
ALIGNRIGHT = "" # Tell robot to align itself using obstacles on the right
# ALIGNRIGHT = "rol"
ALIGNFRONT2 = "x"#align front with 2 sensors
ALIGNFRONT = "o" # Tell robot to align itself using obstacles on the front
AlIGNFRONTSTAIR = "y"#calibrate using middle front and right front sensors to calibrate for staircase, not using currently

# Arena is 200cm by 150cm
# We take each 10cm by 10cm square as a spot
# Robot is taken as always occupying 3 by 3 space
# MAP CONSTANTS
MAX_ROWS = 20 # 200 / 10 = 20
MAX_COLS = 15 # 150 / 10 = 15
START = np.asarray([18, 1]) # Refers to where the centre of the robot should be at start position
GOAL = np.asarray([1, 13]) # Refers to where the centre of the robot should be once it reaches the goal
# Not the actual bottom left corner of the arena but the most bottom left
# where the centre of the robot can reach. (We assume that there is a virtual wall)
BOTTOM_LEFT_CORNER = START
BOTTOM_RIGHT_CORNER = np.asarray([18,13])
TOP_RIGHT_CORNER = GOAL
TOP_LEFT_CORNER = np.asarray([1,1])

# Take note that North does not refer to actual north
# North refers to the direction the robot will be facing when it is placed at the bottom left and then facing the top left
# In the actual run, the robot can be facing any physical direction as long as it starts at the bottom left
# Therefore, we will be moving "south" as row index increases and east as column index increases.
