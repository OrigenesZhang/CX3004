import numpy as np
import os

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, RIGHT, LEFT, MAX_ROWS, MAX_COLS

class Robot:

    """Robot class keeps track of the current location and direction of the robot,
       gets values from the distance sensors and sends commands to move the robot

    Attributes:
        center (list): Center location of the robot
        direction (int): Current direction of the robot (see Constants)
        exploredMap (Numpy array): The current explored map
        head (list): Location of the head of the robot
        map (Numpy array): Real map of the arena for simulation mode
        movement (string): The latest movement of the robot (see Constants)
        realMap (string): File name for map
    """

    def __init__(self, exploredMap, direction, start, realMap):
        """Constructor to initialise an instance of the Exploration class

        Args:
            exploredMap (Numpy array): To initial state of the exploration map
            direction (int): The starting direction for the robot (see Constants)
            start (list): The starting center location of the robot
            realMap (string): File name of the real map
        """
        self.exploredMap = exploredMap # e.g. np.zeros([20, 15])
        self.direction = direction # e.g. EAST
        self.center = np.asarray(start) # e.g. np.asarray([18, 1])
        self.head = None
        self.setHead()
        # If name of actual map is provided, load the map
        if realMap:
            self.realMap = realMap
            self.map = self.loadMap()
        self.movement = []
        # Set the 3X3 area around start coordinate to a
        # value of one on explored map which means that it is explored
        self.markArea(start, 1)

    def markArea(self, center, value):
        """To mark a 3x3 neighbourhood around the center location with a particular
           value

        Args:
            center (list): Location to mark neighbourhood around
            value (int): The value to be filled
        """
        self.exploredMap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = value

    # Sets position of head of robot based on direction
    def setHead(self):
        # If robot facing North (in the direction where row index decreases),
        # coordinate of head should be coordinate of centre minus one from the row index
        if (self.direction == NORTH):
            self.head = self.center + [-1, 0]
        # If robot is facing east (in the direction where column index increases),
        # coordinate of head should be the coordinate of centre plus one to the column index
        elif (self.direction == EAST):
            self.head = self.center + [0, 1]
        # If the robot is facing south (in the direction where row index increases),
        # coordinate of head should be coordinate of centre plus one to the row index
        elif (self.direction == SOUTH):
            self.head = self.center + [1, 0]
        # If robot is facing west (in the direction where column index decreases),
        # coordinate of head should be coordinate of centre minus one from the column index
        else:
            self.head = self.center + [0, -1]

    def loadMap(self):
        """To load the real map file and store it as a Numpy array

        Returns:
            Numpy array: Real map
        """
        # Load the file with filename given by self.realMap from the Maps folder
        with open(os.path.join('Maps', self.realMap)) as f:
            return np.genfromtxt(f, dtype=int, delimiter=1)
        # e.g. Converts
        # 111111121111111
        # 111111111111111
        # 111111111111111
        # 222211111111111
        # 111211121111111
        # 111111121111111
        # 111111121111111
        # 111111121111111
        # 111111121122111
        # 112111121111222
        # 112111111111111
        # 112111111111111
        # 111111111111111
        # 111111112222222
        # 111111111111111
        # 111211111111111
        # 111121111111111
        # 111112111111111
        # 111111211111111
        # 111111111111111
        # to
        # [[1 1 1 1 1 1 1 2 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [2 2 2 2 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 2 1 1 1 2 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 2 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 2 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 2 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 2 1 1 2 2 1 1 1]
        #  [1 1 2 1 1 1 1 2 1 1 1 1 2 2 2]
        #  [1 1 2 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 2 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 1 2 2 2 2 2 2 2]
        #  [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 2 1 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 2 1 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 2 1 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 2 1 1 1 1 1 1 1 1]
        #  [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]]

    def getValue(self, inds):
        """To get sensor values (from the real map for simulation) at the given indices

        Args:
            inds (list of list): Indices to get sensor values at

        Returns:
            list: values of the sensors and puts none after the first obstacle is encountered
        """
        vals = []
        for (r, c) in inds:
            if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                if self.map[r][c] == 2: # When there is an obstacle at that coordinate (according to actual map)
                    if self.exploredMap[r][c] == 0: # If that coordinate is unexplored
                        self.exploredMap[r][c] = 2 # Set that coordinate as an obstacle
                    vals.append(2)
                    break # If you encounter an obstacle, you do not need to care about the values of the remaining coordinates
                    # Hence, you break out of the loop
                else: # If there is no obstacle
                    if self.exploredMap[r][c] == 0: # If unexplored
                        self.exploredMap[r][c] = 1 # Set the value at that coordinate to be free
                    vals.append(1)
                    # Don't break out of the for loop if there is no obstacle
        if len(vals) < len(inds): # If break out of for loop prematurely or boundary of arena is reached
            vals = vals + [None]*(len(inds)-len(vals)) # Append values of None after the obstacle
        return vals

    def getLongValue(self, inds):
        counter = 0
        for (r, c) in inds:
            counter += 1
            if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                if self.map[r][c] == 2:
                    break
            else:
                break
        if(counter == 1 or counter == 5):
            return [None] * len(inds)
        else:
            return self.getValue(inds)
            


    def getSensors(self):
        """Generated indices to get values from sensors and gets the values using getValue() function.
           For this simulator it is assumed that the sensors can get values up a distance of 4 cells

        Returns:
            Numpy array of Numpy arrays: Sensor values from all sensors
        """
        distanceShort = 3
        distanceLong = 3
        r, c = self.center

        # Front Left
        if self.direction == NORTH:
            # e.g. If r = 5, c = 12 and distanceShort = 3,
            # zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1]
            # gives [(3, 11), (2, 11), (1, 11)]
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1])
        elif self.direction == EAST:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)))
        elif self.direction == WEST:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1])
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort))

        # Front Center
        if self.direction == NORTH:
            # e.g. If r = 5, c = 12 and distanceShort = 3,
            # zip(range(r-distanceShort-1, r-1), [c]*distanceShort)[::-1]
            # gives [(3, 12), (2, 12), (1, 12)]
            self.getValue(zip(range(r-distanceShort-1, r-1), [c]*distanceShort)[::-1])
        elif self.direction == EAST:
            self.getValue(zip([r]*distanceShort, range(c+2, c+distanceShort+2)))
        elif self.direction == WEST:
            self.getValue(zip([r]*distanceShort, range(c-distanceShort-1, c-1))[::-1])
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c]*distanceShort))

        # Front Right
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1])
        elif self.direction == EAST:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)))
        elif self.direction == WEST:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1))[::-1])
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort))

        # Right Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)))
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort))
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1])
        else:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1])

        # Right Bottom
        if self.direction == NORTH:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)))
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort))
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1])
        else:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1))[::-1])

        # Left Middle
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceLong, range(c-distanceLong-1, c-1))[::-1])
        elif self.direction == EAST:
            self.getValue(zip(range(r-distanceLong-1, r-1), [c+1]*distanceLong)[::-1])
        elif self.direction == WEST:
            self.getValue(zip(range(r+2, r+distanceLong+2), [c-1]*distanceLong))
        else:
            self.getValue(zip([r+1]*distanceLong, range(c+2, c+distanceLong+2)))

    def moveBot(self, movement):
        """Simulates the bot movement based on current location, direction and received action
        Sets self.direction, self.center and self.head

        Args:
            movement (string): Next movement received (see Constants)
        """
        # movement will be a direction defined in Constants.py (LEFT, RIGHT, Forward)
        self.movement.append(movement)
        if self.direction == NORTH: # If current direction is North
            if movement == RIGHT: # And you start moving to the right
                self.direction = EAST # Your new direction will be EAST
            # Moving left and right means turning on the spot
            elif movement == LEFT:
                self.direction = WEST
            # If movement is FORWARD, a new center to be one step forward in the north direction
            else:
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1) # Set 3 by 3 area around robot as explored on self.exploredMap
            self.setHead() # Update coordinates of the robot's head
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
            self.setHead()
        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
            self.setHead()
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
            self.setHead()

    def descriptor_1(self):
        # Create a 20 by 15 array filled with zero first
        descriptor = np.zeros([20, 15]).astype(int)
        # If the coordinate is 2 (has obstacle) or 1 (explored and free),
        # then set value at coordinate in descriptor to be 1
        descriptor[self.exploredMap[:, :] != 0] = 1
        # Need to pad map descriptor at the start according to the requirements
        bits = '11'
        for i in range(len(descriptor)):
            for j in range(len(descriptor[i])):
                bits += str(descriptor[i][j])
        bits += '11' # Need to pad again at the end
        # 20 * 15 + 4 = 304 which is a multiple of 4 so it can be converted into a hex string
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)] # Generate hex string
        return ''.join(hex_str)
        # e.g.
        # [[0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        #           [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        #           [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1]]
        # turns into FF3FFE7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFCFFF9FFFFF3FFE7FFFFFFFFFFFFFFFF3FFE7FFF
        # where final value of bits is 1111111100111111111111100111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111110011111111111110011111111111111111111100111111111111100111111111111111111111111111111111111111111111111111111111111111111100111111111111100111111111111111

    def descriptor_2(self):
        descriptor = np.ones([20, 15]).astype(int)
        descriptor[self.exploredMap[::-1, :] != 2] = 0
        bits = ''
        for i in range(len(descriptor)):
            for j in range(len(descriptor[i])):
                bits += str(descriptor[i][j])
        # bits += '11'
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)

    # def descriptor_3(self):
    #     bits = ''
    #     for row in self.exploredMap[::-1, :]:
    #         for bit in row:
    #             if bit == 2:
    #                 bits += '1'
    #             elif bit == 1:
    #                 bits += '0'
    #     bits += '0'*(4 - len(bits) % 4)
    #     hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
    #     hex_str = hex_str[:-1]
    #     return ''.join(hex_str)
