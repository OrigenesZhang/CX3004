import numpy as np

from Constants import START, MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, RIGHT, LEFT,AlIGNFRONTSTAIR, BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER, TOP_RIGHT_CORNER, TOP_LEFT_CORNER, ALIGNRIGHT, ALIGNFRONT, BACKWARDS, FORWARD, FORWARDFAST, BACKWARDS, BACKWARDSFAST, ALIGNFRONT2


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
    """

    def __init__(self, exploredMap, direction, start):
        """Constructor to initialise an instance of the Exploration class

        Args:
            exploredMap (Numpy array): To initial state of the exploration map
            hierarchyMap (Numpy array): confidence level of the mapping
            direction (int): The starting direction for the robot (see Constants)
            start (list): The starting center location of the robot
        """
        self.exploredMap = exploredMap
        # 0 -- unexplored
        # 1 -- marked by left sensor
        # 2 -- marked by right sensors
        # 3 -- marked by front sensors
        self.hierarchyMap = np.zeros((20, 15))
        self.direction = direction
        self.center = np.asarray(start)
        self.marked = np.zeros((20, 15))
        self.setHead()
        self.movement = []
        self.markArea(start, 1)
        self.stepCounter = 0
        self.phase = 1

    def markArea(self, center, value):
        """To mark a 3x3 neighbourhood around the center location with a particular
           value

        Args:
            center (list): Location to mark neighbourhood around
            value (int): The value to be filled
        """
        self.exploredMap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = value

    def setHead(self):
        if (self.direction == NORTH):
            self.head = self.center + [-1, 0]
        elif (self.direction == EAST):
            self.head = self.center + [0, 1]
        elif (self.direction == SOUTH):
            self.head = self.center + [1, 0]
        else:
            self.head = self.center + [0, -1]

    def getValue(self, inds, value, distance, sr, right=False):
        vals = []
        startInds = [(19, 0), (18, 0), (17, 0), (19, 1), (18, 1), (17, 1), (19, 2), (18, 2), (17, 2)]
        goalInds = [(0, 12), (1, 12), (2, 12), (0, 13), (1, 13), (2, 13), (0, 14), (1, 14), (2, 14)]
        if value == 0:
            vals = [2]
            for idx, coord in enumerate(inds):
                r, c = coord
                if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS) and (coord not in startInds) and (coord not in goalInds):
                    if self.phase == 1:
                        if self.exploredMap[r][c] == 1 and vals[idx] == 2 and sr:
                            if not right:   # front sensor
                                self.exploredMap[r][c] = vals[idx]
                                self.marked[r][c] = 1   # Changed to = 1 not == 1
                                self.hierarchyMap[r][c] = 3
                            else:   # right sensor
                                if self.hierarchyMap[r][c] > 2:
                                    break
                                self.exploredMap[r][c] = vals[idx]
                                self.marked[r][c] = 1
                                self.hierarchyMap[r][c] = 2
                        # If any one of the condition above holds true and r, c is an obstacle, break out of the loop
                        elif self.exploredMap[r][c] == 2:
                            if sr:
                                if right:
                                    self.hierarchyMap[r][c] = max(2, self.hierarchyMap[r][c])     # right
                                else:
                                    self.hierarchyMap[r][c] = 3     # front
                            else:
                                self.hierarchyMap[r][c] = max(1, self.hierarchyMap[r][c])         # left
                            break
                        elif self.exploredMap[r][c] == 0: # If unexplored
                            self.exploredMap[r][c] = vals[idx]
                            if sr:
                                if right:
                                    self.hierarchyMap[r][c] = max(2, self.hierarchyMap[r][c])  # right
                                else:
                                    self.hierarchyMap[r][c] = 3  # front
                            else:
                                self.hierarchyMap[r][c] = max(1, self.hierarchyMap[r][c])  # left
                        # self.marker[r][c] is probably to indicate how many times the coordinate
                        # has been marked
                        self.marked[r][c] += 1
                    else:
                        h_val = 1
                        if not sr:
                            h_val = 2
                            if not right:
                                h_val = 3
                        self.hierarchyMap[r][c] = max(self.hierarchyMap[r][c], h_val)
                        if self.exploredMap[r][c] == 2:
                            break
                        elif self.exploredMap[r][c] == 0:
                            self.exploredMap[r][c] = vals[idx]
                break
        else:
            # No need to adjust sensor values for now as the Arduino team will be doing it
            # Sensor may not be exactly at the edge of the robot
            # Hence, we will need to adjust the sensor reading
            # adjustment_value = 6
            # if value != -1:
                # Adjust the sensor reading by subtracting the number of cm the sensor is away from the edge of the robot
                # Round to nearest 10 cm as it should not be the case where the obstacle does not lie exactly in the
                # 10cm by 10cm space
                # value = round(value - adjustment_value, -1)
            # Distance is the number of spaces we assume that the sensor is accurate
            if (value >= distance):
                # If sensor value is greater than the range which we assume the sensor is accurate
                # just assume that there are no obstacles within that accurate range
                vals = [1]*distance
            else:
                # If the sensor detects an obstacle within the accurate range
                value = int(value)
                # Set coordinates to be marked to be only those coordinates up to the obstacle
                inds = inds[:value+1]
                # Set spaces with no obstacle to be explored (value of 1) and space with obstacle with value of 2
                vals = [1]*value + [2]
            for idx, coord in enumerate(inds): # For each supposedly free space
                r, c = coord
                if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS) and (coord not in startInds) and (coord not in goalInds):
                    # for override
                    if self.phase == 1:
                        # right==true for right top and right bottom
                        # sr ==False for right bottom and left changed to only left
                        #if (self.exploredMap[r][c] == 2 and vals[idx] == 1 and sr and (not right)):
                        if self.exploredMap[r][c] == 2 and vals[idx] == 1 and sr:
                            if not right:
                                self.exploredMap[r][c] = vals[idx]
                                self.marked[r][c] = 1  # Changed to = 1 not == 1
                                self.hierarchyMap[r][c] = 3
                            else:
                                if self.hierarchyMap[r][c] > 2:
                                    continue
                                self.exploredMap[r][c] = vals[idx]
                                self.marked[r][c] = 1
                                self.hierarchyMap[r][c] = 2
                        # If any one of the condition above holds true and r, c is an obstacle, break out of the loop
                        elif self.exploredMap[r][c] == 2:
                            break
                        elif self.exploredMap[r][c] == 0: # If unexplored
                            self.exploredMap[r][c] = vals[idx]
                            if sr:
                                if right:
                                    self.hierarchyMap[r][c] = max(2, self.hierarchyMap[r][c])  # right
                                else:
                                    self.hierarchyMap[r][c] = 3  # front
                            else:
                                self.hierarchyMap[r][c] = max(1, self.hierarchyMap[r][c])  # left

                        # self.marker[r][c] is probably to indicate how many times the coordinate
                        # has been marked
                        self.marked[r][c] += 1
                    # self.phase will be marked as 2 in comm_rpi.py when robot has went one round around arena but
                    # there are still unmapped spaces
                    else:
                        if self.exploredMap[r][c] == 2:
                            break
                        elif (self.exploredMap[r][c] == 0):
                            self.exploredMap[r][c] = vals[idx]
                            if sr:
                                if right:
                                    self.hierarchyMap[r][c] = max(2, self.hierarchyMap[r][c])  # right
                                else:
                                    self.hierarchyMap[r][c] = 3  # front
                            else:
                                self.hierarchyMap[r][c] = max(1, self.hierarchyMap[r][c])  # left
                    # without override
                    # if self.exploredMap[r][c] == 0:
                    #     self.exploredMap[r][c] = vals[idx]
                    # # elif (sr and self.marked[r][c] == 0):
                    # #     self.exploredMap[r][c] = vals[idx]
                    # #     self.marked[r][c] = 1
                    # elif self.exploredMap[r][c] == 2:
                    #     break


    def getSensors(self, sensor_vals):
        """Generated indices to get values from sensors and gets the values using getValue() function.

        sensor_vals is an array of values returned from the sensors which indicates the number of cm before
        an obstacle is detected

        sensor_val[0] is the value from the front left sensor
        sensor_val[1] is the value from the front center sensor
        sensor_val[2] is the value from the front right sensor
        sensor_val[3] is the value from the right top sensor
        sensor_val[4] is the value from the right bottom sensor
        sensor_val[5] is the value from the left middle sensor

        Returns:
            Numpy array of Numpy arrays: Sensor values from all sensors
        """
        # Some sensors are more accurate than others
        # We assume that the less accurate sensors are only accurate up to 30 cm
        # We assume that the more accurate sensors are accurate up to 50 cm
        # We assume that the front left, front center and right top sensors are less accurate
        # We assume that the right bottom and left middle sensors are more accurate
        distanceShort = 2
        distanceLong = 4
        r, c = self.center
        # right==true for right top and right bottom
        # sr ==False for right bottom and left changed to only left
        # Front Left
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1],
                          sensor_vals[0], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[0], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[0], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort),
                          sensor_vals[0], distanceShort, True)

        # Front Center
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c]*distanceShort)[::-1],
                          sensor_vals[1], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[1], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[1], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c]*distanceShort),
                          sensor_vals[1], distanceShort, True)

        # Front Right
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1],
                          sensor_vals[2], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[2], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[2], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort),
                          sensor_vals[2], distanceShort, True)

        # Right Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[3], distanceShort, True, True)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort),
                          sensor_vals[3], distanceShort, True, True)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1],
                          sensor_vals[3], distanceShort, True, True)
        else:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[3], distanceShort, True, True)

        # Right Bottom
        if self.direction == NORTH:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[4], distanceShort, True, True)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort),
                          sensor_vals[4], distanceShort, True, True)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1],
                          sensor_vals[4], distanceShort, True, True)
        else:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[4], distanceShort, True, True)
        
        # Left
        if self.direction == NORTH:
            self.getValue(zip([r+1]*distanceLong, range(c-distanceLong-1, c-1))[::-1],
                          sensor_vals[5], distanceLong, False)
        elif self.direction == EAST:
            self.getValue(zip(range(r-distanceLong-1, r-1), [c-1]*distanceLong)[::-1],
                          sensor_vals[5], distanceLong, False)
        elif self.direction == WEST:
            self.getValue(zip(range(r+2, r+distanceLong+2), [c+1]*distanceLong),
                          sensor_vals[5], distanceLong, False)
        else:
            self.getValue(zip([r-1]*distanceLong, range(c+2, c+distanceLong+2)),
                          sensor_vals[5], distanceLong, False)
        
    # Checks to see if the robot is at a corner
    def is_corner(self):
        r, c = self.center
        # If the robot is facing north
        if self.direction == NORTH:
            fw = False # fw default is false
            # fw is true when robot at boundary of the arena's top or
            # there is obstacles directly on the robot's front centre, front left and front right
            fw = (r-2 < 0) or (r-2 >= 0 and self.exploredMap[r-2][c-1] == 2 and
                                 self.exploredMap[r-2][c] == 2
                                 and self.exploredMap[r-2][c+1] == 2)
            # rw is true when robot at boundary of the arena's right or
            # when there are obstacles directly on the robot's top right, middle right and bottom right
            rw = (c+2 == MAX_COLS) or (c+2 < MAX_COLS and self.exploredMap[r-1][c+2] == 2 
                                       and self.exploredMap[r+1][c+2] == 2)
            if fw and rw:
                return True
            else:
                return False
        elif self.direction == EAST:
            fw = (c+2 == MAX_COLS) or (c+2 < MAX_COLS and self.exploredMap[r-1][c+2] == 2 and
                                       self.exploredMap[r][c+2] == 2
                                       and self.exploredMap[r+1][c+2] == 2)
            rw = (r+2 == MAX_ROWS) or (r+2 < MAX_ROWS and self.exploredMap[r+2][c-1] == 2 
                                       and self.exploredMap[r+2][c+1] == 2)
            if fw and rw:
                return True
            else:
                return False
        elif self.direction == SOUTH:
            fw = (r+2 == MAX_ROWS) or (r+2 < MAX_ROWS and self.exploredMap[r+2][c-1] == 2 and
                                       self.exploredMap[r+2][c] == 2
                                       and self.exploredMap[r+2][c+1] == 2)
            rw = (c-2 == -1) or (c-2 >= 0 and self.exploredMap[r-1][c-2] == 2 
                                 and self.exploredMap[r+1][c-2] == 2)
            if fw and rw:
                return True
            else:
                return False
        else:
            fw = (c-2 == -1) or (c-2 >= 0 and self.exploredMap[r-1][c-2] == 2 and
                                 self.exploredMap[r][c-2] == 2
                                 and self.exploredMap[r+1][c-2] == 2)
            rw = (r-2 == -1) or (r-2 >= 0 and self.exploredMap[r-2][c-1] == 2 
                                 and self.exploredMap[r-2][c+1] == 2)
            if fw and rw:
                return True
            else:
                return False

    # Checks to see if there is a wall in front for the robot to calibrate
    def can_calibrate_front(self):
        r, c = self.center
        flag = [False, None]
        if self.direction == NORTH:
            if((r - 2) < 0):
                flag = [True, ALIGNFRONT]
            elif ((r - 2) >= 0 and (self.exploredMap[r-2][c-1] == 2 and self.exploredMap[r-2][c] == 2 and self.exploredMap[r-2][c+1] == 2)):
                flag = [True, ALIGNFRONT]
            elif ((r - 2) >= 0 and (self.exploredMap[r-2][c] == 2 and self.exploredMap[r-2][c+1] == 2)):
                flag = [True, ALIGNFRONT2]
            # elif ((r - 3) >= 0 and (self.exploredMap[r-2][c] == 1 and self.exploredMap[r-3][c] == 2 and self.exploredMap[r-2][c+1] == 2)):
            #     flag = [True, AlIGNFRONTSTAIR]
        elif self.direction == WEST:
            if((c - 2) < 0):
                flag = [True, ALIGNFRONT]
            elif ((c-2) >= 0 and (self.exploredMap[r-1][c-2] == 2 and self.exploredMap[r][c-2] == 2 and self.exploredMap[r+1][c-2] == 2)):
                flag = [True, ALIGNFRONT]
            elif ((c-2) >= 0 and (self.exploredMap[r][c-2] == 2 and self.exploredMap[r-1][c-2] == 2)):
                flag = [True, ALIGNFRONT2]
            # elif ((c-3) >= 0 and (self.exploredMap[r][c-2] == 1 and self.exploredMap[r][c-3] == 2 and self.exploredMap[r-1][c-2] == 2)):
            #     flag = [True, AlIGNFRONTSTAIR]
        elif self.direction == EAST:
            if((c + 2) == MAX_COLS):
                flag = [True, ALIGNFRONT]
            elif ((c + 2) < MAX_COLS and (self.exploredMap[r-1][c+2] == 2 and self.exploredMap[r][c+2] == 2 and self.exploredMap[r+1][c+2] == 2)):
                flag = [True, ALIGNFRONT]
            elif ((c + 2) < MAX_COLS and (self.exploredMap[r][c+2] == 2 and self.exploredMap[r+1][c+2] == 2)):
                flag = [True, ALIGNFRONT2]
            # elif ((c + 3) < MAX_COLS and (self.exploredMap[r][c+2] == 1 and self.exploredMap[r][c+3] == 2 and self.exploredMap[r+1][c+2] == 2)):
            #     flag = [True, AlIGNFRONTSTAIR]
        else:
            if((r+2) == MAX_ROWS):
                flag = [True, ALIGNFRONT]
            elif ((r+2) < MAX_ROWS and (self.exploredMap[r+2][c-1] == 2 and self.exploredMap[r+2][c] == 2 and self.exploredMap[r+2][c+1] == 2)):
                flag = [True, ALIGNFRONT]
            elif ((r+2) < MAX_ROWS and (self.exploredMap[r+2][c-1] == 2 and self.exploredMap[r+2][c] == 2)):
                flag = [True, ALIGNFRONT2]
            # elif ((r+3) < MAX_ROWS and (self.exploredMap[r+2][c-1] == 2 and self.exploredMap[r+2][c] == 1 and self.exploredMap[r+3][c]==2)):
            #     flag = [True, AlIGNFRONTSTAIR]
        return flag

    # Checks to see if there is a wall to the right of the robot for it to calibrate
    def can_calibrate_right(self):
        r, c = self.center
        flag = [False, None]
        if self.direction == NORTH:
            if((c + 2) == MAX_COLS):
                flag = [True, ALIGNRIGHT]
            elif ((c + 2) < MAX_COLS and (self.exploredMap[r-1][c+2] == 2 and self.exploredMap[r+1][c+2] == 2)):
                flag = [True, ALIGNRIGHT]
            elif ((c + 2) < MAX_COLS and (self.exploredMap[r][c+2] == 2 and self.exploredMap[r+1][c+2] == 2 and ((r+2< MAX_ROWS and self.exploredMap[r+2][c+2] != 2) or r+2 == MAX_ROWS))):
                flag = [True, ALIGNFRONT2]
        elif self.direction == WEST:
            if((r - 2) < 0):
                flag = [True, ALIGNRIGHT]
            elif ((r - 2) >= 0 and (self.exploredMap[r-2][c-1] == 2 and self.exploredMap[r-2][c+1] == 2)):
                flag = [True, ALIGNRIGHT]
            elif ((r - 2) >= 0 and (self.exploredMap[r-2][c] == 2 and self.exploredMap[r-2][c+1] == 2 and ((c+2< MAX_COLS and self.exploredMap[r-2][c+2] != 2) or c+2 ==MAX_ROWS) )):
                flag = [True, ALIGNFRONT2]
        elif self.direction == EAST:
            if((r+2) == MAX_ROWS):
                flag = [True, ALIGNRIGHT]
            elif ((r+2) < MAX_ROWS and (self.exploredMap[r+2][c-1] == 2 and self.exploredMap[r+2][c+1] == 2)):
                flag = [True, ALIGNRIGHT]
            elif ((r+2) < MAX_ROWS and (self.exploredMap[r+2][c-1] == 2 and self.exploredMap[r+2][c] == 2 and ((c-2>=0 and self.exploredMap[r+2][c-2] != 2) or c-2 <0) )):
                flag = [True, ALIGNFRONT2]
        else:
            if((c - 2) < 0):
                flag = [True, ALIGNRIGHT]
            elif ((c-2) >= 0 and (self.exploredMap[r-1][c-2] == 2 and self.exploredMap[r+1][c-2] == 2)):
                flag = [True, ALIGNRIGHT]
            elif ((c-2) >= 0 and (self.exploredMap[r-1][c-2] == 2 and self.exploredMap[r][c-2] == 2 and ((r-2>=0 and self.exploredMap[r-2][c-2]!=2)or r-2<0))):
                flag = [True, ALIGNFRONT2]
        return flag

    def moveBot(self, movement):
        """Simulates the bot movement based on current location, direction and received action

        Args:
            movement (string): Next movement received (see Constants)
        """
        self.stepCounter += 1
        self.movement.append(movement)
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            elif movement == FORWARD or movement == FORWARDFAST:
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1)
            elif movement == BACKWARDS or movement == BACKWARDSFAST:
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
            self.setHead()
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            elif movement == FORWARD or movement == FORWARDFAST:
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
            elif movement == BACKWARDS or movement == BACKWARDSFAST:
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
            self.setHead()
        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            elif movement == FORWARD or movement == FORWARDFAST:
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
            elif movement == BACKWARDS or movement == BACKWARDSFAST:
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1)
            self.setHead()
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            elif movement == FORWARD or movement == FORWARDFAST:
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
            elif movement == BACKWARDS or movement == BACKWARDSFAST:
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
            self.setHead()


    # def fillUnexplored(self):
    #     #fill the unexplored cells with blocks
    #     for i in range(MAX_ROWS):
    #         for j in range(MAX_COLS):
    #             if self.exploredMap[i][j] == 0:
    #                 self.exploredMap[i][j] = 2


    def descriptor_1(self):
        descriptor = np.zeros([20, 15]).astype(int)
        descriptor[self.exploredMap[::-1, :] != 0] = 1
        bits = '11'
        for i in range(len(descriptor)):
            for j in range(len(descriptor[i])):
                bits += str(descriptor[i][j])
        bits += '11'
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)

    # def descriptor_2(self):
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

    def descriptor_3(self):
        bits = ''
        for row in self.exploredMap[::-1, :]:
            for bit in row:
                if bit == 2:
                    bits += '1'
                elif bit == 1:
                    bits += '0'
        bits += '0'*(4 - len(bits) % 4)
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
        hex_str = hex_str[:-1]
        return ''.join(hex_str)