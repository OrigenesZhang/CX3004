import numpy as np
import time

from Constants import NORTH, SOUTH, WEST, EAST, FORWARD, LEFT, RIGHT, START, MAX_ROWS, MAX_COLS, ALIGNFRONT, ALIGNRIGHT


"""
instead of forward use fastforward
detect phantom blocks (if possible)
implement backwards for fastest path ?!
Use ultrasonic sensor instead of short/long sensor ---> not much time difference (tested in one map)

"""

class Exploration:

    """Implementation of the Right-Wall hugging algorithm for a maze solving
       robot.
       The implementation assumes that the robot starts at the bottom-left corner of the map,
       i.e. (MAX_ROWS - 2, 1). And the robot is facing North

    Attributes:
        currentMap (Numpy array): To store the current state of the exploration map
        exploredArea (float): Percentage of arena explored
        robot (Robot): Instance of the Robot class
        sensors (list of Numpy arrays): Readings from all sensors
        timeLimit (int): Maximum time allowed for exploration
    """

    def __init__(self, realMap=None, timeLimit=None, calibrateLim=6, sim=True):
        """Constructor to initialise an instance of the Exploration class

        Args:
            realMap (string): File name for real map during simulation stage
            timeLimit (int): Maximum time allowed for exploration
            sim (bool, optional): To specify is the exploration mode is simulation or real
        """
        self.timeLimit = timeLimit
        self.exploredArea = 0
        self.currentMap = np.zeros([20, 15])
        # Mark start as free
        self.currentMap[17:20, 0:3] = 1
        self.currentMap[0:3, 12:15] = 1
        # call class according to running simulator or real run
        if sim:
            from Simulator import Robot
            self.robot = Robot(self.currentMap, EAST, START, realMap)
            self.sensors = self.robot.getSensors() # Returns a numpy array of numpy arrays of the sensor values from all sensors
        else:
            from Real import Robot
            self.robot = Robot(self.currentMap, EAST, START)
        self.exploredNeighbours = dict()
        self.sim = sim
        # self.calibrateLim = calibrateLim
        # initialize as boundary of the arena
        # Set the limits of unexplored area on the map
        self.virtualWall = [0, 0, MAX_ROWS, MAX_COLS]
        self.alignRightCount = 0
        self.justCheckedRight = False

    def __validInds(self, inds):
        """To check if the passed indices are valid or not
        To be valid the following conditions should be met:
            * A 3x3 neighbourhood around the center should lie within the arena
            * A 3x3 neighbourhood around the center should have no obstacle

        Args:
            inds (list of list): List of coordinates to be checked

        Returns:
            list of list: All indices that were valid
        """
        valid = []
        for i in inds:
            r, c = i
            x, y = np.meshgrid([-1, 0, 1], [-1, 0, 1])
            # np.meshgrid([-1, 0, 1], [-1, 0, 1]) outputs
            # [array([[-1,  0,  1],
            #        [-1,  0,  1],
            #        [-1,  0,  1]]),
            # array([[-1, -1, -1],
            #        [ 0,  0,  0],
            #        [ 1,  1,  1]])]
            # x is row coordinate while y is column coordinate
            x, y = x+r, y+c
            # If any of the x coordinate is less than zero
            # or if any of the y coordinate is less than
            # or if any of the x coordinate is greater than or equal to the maximum number of rows
            # or if any of the y coordinates is greater than or equal to the maximum number of columns
            if (np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS)):
                # Indicate that the coordinate is invalid
                valid.append(False)
            # If the value of any of the coordinate is not 1 (meaning that it is free)
            # We assume that there is an obstacle in the 3 by 3 area
            elif (np.any(self.currentMap[x[0, 0]:x[0, 2]+1, y[0, 0]:y[2, 0]+1] != 1)):
                # Indicate that the coordinate is invalid
                valid.append(False)
            # If coordinates within arena and there are no obstacles
            else:
                # Indicate that the coordinate is valid
                valid.append(True)
        # Return all valid coordinates
        return [tuple(inds[i]) for i in range(len(inds)) if valid[i]]

    def getExploredArea(self):
        """Updates the total number of cells explored at the current state
        """
        # returns percent of map explored
        self.exploredArea = (np.sum(self.currentMap != 0)/300.0)*100


    def nextMove(self):
        # Decides which direction is free and commands the robot the next action
        move = []

        if(not (self.sim) and self.robot.is_corner() == True):
            move = [RIGHT, ALIGNFRONT, LEFT, ALIGNFRONT, LEFT, ALIGNRIGHT]
            self.robot.moveBot(LEFT)
        else:
            if not (self.sim):
                # Check if there is a wall in front for robot to calibrate
                calibrate_front = self.robot.can_calibrate_front()
                # Check if there is a wall to the right for the robot to calibrate
                calibrate_right = self.robot.can_calibrate_right()
                # If the robot is at a corner
                # if self.robot.is_corner():
                #     move.append(RIGHT)
                #     move.append(ALIGNFRONT)
                #     move.append(LEFT)
                #     move.append(ALIGNFRONT)
                # If robot is not at a corner but there is a wall to the right for calibration
                if (calibrate_right[0]):
                    self.alignRightCount += 1
                    if(self.alignRightCount % 3) == 0:
                        move.append(RIGHT)
                        move.append(ALIGNFRONT)
                        move.append(LEFT)
                        move.append(ALIGNRIGHT)
                        #added , may remove later
                    else:
                        # Append command from can_calibrate_right function
                        move.append(calibrate_right[1])
                # If robot is not at a corner and there is no wall to the right of the robot
                # If there is a wall to the front for calibration
                elif (calibrate_front[0]):
                    # Append command from can_calibrate_front function
                    move.append(calibrate_front[1])
            # multi step
            # Number of spaces in front of the robot which is free and where there are obstacles on the right
            # and all spaces detectable by the left middle, right top and right bottom sensors at these spaces have been explored
            front = self.frontFree()
            
            # If right of robot is free
            if(self.justCheckedRight == True):
                self.justCheckedRight = False
                if (front):
                    # Move robot forward
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                elif (self.checkFree([1, 2, 3, 0], self.robot.center)):
                    # Move robot to the right
                    self.robot.moveBot(RIGHT)
                    move.append(RIGHT)
                    front = self.frontFree()
                    # Move robot forward according to frontFree function
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                # Else if the robot's left is free
                elif (self.checkFree([3, 0, 1, 2], self.robot.center)):
                    # Move robot to the left
                    self.robot.moveBot(LEFT)
                    move.append(LEFT)
                    front = self.frontFree()
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                elif(self.checkLeftUnexplored()):
                    self.robot.moveBot(LEFT)
                    move.append(LEFT)
                # Else, turn the robot around
                else:
                    self.robot.moveBot(RIGHT)
                    self.robot.moveBot(RIGHT)
                    move.append(RIGHT)
                    move.append(RIGHT)
            else:   
                if (self.checkFree([1, 2, 3, 0], self.robot.center)):
                    # Move robot to the right
                    self.robot.moveBot(RIGHT)
                    move.append(RIGHT)
                    front = self.frontFree()
                    # Move robot forward according to frontFree function
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                elif(self.checkRightUnexplored()):
                    self.justCheckedRight = True
                    self.robot.moveBot(RIGHT)
                    move.append(RIGHT)
                # If front > 0
                elif (front):
                    # Move robot forward
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                # Else if the robot's left is free
                elif (self.checkFree([3, 0, 1, 2], self.robot.center)):
                    # Move robot to the left
                    self.robot.moveBot(LEFT)
                    move.append(LEFT)
                    front = self.frontFree()
                    for i in range(front):
                        self.robot.moveBot(FORWARD)
                    move.extend([FORWARD]*front)
                elif(self.checkLeftUnexplored()):
                    self.robot.moveBot(LEFT)
                    move.append(LEFT)
                # Else, turn the robot around
                else:
                    self.robot.moveBot(RIGHT)
                    self.robot.moveBot(RIGHT)
                    move.append(RIGHT)
                    move.append(RIGHT)
        # Return list of moves
        return move

    def checkFree(self, order, center):
        """Checks if a specific direction is free to move to

        Args:
            order (list): Ordering for the directionFree list based on the
                          the next move (Right, Left, Forward)

        Returns:
            bool: If the queried direction is free
        """
        # Creates an array of boolean values
        directionFree = np.asarray([self.northFree(center), self.eastFree(center),
                                    self.southFree(center), self.westFree(center)])
        directionFree = directionFree[order]
        # If directionFree = np.asarray(["North", "East", "South", "West"])
        # directionFree[[1, 2, 3, 0]] = ['East' 'South' 'West' 'North']
        # directionFree[[3, 0, 1, 2]] = ['West' 'North' 'East' 'South']
        if self.robot.direction == NORTH:
            return directionFree[0]
        elif self.robot.direction == EAST:
            return directionFree[1]
        elif self.robot.direction == SOUTH:
            return directionFree[2]
        else:
            return directionFree[3]

    def checkLeftUnexplored(self):
        r, c = self.robot.center
        if self.robot.direction == NORTH:
            if(c-2 < 0):
                return False
            return (self.currentMap[r][c - 2] == 0 or self.currentMap[r+1][c - 2] == 0)
        elif self.robot.direction == EAST:
            if(r-2 < 0):
                return False
            return (self.currentMap[r-2][c] == 0 or self.currentMap[r-2][c-1] == 0)
        elif self.robot.direction == SOUTH:
            if(c+2 >= MAX_COLS):
                return False
            return (self.currentMap[r][c+2] == 0 or self.currentMap[r-1][c+2] == 0)
        else:
            if(r+2 >= MAX_ROWS):
                return False
            return (self.currentMap[r+2][c] == 0 or self.currentMap[r+2][c+1] == 0)
    
    def checkRightUnexplored(self):
        r, c = self.robot.center
        if self.robot.direction == NORTH:
            if(c+2 >= MAX_COLS):
                return False
            return (self.currentMap[r][c + 2] == 0)
        elif self.robot.direction == EAST:
            if(r+2 >= MAX_ROWS):
                return False
            return (self.currentMap[r+2][c] == 0)
        elif self.robot.direction == SOUTH:
            if(c-2 < 0):
                return False
            return (self.currentMap[r][c-2] == 0)
        else:
            if(r-2 < 0):
                return False
            return (self.currentMap[r-2][c] == 0)
    
    def validMove(self, inds):
        """Checks if all the three cells on one side of the robot are free

        Args:
            inds (list of list): List of cell indices to be checked

        Returns:
            bool: If all indices are free (no obstacle)
        """
        for (r, c) in inds:
            # self.virtualWall indicates the boundaries of unexplored area
            # If the indices are not within the unexplored area
            if not ((0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS)):
                # Indicate that the move is not valid
                return False
        # Check if all three indices have a value of 1 (Explored and is free)
        return (self.currentMap[inds[0][0], inds[0][1]] == 1 and
                self.currentMap[inds[1][0], inds[1][1]] == 1 and
                self.currentMap[inds[2][0], inds[2][1]] == 1)

    def northFree(self, center):
        """Checks if the north direction is free to move

        Returns:
            bool: if north is free
        """
        r, c = center
        # Checks if the three spaces immediately to the north of the robot is free and within the unexplored area
        inds = [[r-2, c], [r-2, c-1], [r-2, c+1]]
        return self.validMove(inds)

    def eastFree(self, center):
        """Checks if the east direction is free to move

        Returns:
            bool: if east is free
        """
        r, c = center
        # Checks if the three spaces immediately to the east of the robot is free and within the unexplored area
        inds = [[r, c+2], [r-1, c+2], [r+1, c+2]]
        return self.validMove(inds)

    def southFree(self, center):
        """Checks if the south direction is free to move

        Returns:
            bool: if south is free
        """
        r, c = center
        # Checks if the three spaces immediately to the south of the robot is free and within the unexplored area
        inds = [[r+2, c], [r+2, c-1], [r+2, c+1]]
        return self.validMove(inds)

    def westFree(self, center):
        """Checks if the west direction is free to move

        Returns:
            bool: if west is free
        """
        r, c = center
        # Checks if the three spaces immediately to the west of the robot is free and within the unexplored area
        inds = [[r, c-2], [r-1, c-2], [r+1, c-2]]
        return self.validMove(inds)

    def frontFree(self):
        """
        returns number of spaces in front of the robot which is free and where there are obstacles on the right
        and all spaces detectable by the left middle, right top and right bottom sensors at these spaces have been explored
        """
        r, c = self.robot.center
        counter = 0 # Keeps track of number of moves the robot can make that are directly in front
        # If robot is facing north and three spaces immediately to the north of robot is within unexplored boundaries
        # and has no obstacles
        
        if self.robot.direction == NORTH and self.validMove([[r-2, c], [r-2, c-1], [r-2, c+1]]):
            # Increase counter by 1
            counter = 1
            #     # If all three spaces immediately to the north of the robot is free
            #     # and the three spaces immediately to the east of the robot is not free
            #     # and all coordinates reachable by sensors are explored at position with coordinate [r-counter, c]
            #     # increase counter by 1

            # while (True):
            #     if (self.validMove([[r-2-counter, c], [r-2-counter, c-1], [r-2-counter, c+1]])) and self.eastFree([r-counter, c]):
            #         counter += 1
            #     else:
            #         break

            #             not self.checkFree([1, 2, 3, 0], [r-(counter), c]) and\
            #             self.checkExplored([r-(counter), c]):
            #     else:
            #         break
        elif self.robot.direction == EAST and self.validMove([[r, c+2], [r-1, c+2], [r+1, c+2]]):
            counter = 1
            # while(True):
            #     if (self.validMove([[r, c+2+counter], [r-1, c+2+counter], [r+1, c+2+counter]])) and\
            #             not self.checkFree([1, 2, 3, 0], [r, c+(counter)]) and\
            #             self.checkExplored([r, c+(counter)]):
            #         counter += 1
            #     else:
            #         break
        elif self.robot.direction == WEST and self.validMove([[r, c-2], [r-1, c-2], [r+1, c-2]]):
            counter = 1
            # while(True):
            #     if (self.validMove([[r, c-2-counter], [r-1, c-2-counter], [r+1, c-2-counter]])) and\
            #             not self.checkFree([1, 2, 3, 0], [r, c-(counter)]) and\
            #             self.checkExplored([r, c-(counter)]):
            #         counter += 1
            #     else:
            #         break
        elif self.robot.direction == SOUTH and self.validMove([[r+2, c], [r+2, c-1], [r+2, c+1]]):
            counter = 1
            # while(True):
            #     if (self.validMove([[r+2+counter, c], [r+2+counter, c-1], [r+2+counter, c+1]])) and\
            #             not self.checkFree([1, 2, 3, 0], [r+(counter), c]) and\
            #             self.checkExplored([r+(counter), c]):
            #         counter += 1
            #     else:
            #         break
        return counter

    # ***Update according to sensor placement
    #not using
    def checkExplored(self, center):
        r, c = center
        flag = True
        inds = []
        distanceShort = 2
        distanceLong = 4
        if self.robot.direction == NORTH:
            # If r = 5, c = 8, distanceShort = 3 and distanceLong = 5
            # zip([r-1]*distanceShort, range(c+2, c+distanceShort+2))
            # gives [(4, 10), (4, 11), (4, 12)]
            inds.append(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)))
            # zip([r+1]*distanceLong, range(c+2, c+distanceLong+2))
            # gives [(6, 10), (6, 11), (6, 12), (6, 13), (6, 14)]
            inds.append(zip([r+1]*distanceLong, range(c+2, c+distanceLong+2)))
            # zip([r]*distanceLong, range(c-distanceLong-1, c-1))[::-1]
            # gives [(5, 6), (5, 5), (5, 4), (5, 3), (5, 2)]
            inds.append(zip([r]*distanceLong, range(c-distanceLong-1, c-1))[::-1])
        elif self.robot.direction == EAST:
            # If r = 5, c = 8, distanceShort = 3 and distanceLong = 5
            # zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort)
            # [(7, 9), (8, 9), (9, 9)]
            inds.append(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort))
            # zip(range(r+2, r+distanceLong+2), [c-1]*distanceLong)
            # gives [(7, 7), (8, 7), (9, 7), (10, 7), (11, 7)]
            inds.append(zip(range(r+2, r+distanceLong+2), [c-1]*distanceLong))
            # zip(range(r-distanceLong-1, r-1), [c]*distanceLong)[::-1]
            # gives [(3, 8), (2, 8), (1, 8), (0, 8), (-1, 8)]
            inds.append(zip(range(r-distanceLong-1, r-1), [c]*distanceLong)[::-1])
        elif self.robot.direction == WEST:
            # zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1]
            # gives [(3, 7), (2, 7), (1, 7)]
            inds.append(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1])
            # zip(range(r-distanceLong-1, r-1), [c+1]*distanceLong)[::-1]
            # gives [(3, 9), (2, 9), (1, 9), (0, 9), (-1, 9)]
            inds.append(zip(range(r-distanceLong-1, r-1), [c+1]*distanceLong)[::-1])
            # zip(range(r+2, r+distanceLong+2), [c]*distanceLong)
            # gives [(7, 8), (8, 8), (9, 8), (10, 8), (11, 8)]
            inds.append(zip(range(r+2, r+distanceLong+2), [c]*distanceLong))
        else: # self.direction == SOUTH
            # zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1]
            # gives [(6, 6), (6, 5), (6, 4)]
            inds.append(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1])
            # zip([r-1]*distanceLong, range(c-distanceLong-1, c-1))[::-1]
            # gives [(4, 6), (4, 5), (4, 4), (4, 3), (4, 2)]
            inds.append(zip([r-1]*distanceLong, range(c-distanceLong-1, c-1))[::-1])
            # zip([r]*distanceLong, range(c+2, c+distanceLong+2))
            # [(5, 10), (5, 11), (5, 12), (5, 13), (5, 14)]
            inds.append(zip([r]*distanceLong, range(c+2, c+distanceLong+2)))
        # For each list of coordinates
        for sensor in inds:
            if flag:
                # For each coordinate
                for (x, y) in sensor:
                    # If x coordinate is outside the unexplored boundaries
                    # or x coordinate is equal to upper limit of unexplored boundaries in terms of row
                    # or if y is outside the unexplored boundaries
                    # or if y is equal to upper limit of unexplored boundaries in terms of column
                    # of if the coordinate is an obstacle
                    # break out of the for loop, where flag will still be true
                    if (x < self.virtualWall[0] or x == self.virtualWall[2] or
                            y < self.virtualWall[1] or y == self.virtualWall[3] or
                            self.currentMap[x, y] == 2):
                        break
                    # If the coordinate has a value of 0, indicating that it is unexplored
                    # Set flag to be False and break out of the loop
                    elif (self.currentMap[x, y] == 0):
                        flag = False
                        break
        # return final value of flag
        return flag


    def moveStep(self, sensor_vals=None):
        """Moves the robot one step for exploration

        Returns:
            bool: True is the map is fully explored
        """
        # If sensor values are given, update self.exploredMap
        if (sensor_vals):
            self.robot.getSensors(sensor_vals)
        else:
            self.robot.getSensors()
        move = self.nextMove()
        self.getExploredArea()
        # If full coverage
        if (self.exploredArea == 100):
            return move, True
       # if self.robot.center[0]==START[0] and self.robot.center[1]==START[1] and self.exploredArea>50:
        #    return move,True
        else:
            return move, False

    def explore(self):
        """Runs the exploration till the map is fully explored of time runs out
        """
        print("Starting exploration ...")
        startTime = time.time()
        endTime = startTime + self.timeLimit

        while(time.time() <= endTime):
            if (self.moveStep()):
                print("Exploration completed !")
                return

        print ("Time over !")
        return

    def getCloseExploredNeighbour(self):
        locs = np.where(self.currentMap == 0)
        if(len(locs[0]) == 0):
            return None
        self.virtualWall = [np.min(locs[0]), np.min(locs[1]), np.max(locs[0])+1, np.max(locs[1])+1]
        if ((self.virtualWall[2]-self.virtualWall[0] < 3) and self.virtualWall[2] < MAX_ROWS-3):
            self.virtualWall[2] += 3
        locs = list(np.asarray(zip(locs[0], locs[1])))

        if(0 <= self.robot.center[0] < 10):
            if(0 <= self.robot.center[1] < 8):
                locs = [loc for loc in locs if 0 <= loc[0] < 10 and 0 <= loc[1] < 8]
            else:
                locs = [loc for loc in locs if 0 <= loc[0] < 10 and 8 <= loc[1] < 15]
        else:
            if(0 <= self.robot.center[1] < 8):
                locs = [loc for loc in locs if 10 <= loc[0] < 20 and 0 <= loc[1] < 8]
            else:
                locs = [loc for loc in locs if 10 <= loc[0] < 20 and 8 <= loc[1] < 15]
        if(len(locs) == 0):
            return None
        locs = np.asarray(locs)
        cost = np.abs(locs[:, 0] - self.robot.center[0]) + np.abs(locs[:, 1] - self.robot.center[1])
        cost = cost.tolist()
        # for each coordinate the cost is (x- center_x   + y- center_y)
        # the distance of the neighbour away from the center of robot
        locs = locs.tolist()
        while (cost):
            position = np.argmin(cost) # position gives index of minimum cost
            coord = locs.pop(position)
            cost.pop(position)
            neighbours = np.asarray([[-2, 0], [-2, -1], [-2, 1], [2, 0], [2, -1], [2, 1], [0, -2], [1, -2], [-1, -2], [0, 2], [1, 2], [-1, 2]]) + coord
            neighbours = self.__validInds(neighbours) # check if the coordinates in the list are valid or not
            for neighbour in neighbours:
                if(0 <= self.robot.center[0] < 10):
                    if(0 <= self.robot.center[1] < 8):
                        if(neighbour not in self.exploredNeighbours and 0 <= neighbour[0] < 10 and 0 <= neighbour[1] < 8):
                            self.exploredNeighbours[neighbour] = True
                            return neighbour
                    else:
                        if (neighbour not in self.exploredNeighbours and 0 <= neighbour[0] < 10 and 8 <= neighbour[1] < 15):
                            self.exploredNeighbours[neighbour] = True
                            return neighbour
                else:
                    if(10 <= self.robot.center[0] < 20):
                        if (neighbour not in self.exploredNeighbours and 10 <= neighbour[0] < 20 and 0 <= neighbour[1] < 8):
                            self.exploredNeighbours[neighbour] = True
                            return neighbour
                    else:
                        if (neighbour not in self.exploredNeighbours and 10 <= neighbour[0] < 20 and 8 <= neighbour[1] < 15):
                            self.exploredNeighbours[neighbour] = True
                            return neighbour
        return None
        


    # Get valid and explored neighbours of unexplored spaces
    def getExploredNeighbour(self):
        # initial virtual wall = [0, 0, MAX_ROWS, MAX_COLS]

        # returns all location where current map is unexplored, with x coordinate in one array and y coordinate in different array
        locs = np.where(self.currentMap == 0)
        if(len(locs[0]) == 0):
            return None
        # forms a virtual arena of unexplored area as rectangle
        self.virtualWall = [np.min(locs[0]), np.min(locs[1]), np.max(locs[0])+1, np.max(locs[1])+1]
        if ((self.virtualWall[2]-self.virtualWall[0] < 3) and self.virtualWall[2] < MAX_ROWS-3):
            self.virtualWall[2] += 3
        # if there is more than 3 blocks between this new wall and the actual arean wall, we move the virtual wall 3 blocks away from the top of areana to
        # accomodaate the size of the robot
        # new virtual arena is    [ unexplored rows +3  x  unexplored column]
        locs = np.asarray(zip(locs[0], locs[1]))
        # converts locs back into unexplored coordinates
        cost = np.abs(locs[:, 0] - self.robot.center[0]) + np.abs(locs[:, 1] - self.robot.center[1])
        cost = cost.tolist()
        # for each coordinate the cost is (x- center_x   + y- center_y)
        # the distance of the neighbour away from the center of robot
        locs = locs.tolist()
        while (cost):
            position = np.argmin(cost) # position gives index of minimum cost
            coord = locs.pop(position)
            cost.pop(position)
            neighbours = np.asarray([[-2, 0], [-2, -1], [-2, 1], [2, 0], [2, -1], [2, 1], [0, -2], [1, -2], [-1, -2], [0, 2], [1, 2], [-1, 2]]) + coord
            neighbours = self.__validInds(neighbours) # check if the coordinates in the list are valid or not
            for neighbour in neighbours:
                # added a new item to the exploredNeighbours dictionary
                if (neighbour not in self.exploredNeighbours):
                    self.exploredNeighbours[neighbour] = True
                    return neighbour
                return neighbour
        return None
