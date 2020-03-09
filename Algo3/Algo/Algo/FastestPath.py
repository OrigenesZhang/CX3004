import copy
import numpy as np

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, FORWARD, LEFT, RIGHT, BACKWARDS


class Node:

    """To create nodes of a graph

    Attributes:
        coord (tuple): The (row, col) coordinate
        G (int): The cost to reach this node
        H (int): The cost to goal node
        parent (Node): The parent of this node
        value (int): The value at this location in exploration map
    """

    def __init__(self, value, coord, H):
        """Constructor to initialize an instance of Node class

        Args:
            value (int): The value of the node
            coord (tuple): The (row, col) coordinate
            H (int): The cost to goal node
        """
        self.value = value
        self.coord = coord
        self.parent = None
        self.H = H
        self.G = float('inf')


class FastestPath:

    """Implementation of the A* algorithm for getting the fastest path in a 2D grid maze

    Attributes:
        exploredMap (Numpy array): The map after the exploration stage
        goal (list): Coordinate of the goal cell
        graph (list): To form the fastest path graph
        index (int): Counter
        path (list): The fastest path
        robot (Robot): Instance of the Robot class
        start (list): Coordinate of the initial cell
        waypoint (list): Coordinate of the way-point if specified
    """

    def __init__(self, exploredMap, start, goal, direction, waypoint=None, calibrateLim=5, sim=True):
        """Constructor to initialize an instance of the FastestPath class

        Args:
            exploredMap (Numpy array): The map after the exploration stage
            start (list): Coordinate of the initial cell
            goal (list): Coordinate of the goal cell
            direction (int): The starting direction of the robot
            waypoint (list, optional): The coordinate of the way-point if specified
            sim (bool, optional): If specify if run is simulation or real
        """
        self.exploredMap = exploredMap
        self.graph = []
        self.start = start
        self.goal = goal
        self.waypoint = waypoint
        self.index = 1
        self.direction = direction
        self.path = []
        self.movement = []
        self.calibrateLim = calibrateLim
        self.sim = sim
        if sim:
            from Simulator import Robot
            self.robot = Robot(self.exploredMap, direction, start, None)
        else:
            from Real import Robot
            self.robot = Robot(self.exploredMap, direction, start)

    def __getHeuristicCosts(self, goal):
        """Calculates the Manhattan distance between each cell and the goal cell

        Args:
            goal (list): Coordinates of the goal cell

        Returns:
            Numpy array: 2D grid with each cell storing the distance to the goal cell
        """
        # this will create a grid of coordinates
        cols, rows = np.meshgrid(range(0, 15), range(0, MAX_ROWS))
        cost = np.zeros([MAX_ROWS, 15])
        cost = np.sqrt(np.square(rows - goal[0]) + np.square(cols - goal[1]))
        cost /= np.max(cost)
        return cost
        # e.g. If goal = np.asarray([1, 13])
        # cost will be
        # [[0.58722022 0.54232614 0.49745804 0.45262363 0.40783404 0.36310583
        #   0.31846488 0.27395385 0.22964829 0.18569534 0.14242182 0.10070744
        #   0.06369298 0.04503773 0.06369298]
        #  [0.58549055 0.54045282 0.49541508 0.45037735 0.40533961 0.36030188
        #  0.31526414 0.27022641 0.22518867 0.18015094 0.1351132  0.09007547
        #  0.04503773 0.         0.04503773]
        # [0.58722022
        # 0.54232614
        # 0.49745804
        # 0.45262363
        # 0.40783404
        # 0.36310583
        # 0.31846488
        # 0.27395385
        # 0.22964829
        # 0.18569534
        # 0.14242182
        # 0.10070744
        # 0.06369298
        # 0.04503773
        # 0.06369298]
        # [0.59237891 0.54790769 0.50353718 0.45929658 0.4152274  0.37139068
        #  0.32787966 0.28484365 0.24253563 0.20141487 0.16238586 0.12738595
        #  0.10070744 0.09007547 0.10070744]
        # [0.60087833
        # 0.55708601
        # 0.51350919
        # 0.47020776
        # 0.42726547
        # 0.38480258
        # 0.34299717
        # 0.30212231
        # 0.26261287
        # 0.22518867
        # 0.19107893
        # 0.16238586
        # 0.14242182
        # 0.1351132
        # 0.14242182]
        # [0.61257942 0.56968729 0.52715317 0.48507125 0.44357025 0.40282975
        #  0.36310583 0.32477173 0.28838221 0.2547719  0.22518867 0.20141487
        #  0.18569534 0.18015094 0.18569534]
        # [0.62730306
        # 0.58549055
        # 0.54419302
        # 0.50353718
        # 0.46369186
        # 0.42488514
        # 0.38742924
        # 0.35175595
        # 0.31846488
        # 0.28838221
        # 0.26261287
        # 0.24253563
        # 0.22964829
        # 0.22518867
        # 0.22964829]
        # [0.64484223 0.60424462 0.5643212  0.52522573 0.48715759 0.45037735
        #  0.4152274  0.38215785 0.35175595 0.32477173 0.30212231 0.28484365
        #  0.27395385 0.27022641 0.27395385]
        # [0.66497419
        # 0.62568421
        # 0.58722022
        # 0.54975562
        # 0.51350919
        # 0.47875769
        # 0.44585083
        # 0.4152274
        # 0.38742924
        # 0.36310583
        # 0.34299717
        # 0.32787966
        # 0.31846488
        # 0.31526414
        # 0.31846488]
        # [0.68747119 0.64954345 0.61257942 0.57676442 0.54232614 0.5095438
        #  0.47875769 0.45037735 0.42488514 0.40282975 0.38480258 0.37139068
        #  0.36310583 0.36030188 0.36310583]
        # [0.71210911
        # 0.67556602
        # 0.64010648
        # 0.60592075
        # 0.57323678
        # 0.54232614
        # 0.51350919
        # 0.48715759
        # 0.46369186
        # 0.44357025
        # 0.42726547
        # 0.4152274
        # 0.40783404
        # 0.40533961
        # 0.40783404]
        # [0.73867377 0.70351191 0.66953406 0.63692976 0.60592075 0.57676442
        #  0.54975562 0.52522573 0.50353718 0.48507125 0.47020776 0.45929658
        #  0.45262363 0.45037735 0.45262363]
        # [0.76696499
        # 0.73316121
        # 0.70062273
        # 0.66953406
        # 0.64010648
        # 0.61257942
        # 0.58722022
        # 0.5643212
        # 0.54419302
        # 0.52715317
        # 0.51350919
        # 0.50353718
        # 0.49745804
        # 0.49541508
        # 0.49745804]
        # [0.79679887 0.76431571 0.73316121 0.70351191 0.67556602 0.64954345
        #  0.62568421 0.60424462 0.58549055 0.56968729 0.55708601 0.54790769
        #  0.54232614 0.54045282 0.54232614]
        # [0.82800868
        # 0.79679887
        # 0.76696499
        # 0.73867377
        # 0.71210911
        # 0.68747119
        # 0.66497419
        # 0.64484223
        # 0.62730306
        # 0.61257942
        # 0.60087833
        # 0.59237891
        # 0.58722022
        # 0.58549055
        # 0.58722022]
        # [0.86044472 0.8304548  0.80187407 0.77485849 0.7495773  0.72621165
        #  0.70495206 0.68599434 0.66953406 0.65575932 0.64484223 0.63692976
        #  0.63213473 0.63052829 0.63213473]
        # [0.89397351
        # 0.86514664
        # 0.8377503
        # 0.81192931
        # 0.7878386
        # 0.76564149
        # 0.74550716
        # 0.72760688
        # 0.71210911
        # 0.69917366
        # 0.68894487
        # 0.6815446
        # 0.67706562
        # 0.67556602
        # 0.67706562]
        # [0.92847669 0.9007547  0.87447463 0.84977028 0.82678291 0.80565949
        #  0.78655023 0.76960515 0.75497001 0.74278135 0.73316121 0.72621165
        #  0.72200982 0.72060376 0.72200982]
        # [0.96384962
        # 0.93717455
        # 0.91194463
        # 0.88828298
        # 0.86631813
        # 0.84618221
        # 0.82800868
        # 0.81192931
        # 0.7980707
        # 0.78655023
        # 0.77747185
        # 0.77092184
        # 0.76696499
        # 0.76564149
        # 0.76696499]
        # [1.         0.97431518 0.95007206 0.92738372 0.90636693 0.88714049
        #  0.86982314 0.85453094 0.84137432 0.8304548  0.82186154 0.81566807
        #  0.81192931 0.81067923 0.81192931]]

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
            # x: [[-1  0  1]
            #  [-1  0  1]
            #  [-1  0  1]]
            # y: [[-1 -1 -1]
            #  [ 0  0  0]
            #  [ 1  1  1]]
            x, y = x+r, y+c
            # e.g. If r = 5 and c = 12
            # x: [[4 5 6]
            #  [4 5 6]
            #  [4 5 6]]
            # y: [[11 11 11]
            #  [12 12 12]
            #  [13 13 13]]
            if np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS):
                valid.append(False)
            elif (np.any(self.exploredMap[x[0, 0]:x[0, 2]+1, y[0, 0]:y[2, 0]+1] != 1)):
                # If any of the space within the 3 by 3 space is not a free space, append false
                valid.append(False)
            else:
                valid.append(True)
        return [inds[i] for i in range(len(inds)) if valid[i]] # Returns only valid indices

    def __getNeighbours(self, loc):
        """Returns the coordinates of a 3x3 neighbourhood around the passed location
        Only those neighbours are returned that have been explored and are not obstacles

        Args:
            loc (list): Coordinates of a cell

        Returns:
            list of list: Coordinates of all valid neighbours of the cell
        """
        r, c = loc.coord
        inds = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        inds = self.__validInds(inds)
        neighbours = [self.graph[n[0]][n[1]] for n in inds]
        # if its explored and not an obstacle
        return [n for n in neighbours if n.value == 1]


    def __getCost(self, current_pos, next_pos):
        # Cost is one if the robot needs to turn if it wants to move from current position to the next position
        if self.direction in [NORTH, SOUTH]:
            if current_pos[1] == next_pos[1]:
                return 0
            else:
                return 1
        else:
            if current_pos[0] == next_pos[0]:
                return 0
            else:
                return 1
        return 1


    # Function to set current direction of robot based on previous and current position
    def __setDirection(self, prev_pos, current_pos, backwards = False):
        # If row index increases, the robot has moved south
        if prev_pos[0] < current_pos[0]:
            if(backwards == False):
                self.direction = SOUTH
            else:
                self.direction = NORTH
        # If the column index increases, the robot has moved east
        elif prev_pos[1] < current_pos[1]:
            if(backwards == False):
                self.direction = EAST
            else:
                self.direction = WEST
        # If the column index has decreased, the robot has moved west
        elif prev_pos[1] > current_pos[1]:
            if (backwards == False):
                self.direction = WEST
            else:
                self.direction = EAST
        # If row index decreases, the robot has moved north
        else:
            if(backwards == False):
                self.direction = NORTH
            else:
                self.direction = SOUTH


    def __astar(self, start, goal, backwards = False):
        """Implementation of the a* algorithm for finding the shortest path in a 2d grid maze

        Args:
            start (list): Coordinates of the starting cell
            goal (list): Coordinates of the goal cell

        Returns:
            list of list: A list of coordinates specifying the path from start to goal

        Raises:
            ValueError: If no path is found between the given start and goal
        """
        goal = self.graph[goal[0]][goal[1]]
        # set of nodes already evaluated
        closedSet = set()
        # set of discovered nodes that have not been evaluated yet
        openSet = set()
        current = self.graph[start[0]][start[1]]
        current.G = 0
        # add start node to openSet
        openSet.add(current)
        prev = None
        # While there are still nodes to be evaluated
        while (openSet):
            # Get the node with the minimum sum of cost from start to that position and from that position to the goal
            current = min(openSet, key=lambda o: o.G + o.H)
            # If the node has a previous node (not start), set the new direction of robot
            if prev:
                if(backwards == False):
                    self.__setDirection(prev.coord, current.coord)
                else:
                    self.__setDirection(prev.coord, current.coord, backwards=True)
            # if goal is reached trace back the path
            if (current == goal):
                path = []
                while (current.parent):
                    path.append(current.coord)
                    current = current.parent
                # Append the coordinate which does not have a parent (which is the start point)
                path.append(current.coord)
                # Return path backwards (from start to goal)
                return path[::-1]
            # If goal is not yet reached
            else:
                # Mark current node to be evaluated
                openSet.remove(current)
                closedSet.add(current)
                # Get indices of 3 by 3 area around the current coordinate which are free and explored
                for node in self.__getNeighbours(current):
                    # For each node returned,
                    # Do nothing if the node has already been evaluated
                    if node in closedSet:
                        continue
                    # If the node has yet to be evaluated
                    if node in openSet:
                        # calculate new G cost
                        new_g = current.G + self.__getCost(current.coord, node.coord)
                        # If previous cost from goal to the coordinate is larger than the new cost
                        if node.G > new_g:
                            # Update to new, lower cost
                            node.G = new_g
                            # Update node's parent to be the current node
                            node.parent = current
                    else:
                        # if neither in openSet nor in closedSet
                        # cost of moving between neighbours is 1
                        node.G = current.G + self.__getCost(current.coord, node.coord)
                        node.parent = current
                        # Add new node to be evaluated
                        openSet.add(node)
            prev = copy.deepcopy(current)
        # exception is no path is found
        raise ValueError('No Path Found')

    def __initGraph(self, h_n):
        """To create a grid of graph nodes from the exploration map

        Args:
            h_n (Numpy array): The heuristic cost of each node to goal node
        """
        self.graph = []
        for row in xrange(MAX_ROWS):
            self.graph.append([])
            for col in xrange(MAX_COLS):
                self.graph[row].append(Node(self.exploredMap[row][col], (row, col), h_n[row][col]))

    def getFastestPath(self, backwards = False):
        """To call the fastest path algorithm and handle the case if there is a way-point or not
        """
        path = []
        start = copy.copy(self.start)
        # If there is a waypoint
        if (self.waypoint):
            h_n = self.__getHeuristicCosts(self.waypoint)
            self.__initGraph(h_n)
            # Set waypoint as the goal first and find shortest path from start to goal
            if(backwards == False):
                fsp = self.__astar(start, self.waypoint)
            else:
                fsp = self.__astar(start, self.waypoint, backwards=True)
            # Set start to be way point
            start = copy.copy(self.waypoint)
            # Leaves out the last element as it will be the starting node for the next fastest path
            # (Otherwise the way-point will be added twice)
            path.extend(fsp[:-1]) # Up to but not including the last element
        h_n = self.__getHeuristicCosts(self.goal)
        self.__initGraph(h_n)
        # start will be waypoint at this point if there is a waypoint
        if(backwards == False):
            fsp = self.__astar(start, self.goal)
        else:
            fsp = self.__astar(start, self.goal, backwards=True)
        path.extend(fsp)
        self.path = path

        self.markMap()

    def markMap(self):
        """To mark the path on the exploration map
        """
        # Related to the GUI (Refer to the color coding in the javascript code for the GUI)
        for ind in self.path:
            self.exploredMap[tuple(ind)] = 6

    # Adds more movements to self.movement so that robot moves one space if possible
    def moveStep(self, backwards=False):
        """To take the fastest path, robot location and determine the next
        move for the robot to follow the path
        """
        movement = []
        if(backwards == False):
            move_command = FORWARD
        else:
            move_command = BACKWARDS
        # self.index is first initialised to 1 when the FastestPath class is first initialised
        # Moving one step will increase self.index by one, thereby making self.path[self.index]) go to the next space allong the path
        # If the next space in the path is not equal to robot's centre and hence, robot should move to the next space in the path
        if (self.robot.center.tolist() != self.path[self.index]):
            # diff will be an array where first element is vertical distance from robot's centre to next space in path
            # Second element is horizontal distance from robot's centre to next space in path
            diff = self.robot.center - np.asarray(self.path[self.index])
            # If difference between row index of robot's centre and the next space is -1, ot means that the robot should move south
            if (diff[0] == -1 and diff[1] == 0):  # Going south
                if self.robot.direction == NORTH:
                    # If robot is facing north, it needs to turn right twice and move forward once before it can move south by one space
                    movement.extend((RIGHT, RIGHT, move_command))
                elif self.robot.direction == EAST:
                    movement.extend((RIGHT, move_command))
                elif self.robot.direction == SOUTH:
                    movement.append(move_command)
                else:
                    movement.extend((LEFT, move_command))
            elif (diff[0] == 0 and diff[1] == 1):  # Going west
                if self.robot.direction == NORTH:
                    movement.extend((LEFT, move_command))
                elif self.robot.direction == EAST:
                    movement.extend((RIGHT, RIGHT, move_command))
                elif self.robot.direction == SOUTH:
                    movement.extend((RIGHT, move_command))
                else:
                    movement.append(move_command)
            elif (diff[0] == 0 and diff[1] == -1):  # Going east
                if self.robot.direction == NORTH:
                    movement.extend((RIGHT, move_command))
                elif self.robot.direction == EAST:
                    movement.append(move_command)
                elif self.robot.direction == SOUTH:
                    movement.extend((LEFT, move_command))
                else:
                    movement.extend((RIGHT, RIGHT, move_command))
            else:  # Going north
                if self.robot.direction == NORTH:
                    movement.append(move_command)
                elif self.robot.direction == EAST:
                    movement.extend((LEFT, move_command))
                elif self.robot.direction == SOUTH:
                    movement.extend((RIGHT, RIGHT, move_command))
                else:
                    movement.extend((RIGHT, move_command))
            for move in movement:
                self.robot.moveBot(move)
        # Add movements to self.movement
        self.movement.extend(movement)
        # if not (self.sim):
        #     if (self.robot.stepCounter + len(movement) > self.calibrateLim):
        #         calibrate = self.robot.can_calibrate()
        #         if (calibrate[0]):
        #             movement.append(calibrate[1])
        #             self.robot.stepCounter = 0
        # Increase self.index by one so that the next space in self.path can be evaluated
        self.index += 1