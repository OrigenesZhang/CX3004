import json
import numpy as np
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL, NORTH, SOUTH, WEST, EAST, FORWARD, FORWARDFAST, RIGHT, LEFT, MAX_COLS, MAX_ROWS


# Define options.port using define method from tornado.options
define("port", default=8888, help="run on the given port", type=int)

# Keep a record of all clients connected to the socket
clients = dict()

currentMap = np.zeros([20, 15])
map_name = 'map.txt'

area = 0
step = 0.1


class FuncThread(threading.Thread):

    """Class to create and run functions on different threads
    """

    def __init__(self, target, *args):
        """Construction to initialize the thread

        Args:
            target (function): Function to be run on new threads
            *args: arguments to be passed to the function
        """
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        """Overrides run function to run the function with given arguments
        """
        self._target(*self._args)


class IndexHandler(web.RequestHandler):

    """To display the front-end interface
    """

    @web.asynchronous
    def get(self):
        self.render("index.html")


class WebSocketHandler(websocket.WebSocketHandler):

    """Handles web-socket requests from the front-end to receive/send messages

    Attributes:
        id (string): id string from GET request
    """

    def open(self):
        """Open a web socket for communication
        """
        # get_argument is a unique method of tornado
        # This method searches both the query and body arguments.
        # Returns the value of the argument with the given name.
        self.id = self.get_argument("Id")
        # Set the no-delay flag for this stream.
        # By default, small messages may be delayed and/or combined to minimize
        # the number of packets sent.  This can sometimes cause 200-500ms delays
        # due to the interaction between Nagle's algorithm and TCP delayed
        # ACKs.  To reduce this delay (at the expense of possibly increasing
        # bandwidth usage), call ``self.set_nodelay(True)`` once the websocket
        # connection is established.
        self.stream.set_nodelay(True)
        # Note down the newly connected client's ID and WebSocketHandler object
        clients[self.id] = {"id": self.id, "object": self}
        print("WebSocket opened")

    def on_message(self, message):
        """Displays any message received

        Args:
            message (string): Message received from front-end
        """
        print("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        """Run when the web socket is closed
        """
        print("WebSocket closed")
        if self.id in clients:
            del clients[self.id]


class StartHandler(web.RequestHandler):

    """Handles the start of exploration for the maze
    """

    @web.asynchronous
    def get(self):
        self.write("Starting...")
        # Time step simulates the velocity of the robot in the simulator
        self.step = self.get_argument("step")
        # Time limit in seconds (Robot will stop exploration and go back to start if goal is not reached but time limit is up)
        self.limit = self.get_argument("limit")
        # Coverage of map that should be explored in %
        # Once coverage is reached, robot will stop exploration and go back to start if goal is not reached
        self.coverage = self.get_argument("coverage")
        global step
        step = float(self.step)
        startExploration(self.limit, self.coverage)
        self.flush()


class ResetHandler(web.RequestHandler):

    """Handles the reset of the current map
    """

    @web.asynchronous
    def get(self):
        self.write("Reset...")
        global exp
        # in order to qualify for leaderboard A, exploration should stop within 6 minutes
        # Here, time limit is set as 5
        exp = Exploration(map_name, 5)
        print (exp.currentMap)
        # Call update function to reset map in front end
        update(np.zeros([20, 15]), exp.exploredArea, exp.robot.center, exp.robot.head,
               START, GOAL, 0)


class FSPHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        self.x = self.get_argument("x")
        self.y = self.get_argument("y")
        self.write("Starting...")
        # [self.x, self.y] is the coordinates of the waypoint
        startFastestPath([self.x, self.y])
        self.flush()


class LoadMapHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        global map_name
        self.name = self.get_argument("name")
        map_name = self.name


def startExploration(limit, coverage):
    """To start the exploration of the maze
    """
    global exp, t_s
    exp = Exploration(map_name, 5)
    # Get current timestamp
    t_s = time.time()
    t2 = FuncThread(exploration, exp, limit, coverage)
    t2.start()
    # .join() method join a thread to the main thread
    # when join() is used for a particular thread the main thread will stop executing
    # until the execution of joined thread is complete
    # t2.join()


def exploration(exp, limit, coverage):
    """To explore the map and update the front-end after each move

    Args:
        exp (Exploration): New instance of the exploration class
    """
    global currentMap, area
    path = []
    time_limit = float(limit)
    elapsedTime = 0
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL, 0)
    logger('Exploration Started !')
    current = exp.moveStep() # Current will be for example (['w', 'w', 'w'], True)
    currentMap = exp.currentMap
    area = exp.exploredArea
    path.append(tuple(exp.robot.center))
    visited = dict()
    steps = 0
    numCycle = 1
    # current[0] is the set of moves which the robot should follow next
    # current[1] is whether or not 100% of the arena has been explored
    # While not less than 100% of the arena has been explored, time elapsed is below time limt and explored area percentage is less than coverage specified
    while (not current[1] and elapsedTime <= time_limit and exp.exploredArea < int(coverage)):
        # Update time elapsed
        elapsedTime = round(time.time()-t_s, 2)
        # Update front end
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
               elapsedTime)

        if (numCycle > 1 and steps > 4 and exp.exploredArea < 100):
            neighbour = exp.getExploredNeighbour()
            if (neighbour):
                neighbour = np.asarray(neighbour)
                fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                    exp.robot.direction, None)
                # Move the robot and update front end
                fastestPath(fsp, neighbour, exp.exploredArea, None)
                # Set robot's new center to be that neighbour after moving
                exp.robot.center = neighbour
                # Update coordinates of robot's head
                exp.robot.head = fsp.robot.head
                # Update the robot's direction after moving
                exp.robot.direction = fsp.robot.direction
                # Update map based on sensor values
                exp.robot.getSensors()
                if (exp.robot.direction == NORTH):
                    if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                        exp.robot.moveBot(RIGHT)
                        exp.robot.getSensors()
                    elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                        exp.robot.moveBot(LEFT)
                        exp.robot.getSensors()
                elif (exp.robot.direction == SOUTH):
                    if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                        exp.robot.moveBot(LEFT)
                        exp.robot.getSensors()
                    elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                        exp.robot.moveBot(RIGHT)
                        exp.robot.getSensors()
                elif (exp.robot.direction == EAST):
                    if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                        exp.robot.moveBot(RIGHT)
                        exp.robot.getSensors()
                    elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                        exp.robot.moveBot(LEFT)
                        exp.robot.getSensors()
                else:
                    if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                        exp.robot.moveBot(LEFT)
                        exp.robot.getSensors()
                    elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                        exp.robot.moveBot(RIGHT)
                        exp.robot.getSensors()
            else:
                # When there are no such neighbour and robot is stuck, break out of while loop and stop exploration
                break
        else:
            r, c = exp.robot.center
            current = exp.moveStep()
            currentMap = exp.currentMap
            area = exp.exploredArea
            steps += 1
            currentPos = tuple(exp.robot.center)
            path.append(currentPos)
            if(r == 18 and c == 8):
                print(path)
            if (currentPos in visited):
                # If you have visited the coordinates before, increase the number of times which you have visited it
                visited[currentPos] += 1
                # If you have visited the coordinate more than three times
                if ((area > 15 and visited[currentPos] > 1) or (visited[currentPos] > 3)) and area < 99 and len(path) > 9 and (path[-1] == currentPos or path[-2] == currentPos or path[-3] == currentPos or path[-4] == currentPos ):
                    # Get closest neighbour that has been explored
                    neighbour = exp.getCloseExploredNeighbour()
                    neighbour2 = exp.getExploredNeighbour()
                    if(neighbour != None or neighbour2 != None):
                        if(neighbour == None and neighbour2 != None):
                            neighbour = neighbour2
                        elif(neighbour != None and neighbour2 != None):
                            cost1 = abs(neighbour[0] - currentPos[0]) + abs(neighbour[1] - currentPos[1])
                            cost2 = abs(neighbour2[0] - currentPos[0]) + abs(neighbour2[1] - currentPos[1])
                            if(cost2 < cost1):
                                neighbour = neighbour2
                    # If such a neighbour exists
                    if (neighbour):
                        neighbour = np.asarray(neighbour)
                        # Find the shortest path from the robot's current position to the neighbour
                        fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                            exp.robot.direction, None)
                        fastestPath(fsp, neighbour, exp.exploredArea, None)
                        exp.robot.center = neighbour
                        exp.robot.head = fsp.robot.head
                        exp.robot.direction = fsp.robot.direction
                        exp.robot.getSensors()
                        if (exp.robot.direction == NORTH):
                            if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                            elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                        elif (exp.robot.direction == SOUTH):
                            if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                            elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                        elif (exp.robot.direction == EAST):
                            if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                            elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                        else:
                            if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                            elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                            
                    # When there are no such neighbour and robot is stuck, break out of while loop and stop exploration
                    else:
                        break
            else:
                # If you have not visited the coordinate, indicate that you have visited it before
                visited[currentPos] = 1
            if (np.array_equal(exp.robot.center, START)):
                numCycle += 1
                if (numCycle > 1 and steps > 4 and exp.exploredArea < 100):
                    neighbour = exp.getExploredNeighbour()
                    if (neighbour):
                        neighbour = np.asarray(neighbour)
                        fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                            exp.robot.direction, None)
                        # Move the robot and update front end
                        fastestPath(fsp, neighbour, exp.exploredArea, None)
                        # Set robot's new center to be that neighbour after moving
                        exp.robot.center = neighbour
                        # Update coordinates of robot's head
                        exp.robot.head = fsp.robot.head
                        # Update the robot's direction after moving
                        exp.robot.direction = fsp.robot.direction
                        # Update map based on sensor values
                        exp.robot.getSensors()
                        if (exp.robot.direction == NORTH):
                            if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                            elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                        elif (exp.robot.direction == SOUTH):
                            if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                            elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                        elif (exp.robot.direction == EAST):
                            if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                            elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                        else:
                            if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                                exp.robot.moveBot(LEFT)
                                exp.robot.getSensors()
                            elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                                exp.robot.moveBot(RIGHT)
                                exp.robot.getSensors()
                    else:
                        # When there are no such neighbour and robot is stuck, break out of while loop and stop exploration
                        break
                else:
                    break
        # Simulate the velocity of the robot according to the specified time step by pausing execution using time.sleep method
        time.sleep(float(step))
    # Update front end
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime)
    logger('Exploration Done !')
    logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
    logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
    ###############Is exp.robot.center really the goal?##########################3
    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction, None)
    logger('Fastest Path Started !')
    fastestPath(fsp, START, exp.exploredArea, None)


def startFastestPath(waypoint):
    """To start the fastest path of the maze"""
    global fsp
    global t_s
    waypoint = map(int, waypoint)
    fsp = FastestPath(currentMap, START, GOAL, NORTH, waypoint)
    t_s = time.time()
    logger('Fastest Path Started !')
    t3 = FuncThread(fastestPath, fsp, GOAL, area, waypoint)
    t3.start()
    # t3.join() this causes the thread to close after exploration and websocket closes


def markMap(curMap, waypoint):
    if waypoint:
        # 7 is a color code used by front end
        curMap[tuple(waypoint)] = 7
    return curMap


# Function to move the robot along fastest path while updating the front end
def fastestPath(fsp, goal, area, waypoint):
    fsp.getFastestPath()
    logger(json.dumps(fsp.path))
    while (fsp.robot.center.tolist() != goal.tolist()):
        fsp.moveStep()
        elapsedTime = round(time.time()-t_s, 2)
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, elapsedTime)
        time.sleep(step)
    logger('Fastest Path Done !')


def update(current_map, exploredArea, center, head, start, goal, elapsedTime):
    """To send messages to update the front-end

    Args:
        current_map (Numpy array): Current state of the exploration map
        exploredArea (int): Number of cells that have been explored
        center (list): Location of center of the robot
        head (list): Location of head of the robot
        start (list): Location of the starting point for the robot
        goal (list): Location of the finishing point for the robot
        elapsedTime (float): The time that has elapsed since exploration started
    """
    # Update every front end client connected
    for key in clients:
        message = dict()
        message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3 # Color code based on front end
        tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4 # Color code based on front end
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        message['center'] = json.dumps(center.astype(int).tolist())
        message['head'] = json.dumps(head.astype(int).tolist())
        message['time'] = '%.2f' % (elapsedTime)
        clients[key]['object'].write_message(json.dumps(message))


def logger(message):
    for key in clients:
        log = {'log': message}
        clients[key]['object'].write_message(json.dumps(log))


settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True
)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/reset', ResetHandler),
    (r'/fsp', FSPHandler),
    (r'/lm', LoadMapHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)

if __name__ == '__main__':
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()
