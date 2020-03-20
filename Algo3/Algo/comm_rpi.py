import socket
import json
import numpy as np
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading
from threading import Thread

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL, NORTH, SOUTH, WEST, EAST, LEFT, RIGHT, FORWARD, FORWARDFAST, BACKWARDS, BACKWARDSFAST, ALIGNRIGHT, ALIGNFRONT, MAX_ROWS, MAX_COLS, ROTATE180

# Global Variables
define("port", default=8888, help="run on the given port 8888", type=int)
clients = dict()
currentMap = np.zeros([MAX_ROWS, MAX_COLS])

mdfCounter = 3 # For map descriptor

log_file = open('log.txt', 'w')

area = 0 # % of area that has been explored
exp = '' # Global variable to store Exploration class
fsp = '' # Global variable to store Fastest Path class
visited = dict() # Stores how many times each position on the arena has been visited
waypoint = None # Stores the coordinates of the way point
steps = 0 # Records down the number of steps which the robot has taken
numCycle = 1 # Records down the number of times which the robot has ran one round around the arena
t_s = 0 # Records down timestamps
direction = 1
path = []

map_name = 'map.txt'

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
        self.id = self.get_argument("Id")
        self.stream.set_nodelay(True)
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
        self.step = self.get_argument("step")
        self.limit = self.get_argument("limit")
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
        exp = Exploration(map_name, 5)
        update(np.zeros([MAX_ROWS, MAX_COLS]), exp.exploredArea, exp.robot.center, exp.robot.head,
               START, GOAL, 0)


class FSPHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        self.x = self.get_argument("x")
        self.y = self.get_argument("y")
        self.write("Starting...")
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
    t_s = time.time()
    t2 = FuncThread(exploration, exp, limit, coverage)
    t2.start()
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
        current = exp.moveStep()
        currentMap = exp.currentMap
        area = exp.exploredArea
        steps += 1
        currentPos = tuple(exp.robot.center)
        path.append(currentPos)

        if (currentPos in visited and len(path) > 9 and (path[-3] == currentPos or path[-4] == currentPos or path[-5] == currentPos or path[-6] == currentPos)):
            # If you have visited the coordinates before, increase the number of times which you have visited it
            print 'visited this location!'
            visited[currentPos] += 1
            # If you have visited the coordinate more than three times
            if (area > 15 and visited[currentPos] > 1) or (visited[currentPos] > 3):
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
                    print neighbour
                    neighbour = np.asarray(neighbour)
                    # Find the shortest path from the robot's current position to the neighbour
                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                      exp.robot.direction, None)
                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                    exp.robot.center = neighbour
                    exp.robot.head = fsp.robot.head
                    exp.robot.direction = fsp.robot.direction

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

            if numCycle>1 and exp.exploredArea>55:
                break


            elif (numCycle > 1 and steps > 4 and exp.exploredArea < 55):


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
                else:
                    # When there are no such neighbour and robot is stuck, break out of while loop and stop exploration
                    break

            

        # Simulate the velocity of the robot according to the specified time step by pausing execution using time.sleep method
        time.sleep(float(step))
    # Update front end
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime)
    logger('Exploration Done !')
    logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
   # logger("Map Descriptor 3  -->  "+str(exp.robot.descriptor_3()))
    ###############Is exp.robot.center really the goal?##########################3
    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction, None)
    logger('Fastest Path Started !')
    fastestPath(fsp, START, exp.exploredArea, None)


def startFastestPath(waypoint):
    """To start the fastest path of the maze
    """
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
        curMap[tuple(waypoint)] = 7
    return curMap

def fastestPath(fsp, goal, area, waypoint, backwards=False):
    if(backwards == False):
        fsp.getFastestPath()
    else:
        fsp.getFastestPath(backwards=True)
    logger(json.dumps(fsp.path))
    while (fsp.robot.center.tolist() != goal.tolist()):
        if(backwards == False):
            fsp.moveStep()
        else:
            fsp.moveStep(backwards=True)
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, 0)
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
    for key in clients:
        message = dict()
        message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3
        tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        message['center'] = json.dumps(center.astype(int).tolist())
        message['head'] = json.dumps(head.astype(int).tolist())
        message['time'] = '%.2f' % (elapsedTime)
        clients[key]['object'].write_message(json.dumps(message))

def logger(message):
    for key in clients:
        log = {'log': message}
        clients[key]['object'].write_message(json.dumps(log))


# =================communication with Rpi===================
def output_formatter(msg, movement):
    # not used
    if not isinstance(movement, list):
        movement = movement.tolist()
    movement = map(str, movement) #Returns a list of the results after applying the given function to each item of a given iterable (list, tuple etc.)
    return msg+'|'+'|'.join(movement)


def android_message_formatter(msg, array):
    if not isinstance(array, list):
        array = array.tolist()
    msg = "a" + msg + '|' + '|'.join(map(str, array)) + '!'
    return msg.encode('utf-8')

def android_send_fastest(msg, array):
    #array=array.join()
    command = ''.join(map(str,array))
    m = "a"+msg+"|"+ command
    return m.encode('utf-8')


def arduino_message_formatter(movement, getSensor=True, fastest = False):

    if not isinstance(movement, list):
        movement = movement.tolist()
    string = "".join(map(str, movement))
    res = ""
    res1=""

    if len(string) > 0:
        count = 1
        #Add in first character
        res += string[0]
        #Iterate through loop, skipping last one
        for i in range(len(string)-1):
            if(string[i] == string[i+1]):
                count+=1
                if count == 9:
                    res += str(count)
                    res += string[i+1]
                    count = 0
            else:
                res += str(count)
                res += string[i+1]
                count = 1
        if(count != 0):
          res += str(count)
        else:
          res = res[:-1]

        # for i in range(0, len(res), 2):
        #
        #     if (res[i]==RIGHT and res[i+1] == "2") or (res[i]==LEFT and res[i+1] == "2"):
        #         res1 += ROTATE180
        #     elif res[i]==FORWARD:
        #         if res[i+1]=="0":
        #             pass
        #         else:
        #             res1 += FORWARD + res[i+1]
        #     elif (res[i+1]=="1"):
        #         res1 += res[i]
        #     res1 += '\n'
        i = 0
        while i < len(res):
            if i < len(res) - 1 and ord('0') <= ord(res[i + 1]) <= ord('9'):
                if res[i + 1] == '0':
                    i += 2
                    continue
                if fastest and res[i] == FORWARD:
                    tmp = int(res[i+1])
                    while tmp > 3:
                        res1 += '3'
                        tmp -= 3
                    res1 += str(tmp)

                else:
                    tmp = int(res[i + 1])
                    while tmp > 0:
                        #res1 += res[i] + '\n'
                        res1 += res[i]
                        tmp -= 1
                i += 2
            else:
                #res1 += res[i] + '\n'
                res1 += res[i]
                i += 1
    if not getSensor:
        #msg = "h" + res1 + '\n'
        msg = "h" + res1
    else:
        #msg = "h" + res1 + 's\n'
        msg = "h" + res1 + "s"
    return msg.encode('utf-8')


def image_string_formatter(lst,direction):
    str1="I"
    for coordinate in lst:
        str1+=str(coordinate)+"|"
    str1+=direction
    print("=========> Image String is: ",str1)
    return str1

#not using
def fastestPath_with_align(list1):
    if list1[0]==FORWARD:
        for i in range(len(list1)):
            if list1[i]==LEFT:
                list_new=list1[:i]+[RIGHT,ALIGNFRONT,LEFT]+list1[i:]
                return list_new
    elif list1[0]==LEFT:
        for i in range(len(list1)):
            if list1[i]==RIGHT:
                list_new=list1[:i]+[LEFT,ALIGNFRONT,RIGHT]+list1[i:]
                return list_new
    return list1
    

class RPi(threading.Thread):
    def __init__(self):
        print("starting rpi communication")
        threading.Thread.__init__(self)

        self.ip = "192.168.6.6"  # Connecting to IP address of MDPGrp26
        self.port = 1335

        # Create a TCP/IP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print("sent connection request")

    def send_arduino_message(self,string):
        #str_new= self.align_for_farther(exp,string)
        #str_new= string
        self.client_socket.send(string)
        time.sleep(0.1)
        print ('Sent %s to RPi' % (string.decode('utf-8')))
        log_file.write("Sent %s to Arduino" % string)
        return (string)


    # Receive and send data to RPi data

    def receive_send(self):
        explorationDone= False
        time_t = time.time()
        elapsedTime = 0
        continueExplore = True
        fastestPathStart = False
        fastestPathStartCommand = None
        returnFastestPath = False
        returnFastestPathCommand = False
        manual_first = True
        while True:
            currentPos = None
            data = self.client_socket.recv(2048) # Parameter is the maximum size of data to be received from client
            data = data.decode('utf-8')
            log_file.write(data+'\n')
            log_file.flush()
            if (data):
                print ('Received %s from RPi\n' % (data))
                data = str(data)
                data = data.rstrip()
                split_data = data.split("|") # To be updated
                global exp, t_s, area, steps, numCycle, currentMap, fsp, path, visited
                # Initialise exploration
                if (split_data[0] == 'EXPLORE'):
                    t_s = time.time()
                    exp = Exploration(sim=False)
                    currentPos = exp.robot.center
                    path = []
                    area = 0
                    steps = 0
                    numCycle = 1
                    visited = dict()
                    visited[tuple(currentPos)] = 1
                    path.append(tuple(exp.robot.center))
                    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                           START, GOAL, 0)
                    arduino_msg = arduino_message_formatter([])
                    self.send_arduino_message(arduino_msg)

                    # arduino_msg = arduino_message_formatter(["N"], getSensor=False)
                    # self.send_arduino_message(arduino_msg,exp)  #Sent AN to Arduino ( pass AN to Rpi, send N to Arduino)


                elif (split_data[0] == 'obs:'):
                    # print ('Time 0: %s s' % (time.time() - time_t))
                    # Get sensor values
                    """ sensor_val[0] is the value from the front left sensor
        sensor_val[1] is the value from the front center sensor
        sensor_val[2] is the value from the front right sensor
        sensor_val[3] is the value from the right top sensor
        sensor_val[4] is the value from the right bottom sensor
        sensor_val[5] is the value from the left middle sensor """
                    sensors = map(int, split_data[1:])
                    currentPos = tuple(exp.robot.center)

                    #truth_value, coordinate_list, direction = self.image_recog(exp,exp.currentMap)
                    # if truth_value:
                    #     r,c= currentPos
                    #     image_str= image_string_formatter(coordinate_list, direction)
                    #     self.client_socket.send(image_str)
                    #     time.sleep(1)

                    if(fastestPathStart == True):
                        fastestPathStart = False
                        returnFastestPath = True
                        if (fastestPathStartCommand == RIGHT):
                            returnFastestPathCommand = LEFT
                        else:
                            returnFastestPathCommand = RIGHT
                        move = [fastestPathStartCommand]
                        exp.robot.getSensors(sensors)
                        exp.robot.moveBot(fastestPathStartCommand)
                        arduino_msg = arduino_message_formatter(move)
                        android_msg = android_message_formatter('EXPLORE',[str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                        print ('desc 3 = ' + str(exp.robot.descriptor_3()))
                    elif(returnFastestPath == True):
                        returnFastestPath = False
                        move = [returnFastestPathCommand]
                        exp.robot.getSensors(sensors)
                        exp.robot.moveBot(returnFastestPathCommand)
                        arduino_msg = arduino_message_formatter(move)
                        android_msg = android_message_formatter('EXPLORE',[str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                        print ('desc 3 = ' + str(exp.robot.descriptor_3()))
                    else:
                        # If not 100% coverage or even if 100, but not start
                        if ((exp.exploredArea <= 99.67 or np.array_equal(exp.robot.center, START)) and continueExplore):
                        #if (exp.exploredArea <= 99.67 and continueExplore):
                            current = exp.moveStep(sensors) # Get next movements and whether or not 100% covergae is reached
                            currentMap = exp.currentMap
                            area = exp.exploredArea
                            path.append(tuple(exp.robot.center))
                            elapsedTime = round(time.time()-t_s, 2)
                            nextPos = tuple(exp.robot.center)
                            time_t = time.time()
                            move = current[0]
                            update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                                START, GOAL, elapsedTime)
                            steps += 1  
                            # If the robot goes back to the start after explorin more than 50% of the arena
                            #if (np.array_equal(exp.robot.center, START) and exp.exploredArea > 50 and continueExplore):
                            # I force a full exploration here
                            if (np.array_equal(exp.robot.center, START) and exp.exploredArea >= 95 and continueExplore):
                                # Increase cycle count by 1
                                numCycle += 1
                                continueExplore = False
                                pass

                            if (numCycle > 1 and steps > 4 and continueExplore):
                                # Get valid neighbours of unexplored spaces that is the closest to the robot's current position
                                neighbour = exp.getExploredNeighbour()
                                if (neighbour):
                                    neighbour = np.asarray(neighbour)
                                    # If the neighbour exists, get the fastest path from robot's current position to that neighbour
                                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                        exp.robot.direction, None, sim=False)
                                    # Move the robot vrtually
                                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                                    # Shorten multiple forward commands and append to move
                                    move = fsp.movement
                                    exp.robot.phase = 2
                                    exp.robot.center = neighbour
                                    exp.robot.head = fsp.robot.head
                                    exp.robot.direction = fsp.robot.direction
                                    currentMap = exp.currentMap
                                    if (exp.robot.direction == NORTH):
                                        if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                                            fastestPathStart = True
                                            fastestPathStartCommand = RIGHT
                                        elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                                            fastestPathStart = True
                                            fastestPathStartCommand = LEFT
                                    elif (exp.robot.direction == SOUTH):
                                        if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                                            fastestPathStart = True
                                            fastestPathStartCommand = LEFT
                                        elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                                            fastestPathStart = True
                                            fastestPathStartCommand = RIGHT
                                    elif (exp.robot.direction == EAST):
                                        if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                                            fastestPathStart = True
                                            fastestPathStartCommand = RIGHT
                                        elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                                            fastestPathStart = True
                                            fastestPathStartCommand = LEFT
                                    else:
                                        if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                                            fastestPathStart = True
                                            fastestPathStartCommand = LEFT
                                        elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                                            fastestPathStart = True
                                            fastestPathStartCommand = RIGHT
                                else:
                                    continueExplore = False
                                # If the current position has been visited
                                if (nextPos in visited):
                                    # Increase the visited count of the current position by one
                                    visited[nextPos] += 1
                                    # if (len(path) > 9 and ( path[-3] == nextPos or path[-4] == nextPos )):
                                    if (len(path) > 9 and ( exp.robot.movement[-9:-1] == [FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, RIGHT] )):
                                        # If the current position has been visited for more than three times
                                        if ((area > 15 and visited[nextPos] > 1) or (visited[nextPos] > 3)) and area < 99:
                                            # Get closest neighbour that has been explored
                                            neighbour = exp.getCloseExploredNeighbour()
                                            neighbour2 = exp.getExploredNeighbour()
                                            if(neighbour != None or neighbour2 != None):
                                                if(neighbour == None and neighbour2 != None):
                                                    neighbour = neighbour2
                                                elif(neighbour != None and neighbour2 != None):
                                                    cost1 = abs(neighbour[0] - nextPos[0]) + abs(neighbour[1] - nextPos[1])
                                                    cost2 = abs(neighbour2[0] - nextPos[0]) + abs(neighbour2[1] - nextPos[1])
                                                    if(cost2 < cost1):
                                                        neighbour = neighbour2
                                            # If there is such a neighbour
                                            if (neighbour):
                                                neighbour = np.asarray(neighbour)
                                                # Get the shortest path to go to that neighbour
                                                fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                                exp.robot.direction, None, sim=False)
                                                # Move and update robot virtually
                                                fastestPath(fsp, neighbour, exp.exploredArea, None)
                                                move.extend(fsp.movement)
                                                # exp.robot.phase = 2
                                                exp.robot.center = neighbour
                                                exp.robot.head = fsp.robot.head
                                                exp.robot.direction = fsp.robot.direction
                                                currentMap = exp.currentMap
                                                path.append(tuple(exp.robot.center))
                                                if (exp.robot.direction == NORTH):
                                                    if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = RIGHT
                                                    elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = LEFT
                                                elif (exp.robot.direction == SOUTH):
                                                    if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = LEFT
                                                    elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = RIGHT
                                                elif (exp.robot.direction == EAST):
                                                    if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = RIGHT
                                                    elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = LEFT
                                                else:
                                                    if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = LEFT
                                                    elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                                                        fastestPathStart = True
                                                        fastestPathStartCommand = RIGHT
                                else:
                                    visited[nextPos] = 1
                                # If the robot goes back to the start after explorin more than 50% of the arena
                                if (np.array_equal(exp.robot.center, START) and exp.exploredArea < 100):
                                    # Increase cycle count by 1
                                    numCycle += 1

                
                                    # If the number of cycles is greater than one and robot has taken more than 4 steps
                                    if (numCycle > 1 and steps > 4 and continueExplore):
                                        # Get valid neighbours of unexplored spaces that is the closest to the robot's current position
                                        neighbour = exp.getExploredNeighbour()
                                        if (neighbour):
                                            neighbour = np.asarray(neighbour)
                                            # If the neighbour exists, get the fastest path from robot's current position to that neighbour
                                            fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                                exp.robot.direction, None, sim=False)
                                            # Move the robot vrtually
                                            fastestPath(fsp, neighbour, exp.exploredArea, None)
                                            # Shorten multiple forward commands and append to move
                                            move.extend(fsp.movement)
                                            exp.robot.phase = 2
                                            exp.robot.center = neighbour
                                            exp.robot.head = fsp.robot.head
                                            exp.robot.direction = fsp.robot.direction
                                            currentMap = exp.currentMap
                                            if (exp.robot.direction == NORTH):
                                                if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = RIGHT
                                                elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = LEFT
                                            elif (exp.robot.direction == SOUTH):
                                                if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = LEFT
                                                elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = RIGHT
                                            elif (exp.robot.direction == EAST):
                                                if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = RIGHT
                                                elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = LEFT
                                            else:
                                                if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = LEFT
                                                elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                                                    fastestPathStart = True
                                                    fastestPathStartCommand = RIGHT
                                        else:
                                            continueExplore = False
                                    else:
                                        continueExplore = False
                            # print 'Time 1: %s s' % (time.time() - time_t)
                            time_t = time.time()
                            # arduino_msg and android_msg is the message to be sent to Rpi
                            arduino_msg = arduino_message_formatter(move)
                            android_msg = android_message_formatter("EXPLORE",[str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction]) #delete descriptor 3
                            print ('desc 3 = ' + str(exp.robot.descriptor_3()))
                        # print 'Time 2: %s s' % (time.time() - time_t)
                        else:
                            # Inform front end that exploration is complete
                            update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                                START, GOAL, elapsedTime)
                            logger('Exploration Done !')
                            logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
                            logger("Map Descriptor 2  -->  " + str(exp.robot.descriptor_2()))
                            #logger("Map Descriptor 3  -->  "+str(exp.robot.descriptor_3()))
                            move = []
                            if(np.array_equal(exp.robot.center, START) == False):
                                # Initiate fastest path without any waypoint to get from robot's current position to the start point
                                #try: instead of fsp, still use right wall hugging to go back to start. but tell moveStep() not to update the map
                                # or just let it continue to explore until it reaches start?
                                fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction,
                                                None, sim=False)
                                logger('Fastest Path Started !')
                                fastestPath(fsp, START, exp.exploredArea, None)
                                move = fsp.movement
                                path = fsp.path

                                #move = fastestPath_with_align(move)

                                print("FastestP move: ", move)
                                print("FastestP path: ", path)

                                exp.robot.phase = 2
                                exp.robot.center = START
                                exp.robot.head = fsp.robot.head
                                exp.robot.direction = fsp.robot.direction
                                currentMap = exp.currentMap

                            time.sleep(1)

                            if (exp.robot.direction == WEST):
                                calibrate_move = [ROTATE180, ALIGNRIGHT]
                            else:#SOUTH
                                calibrate_move = [LEFT, ALIGNRIGHT]
                            exp.robot.direction = EAST

                            arduino_msg_done= arduino_message_formatter(move + calibrate_move, getSensor=False)
                            #arduino_calibrate_msg = arduino_message_formatter(calibrate_move, getSensor=False)
                            explorationDone=True
                            android_msg = android_message_formatter('DONE', [str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", EAST])
                            print ('desc 3 = ' + str(exp.robot.descriptor_3()))

                            time.sleep(0.1)

                    self.client_socket.send(android_msg)
                    time.sleep(0.1)
                    print ('Sent %s to RPi' % (android_msg.decode('utf-8')))
                    if (explorationDone):
                        self.send_arduino_message(arduino_msg_done)
                    else:
                        self.send_arduino_message(arduino_msg)
                    #print("Direction before starting fastest path:" , direction)
                    log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                    log_file.write('Sent %s to RPi\n\n' % (android_msg))
                    log_file.flush()

                # Start fastest path
                # elif (split_data[0]== "REACHED"):
                #     time.sleep(4)
                #     self.send_arduino_message(arduino_calibrate_msg,exp)
                    # Set waypoint
                elif (split_data[0] == "CALIBRATE"):
                    arduino_msg = arduino_message_formatter([ALIGNFRONT, LEFT, ALIGNFRONT, LEFT, ALIGNRIGHT], getSensor = False)
                    self.send_arduino_message(arduino_msg)


                elif (split_data[0] == 'WAYPOINT'):
                    global waypoint
                    waypoint = map(int, split_data[1:])
                    waypoint[0] = 19 - waypoint[0]
                    print(waypoint)


                elif (split_data[0] == "FASTEST"):
                    fsp = FastestPath(currentMap, START, GOAL, EAST, waypoint, sim=False, fastest=True)
                    #file1= np.ones([20, 15])
                    #sim=True
                    #fsp = FastestPath(file1, START, GOAL, direction, waypoint, sim)
                    currentPos = tuple(fsp.robot.center)
                    fastestPath(fsp, GOAL, 300, waypoint, backwards=False)
                    print("Direction after starting fastest path:" , direction)
                    # move = fsp.movement
                    move = fsp.movement
                    path = fsp.path

                   # move = fastestPath_with_align(move)

                    print("FastestP move: ", move)
                    print("FastestP path: ", path)

                    arduino_msg = arduino_message_formatter(move, getSensor=False, fastest = True)
#--------------------> changed sending move to android instead of path
                    android_msg = android_send_fastest('FASTEST', move)
                    self.client_socket.send(android_msg)
                    time.sleep(0.1)
                    print ('Sent %s to RPi' % (android_msg))
                    self.send_arduino_message(arduino_msg)
                    log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                    log_file.write('Sent %s to RPi\n\n' % (android_msg))
                    log_file.flush()
                    break
                # To move the robot manually
                elif (split_data[0] == 'MANUAL'):
                    manual_movement = split_data[1:]
                    if manual_first:
                        exp = Exploration(sim = False)
                        manual_first = False
                    for move in manual_movement:
                        exp.robot.moveBot(move)
                    arduino_msg = arduino_message_formatter(manual_movement)
                    self.send_arduino_message(arduino_msg)
                    android_msg = android_message_formatter("DONE", [str(exp.robot.descriptor_1()),str(exp.robot.descriptor_2()),"[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                    print ('desc 3 = ' + str(exp.robot.descriptor_3()))
                    self.client_socket.send(android_msg)

    def keep_main(self):
        while True:
            time.sleep(0.5)
            # edit out sleep to check if we can optimize run time
            # edit out fastestPath during exploration and check if it optimizes run time


settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/reset', ResetHandler),
    (r'/fsp', FSPHandler),
    (r'/lm', LoadMapHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})], **settings)


def func1():
    print("Starting communication with RPi")
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.receive_send)
    rt.daemon = True
    rt.start()
    client_rpi.keep_main()


def func2():
    print("Starting communication with front-end")
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()


if __name__ == '__main__':
    Thread(target=func1).start()
    # Thread(target=func2).start()
