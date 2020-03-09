import socket
import random
import string
import threading
import time
from threading import Thread


def randomString(stringLength=10):
    """Generate a random string of fixed length """
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(stringLength))


class RPi(threading.Thread):
    def __init__(self):
        print "starting rpi communication"
        threading.Thread.__init__(self)

        self.ip = "192.168.6.6"  # Connecting to IP address of MDPGrp26
        self.port = 1335

        # Create a TCP/IP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print "sent connection request"

    def receive_send(self):
        while True:
            data = self.client_socket.recv(1024)
            if data:
                print ('Received %s from RPi' % data)
                get_msg = "h"+data
                self.client_socket.send(get_msg)
    def keep_main(self):
        while True:
            time.sleep(0.5)


def func1():
    print "Starting communication with RPi"
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.receive_send)
    rt.daemon = True
    rt.start()
    client_rpi.keep_main()


if __name__ == '__main__':
    Thread(target=func1).start()