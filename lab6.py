#ALex Anderson
import socket
from time import *
from pynput import keyboard
"""pynput: On Mac OSX, one of the following must be true:
* The process must run as root. OR
* Your application must be white listed under Enable access for assistive devices. Note that this might require that you package your application, since otherwise the entire Python installation must be white listed."""
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy

seeking_x = -1
seeking_y = -1
me_x = -1
me_y= -1

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = "192.168.1.106" 	# SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 4 # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = True

# You should fill this in with your states
class States(enum.Enum):
    LISTEN = enum.auto()

class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.LISTEN
        
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
        # Start video
        self.video.start()
        
        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection( (self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)
    
        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        if ENABLE_ROBOT_CONNECTION:
            with socketLock:
                self.sock.sendall("i /dev/ttyUSB0".encode())
                print("Sent command")
                result = self.sock.recv(128)
                print(result)
                if result.decode() != "i /dev/ttyUSB0":
                    self.RUNNING = False
        
        self.sensors = Sensing(self.sock)
        # Start getting data
        if ENABLE_ROBOT_CONNECTION:
            self.sensors.start()
        
        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
            
    def run(self):
        global seeking_x
        global seeking_y
        global me_x
        global me_y
        # BEGINNING OF THE CONTROL LOOP
        while(self.RUNNING):
            sleep(0.1)
            if self.STATE == States.LISTEN:
                pass
            # TODO: Work here
                    #IMPORTANT: MOVE THIS OUT OF THE IMAGE PROCESSING THREAD
            if (seeking_x != -1 and seeking_y != -1): #once we have a coordinate to move to
                    distance = ((seeking_x - me_x) ** 2 +(seeking_y - me_y) ** 2) ** 0.5 #stats something to find distance between bot and point
                    if distance > -1 and distance < 20:
                        break
                    positive = True
                    while positive:
                        with socketLock:
                            self.sock.sendall("a drive_straight(15)".encode())
                            self.sock.recv(128)
                        distance2 = ((seeking_x - me_x) ** 2 +(seeking_y - me_y) ** 2) ** 0.5
                        if distance2 > distance:
                            positive = False
                    #moving forward is no longer positive, so move backwards
                    positive = True
                    while positive:
                        with socketLock:
                            self.sock.sendall("a drive_straight(-15)".encode())
                            self.sock.recv(128)
                        distance2 = ((seeking_x - me_x) ** 2 +(seeking_y - me_y) ** 2) ** 0.5
                        if distance2 > distance:
                            positive = False
                    with socketLock:
                        self.sock.sendall("a spin_right(90)".encode())
                        self.sock.recv(128) #this might not work if it doesn't turn 45 degrees
        

        # END OF CONTROL LOOP
        
        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.video.RUNNING = False
        
        sleep(1)    # Wait for threads to wrap up
        
        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

        #self.sensors.join()
        #self.video.join()

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'q':
                # Stop listener
                self.RUNNING = False
                self.sensors.RUNNING = False
                self.video.RUNNING = False
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
            return False

# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        self.RUNNING = True
        self.sock = socket
    
    def run(self):
        while self.RUNNING:
            sleep(1)
            # This is where I would get a sensor update
            # Store it in this class
            # You can change the polling frequency to optimize performance, don't forget to use socketLock
            with socketLock:
                self.sock.sendall("a distance".encode())
                print(self.sock.recv(128))

        


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.cam = cv2.VideoCapture(0)
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.feedback = []
        self.thresholds = {'low_red':230,'high_red':255,'low_green':0,'high_green':255,'low_blue':0,'high_blue':255}

    def run(self):
        #url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
        #stream = urllib.request.urlopen(url)
        while(self.RUNNING):
            sleep(0.1)
            """bytes = b''
            while self.RUNNING:
                bytes += stream.read(8192)  #image size is about 40k bytes, so this loops about 5 times
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a>b:
                    bytes = bytes[b+2:]
                    continue
                if a!=-1 and b!=-1:
                    jpg = bytes[a:b+2]
                    #bytes= bytes[b+2:]
                    #print("found image", a, b, len(bytes))
                    break
            img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8),cv2.IMREAD_COLOR)"""


                    

                        
            retValue, img = self.cam.read()
            # Resize to half size so that image processing is faster
            img = cv2.resize(img, ((int)(len(img[0])/RESIZE_SCALE),(int)(len(img)/RESIZE_SCALE)))
            
            with imageLock:
                self.latestImg = copy.deepcopy(img) # Make a copy not a reference

            masked = self.doImgProc() #pass by reference for all non-primitve types in Python

            # after image processing you can update here to see the new version
            with imageLock:
                self.feedback = copy.deepcopy(masked)

    def click(self, event, x, y, flags, params):
        global seeking_x
        global seeking_y
        if event == cv2.EVENT_LBUTTONDOWN:
            #go to click fxn
            cv2.circle(self.latestImg,(x,y),10,(255,0,0),2)
            print("clicked!", x, y)
            seeking_x = x
            seeking_y = y
            #with socketLock:
            #    self.sock.sendall("a drive_straight(15)".encode())
            #    self.sock.recv(128)

            
    def setThresh(self, name, value):
        self.thresholds[name] = value
    
    def doImgProc(self):
        global me_x
        global me_y

        low = (self.thresholds['low_blue'], self.thresholds['low_green'], self.thresholds['low_red'])
        high = (self.thresholds['high_blue'], self.thresholds['high_green'], self.thresholds['high_red'])
        theMask = cv2.inRange(self.latestImg, low, high)
        
        # TODO: Work here
        analysis = cv2.connectedComponentsWithStats(theMask,
                                            4,
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis

        if len(stats) > 0: max = stats[1][4]
        index = 1
        for i in range(1, len(stats[1:])):
            if stats[i][4] > max:
                max = stats[i][4] 
                index = i

        me_x = stats[index][2]
        me_y = stats[index][3]
        
        # END TODO
        return cv2.bitwise_and(self.latestImg, self.latestImg, mask=theMask)

        

# END OF IMAGEPROC


if __name__ == "__main__":
    
    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)
    
    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)
    
    sm = StateMachine()
    sm.start()
    
    cv2.setMouseCallback("Create View", sm.video.click, sm.video)
    
    # Probably safer to do this on the main thread rather than in ImgProc init
    cv2.createTrackbar('low_red', 'sliders', sm.video.thresholds['low_red'], 255,
                      lambda x: sm.video.setThresh('low_red', x) )
    cv2.createTrackbar('high_red', 'sliders', sm.video.thresholds['high_red'], 255,
                     lambda x: sm.video.setThresh('high_red', x) )
    
    cv2.createTrackbar('low_green', 'sliders', sm.video.thresholds['low_green'], 255,
                      lambda x: sm.video.setThresh('low_green', x) )
    cv2.createTrackbar('high_green', 'sliders', sm.video.thresholds['high_green'], 255,
                     lambda x: sm.video.setThresh('high_green', x) )
    
    cv2.createTrackbar('low_blue', 'sliders', sm.video.thresholds['low_blue'], 255,
                      lambda x: sm.video.setThresh('low_blue', x) )
    cv2.createTrackbar('high_blue', 'sliders', sm.video.thresholds['high_blue'], 255,
                     lambda x: sm.video.setThresh('high_blue', x) )

    while len(sm.video.latestImg) == 0 or len(sm.video.feedback) == 0:
        sleep(1)

    while(sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View",sm.video.latestImg)
            cv2.imshow("sliders",sm.video.feedback)
        cv2.waitKey(5)

    cv2.destroyAllWindows()

    sleep(1)

    #sm.join()

