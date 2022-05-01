from sys import _current_frames
from turtle import forward, speed
from djitellopy import Tello
import time, cv2
from pynput import keyboard
from Tracker import Tracker
import threading
import numpy


FRAME_HEIGHT = 720
FRAME_WIDTH = 960
FORWARD_SPEED = 50

STEP_ONE_TO_POP = 1
STEP_TWO_TO_POP = 2
NUM_OF_BALLOON_TYPES = 2


RADIUS_TO_FOLLOW = 170
RADIUS_TO_GET_AWAY = 185

"""Colors of Baloon"""
greenLower = (30, 50, 50) 
greenUpper = (80, 255, 255)
greenCode = cv2.COLOR_RGB2HSV

redLower = (161, 155, 84)
redUpper = (179,255,255)
redCode = cv2.COLOR_BGR2HSV

blueLower = (90,150,0)
blueUpper = (135,255,255)
blueCode = cv2.COLOR_BGR2HSV




class TelloCV(object):
    def __init__(self):
        self.keydown = False
        self.speed = 0
        self.up_down_speed = 0
        self.yaw_speed = 0
        self.forward_backward = 0
        self.takeOff = False
        self.drone = Tello()
        self.init_drone()
        self.init_controls()
        self.init_PID()
        #The key is the index, the first element in the list is the progress level, the rest are color settings
        self.popBaloons = {0: [0, greenLower, greenUpper, greenCode], 1: [0, blueLower,blueUpper, blueCode], 2: [0, redLower,redUpper, greenCode]}

        self.popCounter = 0
        self.colortracker = Tracker(FRAME_HEIGHT,FRAME_WIDTH,
                               self.popBaloons[0][1], self.popBaloons[0][2], self.popBaloons[0][3])

    def init_drone(self):
        """Connect, uneable streaming"""
        self.drone.connect()
        self.drone.streamoff()
        self.drone.streamon()

    def on_press(self, keyname):
        """handler for keyboard listener"""
        if self.keydown:
            return
        try:
            self.keydown = True
            keyname = str(keyname).strip('\'')
            if(keyname == 't'):
                self.takeOff = True
                key_handler = self.controls[keyname]
                key_handler()
        except AttributeError:
            print('Error with keys')
    

    def on_release(self, keyname):
        """Reset on key up from keyboard listener"""
        self.keydown = False

    def init_controls(self):
        """Define keys and add listener"""
        self.controls = {
            't': lambda: self.drone.takeoff(),
        }

        self.key_listener = keyboard.Listener(on_press=self.on_press,
                                              on_release=self.on_release)
        self.key_listener.start()


#We want to use a feedback control loop to direct the drone towards the recognized object from tracker.py
    #to do this we will use a PID controller.   
        #find an optimal Vx and Vy using a PID
        # distance = 100 # setpoint, pixels
        # Vx = 0      #manipulated variable
        # Vy = 0      #manipulated variable 
    def init_PID(self):
        def proportional():
            Vx = 0
            Vy = 0
            prev_time = time.time()
            Ix = 0
            Iy = 0
            ex_prev = 0
            ey_prev = 0
            while True:
                #yield an x and y velocity from xoff, yoff, and distance
                xoff, yoff, distance = yield Vx, Vy

                #PID Calculations
                ex = xoff - distance
                ey = yoff - distance
                current_time = time.time()
                delta_t = current_time - prev_time
                
                #Control Equations, constants are adjusted as needed
                Px = 0.1*ex
                Py = 0.1*ey
                Ix = Ix + -0.001*ex*delta_t
                Iy = Iy + -0.001*ey*delta_t
                Dx = 0.01*(ex - ex_prev)/(delta_t)
                Dy = 0.01*(ey - ey_prev)/(delta_t)

                Vx = Px + Ix + Dx
                Vy = Py + Iy + Dy

                #update the stored data for the next iteration
                ex_prev = ex
                ey_prev = ey
                prev_time = current_time
        self.PID = proportional()
        self.PID.send(None)


    def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        #image = cv2.cvtColor(numpy.array(frame), cv2.COLOR_RGB2BGR)
        image = frame
        distance = 0

        xoff, yoff,radius = self.colortracker.track(image)

        image = self.colortracker.draw_arrows(image)
        Vx,Vy=self.PID.send([xoff, yoff, distance])

        cv2.putText(image, f"Radius: {round(radius,2)}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2) # Look up after the radius of the balloon

        if self.takeOff: # if the drone is already in the air

            if radius == 0: # if no object has been detected
                self.drone.send_rc_control(0,0,0,0) #the drone does nothing

                if self.popBaloons[self.popCounter][0] == STEP_TWO_TO_POP: #if the drone just finished to pop one of the baloons
                    self.popCounter += 1 # set to next balloon
                    self.colortracker = Tracker(FRAME_HEIGHT,FRAME_WIDTH,
                            self.popBaloons[self.popCounter][1], self.popBaloons[self.popCounter][2], self.popBaloons[self.popCounter][3]) # set the tracker for the next color balloon
            else:

                if Vx > 0:
                    self.yaw_speed = int(abs(Vx)) # rotate right

                if Vx < 0:
                    self.yaw_speed = - int(abs(Vx)) # rotate left

                if Vy > 0:
                    self.up_down_speed = int(abs(Vy)) # go up

                if Vy < 0:
                    self.up_down_speed = - int(abs(Vy)) # go down

                """ These lines make the drone "kiss" the balloon (he goes forwards and then quickly backwards)"""

                if radius < RADIUS_TO_FOLLOW: # if the drone is not close to the balloon
                    self.forward_backward = FORWARD_SPEED # set to him forward speed towards the balloon
                    self.popBaloons[self.popCounter][0] = STEP_ONE_TO_POP

                elif radius >= RADIUS_TO_GET_AWAY: # if the drone is too close to the balloon
                    self.drone.move_back(60)
                    if self.popBaloons[self.popCounter][0] == STEP_ONE_TO_POP:
                        self.popBaloons[self.popCounter][0] = STEP_TWO_TO_POP # now the drone 100% popped the balloons
                else:
                    self.forward_backward = 0  

            if(self.drone.send_rc_control):
                self.drone.send_rc_control(0,self.forward_backward,self.up_down_speed,self.yaw_speed) #move the drone

        return image


def main():

    telloTrack = TelloCV()
    myTello = telloTrack.drone #reference to tello object

    while True:

        frameRead = myTello.get_frame_read() #get frame by frame from tello

        height, width, _ = frameRead.frame.shape #get the frame config
        current_frame = frameRead.frame # set the frame config
        current_frame = cv2.resize(current_frame, (width,height))

        current_frame = telloTrack.process_frame(current_frame) # control the drone and give updated image with the detected balloon

        if cv2.waitKey(1) & 0xFF == ord('q') and telloTrack.takeOff: # if the 'q' pressed on the cv2 screen
            myTello.land()
            telloTrack.takeOff = False
            break

        cv2.imshow("Image", current_frame) # show the last frame


if __name__ == "__main__":
    main()


