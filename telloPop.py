from sys import _current_frames
from turtle import forward, speed
from djitellopy import Tello
import time, cv2
from pynput import keyboard
from Tracker import Tracker


FRAME_HEIGHT = 720
FRAME_WIDTH = 960
FORWARD_SPEED = 50

STEP_ONE_TO_POP = 1
STEP_TWO_TO_POP = 2
BALLOON_TYPES = 2


RADIUS_TO_FOLLOW = 170
RADIUS_TO_GET_AWAY = 185

TAKEOFF_KEY = 't'

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


"""
    This file is used to pop balloons using the TelloCV class and Tracker class.
"""

class TelloCV(object):
    def __init__(self):
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
        try:
            keyname = str(keyname).strip('\'')
            if(keyname == TAKEOFF_KEY):
                self.drone.takeoff()
                self.takeOff = True

        except AttributeError:
            print('Error with keys')
    


    def init_controls(self):
        """Add keyboard listener"""
        self.key_listener = keyboard.Listener(on_press=self.on_press)
        self.key_listener.start()


    def init_PID(self):
        """
        We want to use a feedback control loop to direct the drone towards the recognized object from tracker.py
        to do this we will use a PID controller.   
        find an optimal Vx and Vy using a PID
        distance = 100 # setpoint, pixels
        Vx = 0      #manipulated variable
        Vy = 0      #manipulated variable 
        """
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


    """ 
    In this function we get the x, y , radius of the balloon
        By this, we calculate the Vx,Vy using the PID. These Vx, Vy is the velocity
        in which the drone should move:
        if the Vx > 0 we move in yaw velocity to the right,
        if the Vx < 0 we move in yaw velocity to the left. 
        if the Vy > 0 we move up, if the Vy < 0, we move down.
        Note that if the balloon was detected we get a radius bigger than 0,
        It means that the balloon was found and we should move forward with FORWARD_SPEED,
        Until the radius of the balloon is RADIUS_TO_GET_AWAY. After this we go backward 60cm.
        There are 2 steps to pop the balloon: 
        1) To move forward until RADIUS_TO_GET_AWAY
        2) To move backward

        Then we repeat this again, we check if the radius is equal to zero,
        if yes, and step 2 was done, it means that there is no balloon with this color (the balloon was popped), and we set our
        Tracker to the next color definded in the dict popBaloons.

        Also we always return the cv2 image to display it in a new window
        
    """
    def process_frame(self, frame):
        image = frame #convert frame to cv2 image and show
        distance = 0

        xoff, yoff,radius = self.colortracker.track(image) # we use track function from colortracker object

        image = self.colortracker.draw_arrows(image) # draw the arrows that shows where the drone should move to
        Vx,Vy=self.PID.send([xoff, yoff, distance]) # calculate the X,Y Velocity using PID

        #Show the radius of the drone in the cv2 image
        cv2.putText(image, f"Radius: {round(radius,2)}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)

        if self.takeOff: # if the drone is already in the air
            
            self.drone.get_acceleration_x()

            if radius == 0: # if no object was detected
                self.drone.send_rc_control(0,0,0,0) #the drone does nothing (waits)

                if self.popBaloons[self.popCounter][0] == STEP_TWO_TO_POP: #if the drone just finished to pop one of the baloons

                    if self.popCounter == BALLOON_TYPES-1: #if we get to the last index
                        self.drone.land() #land the drone


                    else:
                        self.popCounter += 1 # set our target to the next balloon
                        #Set the tracker for the next color balloon
                        self.colortracker = Tracker(FRAME_HEIGHT,FRAME_WIDTH,
                            self.popBaloons[self.popCounter][1], self.popBaloons[self.popCounter][2], self.popBaloons[self.popCounter][3])
            else:

                if Vx > 0:
                    self.yaw_speed = int(abs(Vx)) # rotate right

                if Vx < 0:
                    self.yaw_speed = - int(abs(Vx)) # rotate left

                if Vy > 0:
                    self.up_down_speed = int(abs(Vy)) # go up

                if Vy < 0:
                    self.up_down_speed = - int(abs(Vy)) # go down

                """ These lines make the drone pop the balloon (he goes forward and then backward)
                    Using the two steps mentioned above we know in which state the drone is
                """

                if radius < RADIUS_TO_FOLLOW: # if the drone is not close to the balloon
                    self.forward_backward = FORWARD_SPEED # set to him forward speed towards the balloon
                    self.popBaloons[self.popCounter][0] = STEP_ONE_TO_POP

                elif radius >= RADIUS_TO_GET_AWAY: # if the drone is too close to the balloon
                    self.drone.move_back(60)
                    if self.popBaloons[self.popCounter][0] == STEP_ONE_TO_POP:
                        self.popBaloons[self.popCounter][0] = STEP_TWO_TO_POP
                else:
                    self.forward_backward = 0  

            if(self.drone.send_rc_control):
                # rc control is a command which tells the drone in which velocities he needs to move
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

        cv2.imshow("Image", current_frame) # show the last frame in a new window


if __name__ == "__main__":
    main()


