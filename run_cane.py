import time
import rospy
from gpiozero import DistanceSensor, PWMOutputDevice
from assistive_cane.msg import cane_command_msg
#import RPi.GPIO as GPIO
from gpiozero import Buzzer
from threading import Lock

cane_lock = Lock()
cane_command = cane_command_msg()
cane_command.vibration_forward = 0
cane_command.vibration_right = 0
cane_command.vibration_left = 0
"""
Code Implementation Overview

Day 1: TODO 1

Day 2: None

Day 3: None

Day 4: TODO 2, TODO 3

Day 5: Demo!
"""

class Ultrasonic:
    """ 
    A class used to represent the ultrasonic sensor

    Attributes
    ----------
    distance: float
        Distance of any obstacle infront of sensor
    ultra: DistanceSensor
        Represents an HC-SR04 ultrasonic distance sensor
    """
    def __init__(self):
        self.distance = 0.0
        """
        [Day 4] TODO 3: Set self.ultra to instance of DistanceSensor with arguments:
            - echo
            - trigger
            - max_distance
        """
        ####### Insert Code Here #######
        
        self.ultra = None
        
        ################################

    def update_ultrasonic(self):
        """
        Updates distance attribute of class with distance sensed by self.ultra

        Parameters
        ----------
        None

        Returns
        -------
        None 
        """
        if self.ultra != None:
            self.distance = self.ultra.distance

def cane_callback(cane_msg):
    """
    Prints cane_command_msg describing vibration commands 
    and accounts for mutex locking

    Parameters
    ----------
    cane_msg ROS message
        Message describing vibration commands
    """
    global cane_command
    print("Got Message!")
    print(cane_msg)
    with cane_lock:
        print("GOT LOCK")
        cane_command = cane_msg

if __name__ == '__main__':
    """
    [Day 1] TODO 1: Write code for the following
        - Initialize a ROS node called "smart_cane"
        - Create a subscriber with the following parameters
            - Topic: "/cane_command"
            - Message type: cane_command_msg
            - Callback: cane_callback()
    """
    ####### Insert Code Here #######
    
    # Initialize node
    
    # Create subscriber
    
    ################################

    """
    [Day 4] TODO 2: Integrate with Raspberry Pi
        - Select the correct PIN values for the following variables:
            - buzzer
            - forward_vibrator
            - right_vibrator
            - left_vibrator
            Make sure to use Buzzer() and PWMOutputDevice()
    """
    ####### Insert Code Here #######

    buzzer = None
    forward_vibrator = None
    right_vibrator = None
    left_vibrator = None

    # Remove the following line when done with above
    exit()

    ################################
    try:
        # Initialize ultrasonic sensor
        ultra = Ultrasonic()

        # Loop through code to run demo
        while True:
            ultra.update_ultrasonic()
            print(ultra.distance)
            print("Forward: ",forward_vibrator.value)
            print("Left: ",left_vibrator.value)
            print("Right: ",right_vibrator.value)
            if ultra.distance < 0.2:
                buzzer.on()
            else:
                buzzer.off() 
                with cane_lock:
                    print("Setting Value!")
                    print("Forward:",cane_command.vibration_forward)
                    forward_vibrator.value = cane_command.vibration_forward
                    left_vibrator.value = cane_command.vibration_left
                    right_vibrator.value = cane_command.vibration_right
            time.sleep(0.2)
    except KeyboardInterrupt:
        GPIO.cleanup()
