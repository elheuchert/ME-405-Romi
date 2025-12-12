from pyb import Pin, Timer, ADC
## @brief Initializes Bump sensors
#
# This class manages bump sensors for romi, to detect any collisions.
# 
# The bump sensor we are using is made specifically for the Pololu Romi, and is placed on the front of the chassis board. The sensors should be connected to digital input pins, with their internal pull ups enabled. This means that the lines will be floating up and will be pulled down when a switch is pressed. 
class Bump_Sensor:
    ## @brief This initializes the Pins to input pins and enables internal pull-ups for one set of bump sensors (three switches).
    # @param P1 Pin Object
    # @param P2 Pin Object
    # @param P3 Pin Object
    def __init__ (self, p1,p2,p3):
        
        self.Pin_A=Pin(p1,mode=Pin.IN, pull=Pin.PULL_UP)
        self.Pin_B=Pi n(p2,mode=Pin.IN, pull=Pin.PULL_UP)
        self.Pin_C=Pin(p3,mode=Pin.IN, pull=Pin.PULL_UP)
        
    ## @brief This function tests if any one of three sensors has been pressed.
    #
    # <b> Returns </b> 
    #  <blockquote>
    # True if a sensor is pressed
    def is_pressed(self):
        # print(f"PinA {self.Pin_A.value()}, pinb  {self.Pin_B.value()}, pinc  {self.Pin_A.value()}")
        if 0 in (self.Pin_A.value(), self.Pin_B.value(), self.Pin_C.value()):
            return True
        else:
            return False


        
