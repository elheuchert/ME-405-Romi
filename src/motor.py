from pyb import Pin, Timer
from time import sleep_ms
## @brief The Motor Diver Class
# 
# This class is used to create motor objects
class Motor:
    #A motor driver interface encapsulated in a Python class. Works with
    #motor drivers using separate PWM and direction inputs such as the DRV8838
    #drivers present on the Romi chassis from Pololu.

    ## @brief Motor Initialization
    # @param P1 Motor Effort Pin
    # @param P2 Motor Direction Pin
    # @param P3 Motor Enable pin
    # @param P4 Motor timer channel
    def __init__(self, PWM, DIR, nSLP, timer, ch):
        #Initializes a Motor object
        self.nSLP = Pin(nSLP, mode=Pin.OUT_PP, value=0)  # Defining left motor pins
        self.dir = Pin(DIR, mode=Pin.OUT_PP, value=0)
        self.timCH = timer.channel(ch, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)

    ## @brief Set Effort
    # Sets the present effort requested from the motor based on an input value between -100 and 100
    def set_effort(self, effort):
        if effort < 0:
            self.dir.high()
            self.timCH.pulse_width_percent(-effort)
        else:
            self.dir.low()
            self.timCH.pulse_width_percent(effort)
            
    ## @brief Enable
    ## Enables the motor driver by taking it out of sleep mode into brake mode
    def enable(self):
        self.nSLP.high()
        self.timCH.pulse_width_percent(0)
    ## @brief Disable
    # Disables the motor driver by taking it into sleep mode
    def disable(self):
        self.nSLP.low()
