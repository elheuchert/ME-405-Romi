from pyb import Pin, Timer
from time import sleep_ms
## @brief add description here
class Motor:
    #A motor driver interface encapsulated in a Python class. Works with
    #motor drivers using separate PWM and direction inputs such as the DRV8838
    #drivers present on the Romi chassis from Pololu.

    def __init__(self, PWM, DIR, nSLP, timer, ch):
        #Initializes a Motor object
        self.nSLP = Pin(nSLP, mode=Pin.OUT_PP, value=0)  # Defining left motor pins
        self.dir = Pin(DIR, mode=Pin.OUT_PP, value=0)
        self.timCH = timer.channel(ch, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)

    def set_effort(self, effort):
        # Sets the present effort requested from the motor based on an input value
        # between -100 and 100
        if effort < 0:
            self.dir.high()
            self.timCH.pulse_width_percent(-effort)
        else:
            self.dir.low()
            self.timCH.pulse_width_percent(effort)
            

    def enable(self):
        # Enables the motor driver by taking it out of sleep mode into brake mode
        self.nSLP.high()
        self.timCH.pulse_width_percent(0)

    def disable(self):
        # Disables the motor driver by taking it into sleep mode
        self.nSLP.low()