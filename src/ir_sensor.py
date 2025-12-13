from pyb import Pin, Timer, ADC
## @brief add description here
class IR_Sensor:

    def __init__ (self, my_pin):
        self.adc = ADC(my_pin)
        self.black_ref = 0
        self.white_ref = 0
        self.calibrated = False
