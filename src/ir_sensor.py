## @brief
#  This file contains the IR_sensor class which gets created for each sensor in the line sensor. 
#  This implementation is reminiscient of an old way we were going to do the sensor so really this 
#  class just holds the pin reference for the sensor in practice. 
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @date   2025-Nov-15 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made

from pyb import Pin, Timer, ADC

## @brief The IR_Sensor class is an object for each individual ir sensor on our line sensor. Really just holds which sensor is which pin. 
#
class IR_Sensor:
    
## @brief __init__ is the IR_Sensor object initializer which takes 1 argument. 
#  @param my_pin : my_pin is the pyb pin ID of the IR sensor 
#
    def __init__ (self, my_pin):
        self.adc = ADC(my_pin)
        self.black_ref = 0
        self.white_ref = 0
        self.calibrated = False