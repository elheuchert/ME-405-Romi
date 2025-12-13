## @file line_sensor.py
#  This file contains a class to create a line sensor object. The object can be used for calibrating the sensor
#  and reading the centroid location of the darkest spot the sensors see.
#
#  NOTE: The outer 2 ir sensors are turned off by setting the corresponding self.sens_dist value to 0. Helped with line
#        following lines that split. 
# 
#  IMPORTANT: Calibrating the sensor requires a text file name IR_cal.txt. This calibration will write that file
#             but the user will need to stop ROMI, pull the file off onto a local computer and reflash it to the ROMI.
#             If you do not, ROMI will not recognize the new file and will not calibrate.  
#
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @author Help from ChatGPT5
#  @date   2025-Nov-11 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 
from pyb import Pin, Timer, ADC
from os import listdir, remove
import array

DATA_AMT = 5 # Number of samples on multi read to be sampled 
PITCH = 8 # mm

## @brief Line_Sensor class which will calibrate the sensors, read data from the entire line, and find the centroid of the darkest spot. 
#
class Line_Sensor:
## @brief __init__ is the Line_Sensor object initializer which takes 13 arguments. 
#         The arguments the 13 IR sensor objects. 
#  @param irX : irX is IR sensor object X on the physical line. Must be passed in order they appear on sensor. 
#
    def __init__(self, ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8, ir9, ir10, ir11, ir12, ir13):
        self.ir1 = ir1
        self.ir2 = ir2
        self.ir3 = ir3
        self.ir4 = ir4
        self.ir5 = ir5
        self.ir6 = ir6
        self.ir7 = ir7
        self.ir8 = ir8
        self.ir9 = ir9
        self.ir10 = ir10
        self.ir11 = ir11
        self.ir12 = ir12 
        self.ir13 = ir13
        
        self.tim = Timer(6, freq = 20000)

        self.calibrated = False

        #Create a buffer to hold information from each sensor
        self.buf1 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf2 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf3 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf4 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf5 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf6 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf7 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf8 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf9 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf10 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf11 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf12 = array.array('H', (0 for i in range(DATA_AMT)))
        self.buf13 = array.array('H', (0 for i in range(DATA_AMT)))
    
        #Create array of ir sensor objects for easily looping over entire line
        self.ir_array = [self.ir1, self.ir2, self.ir3, self.ir4, self.ir5, self.ir6, self.ir7, self.ir8, self.ir9, self.ir10, self.ir11, self.ir12, self.ir13]
        self.ir_adc_array = [self.ir1.adc, self.ir2.adc, self.ir3.adc, self.ir4.adc, self.ir5.adc, self.ir6.adc, self.ir7.adc, self.ir8.adc, self.ir9.adc, self.ir10.adc, self.ir11.adc, self.ir12.adc, self.ir13.adc]
        self.data_buffer = [self.buf1, self.buf2, self.buf3,self.buf4,self.buf5,self.buf6,self.buf7,self.buf8,self.buf9,self.buf10,self.buf11,self.buf12,self.buf13 ]        
        self.data_avg = [0]*13 
        
        # White value first, Black value second   
        self.act_data = [0.0]*13

        # Location of each sensor in mm 
        self.sens_dist = [0, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, 0] * PITCH

        # Data array to hold calibration constants
        self.black=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.white=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
## @brief check_Calibrate returns the status of the calibration variable. True is sensor is calibrated. 
#
    def check_Calibrate(self):
        return self.calibrated

## @brief calibrate will try to read the calibration text IR_cal.txt and return whether or not the file exists. It will set the calibration status variable. 
#  @details Attempts to open a file name IR_cal.txt. If it can't find it, it will return False. When it does find it, it assumes the file is a file with calibration constants
#           of the form "BLACK_CALIBRATION_VAL,WHITE_CALIBRATION_VAL". Each row should be one IR sensor's data in the order of the ir sensors as passed to the class init. 
    def calibrate(self):
       
        try:
            with open('/flash/IR_cal.txt') as f:
                file_exists = True
        except OSError:
            file_exists = False
            
        if file_exists:
            with open("/flash/IR_cal.txt", "r") as file:
                idx = 0
                for line in file:
                    text=line.strip().split(',')
                    self.white[idx] = (float(text[1]))
                    self.black[idx] = (float(text[0]))
                    idx += 1
            self.calibrated = True
        else:
            self.calibrated = False

        return file_exists
## @brief getCentroid will return the value of the centroid of the darkest spot the sensors can see along the length of the line. 
#  @details This will read the latest data saved and do a basic centroid calculation on it. Right now it divides by idx-2 because 2 
#           sensors are turned off. 
#
    def getCentroid(self):
        sum = 0
        idx = 0
        self.readSensors()
        for val in self.act_data:
            
            sum += (val * self.sens_dist[idx])
            idx += 1
        idx += 1
        return sum/(idx-2)
## @brief readSensors will read every ir sensor with a time multi read and then normalize with the calibration data. It will save that data in the class and return it. 
#
    def readSensors(self):
        
        ADC.read_timed_multi(self.ir_adc_array, self.data_buffer, self.tim)
        idx = 0
        for array in self.data_buffer:
            sum = 0
            for number in array:
                sum += number
            self.data_avg[idx] = sum/DATA_AMT
            idx+=1   
        idx = 0
        for val in self.data_avg:
            self.act_data[idx] = (val - self.black[idx])/(self.white[idx]-self.black[idx])
            idx += 1

        return self.act_data
    
    #Gets raw data values for completing calibration
## @brief readSensors will read every ir sensor with a time multi read but will return the raw unnormalized data to be saved as calibration data. It will save this and return it. 
#
    def read_Calibrate_Data(self):
        ADC.read_timed_multi(self.ir_adc_array, self.data_buffer, self.tim)
        idx = 0
        for array in self.data_buffer:
            sum = 0
            for number in array:
                sum += number
            self.data_avg[idx] = sum/DATA_AMT
            idx+=1   
        return self.data_avg
## @brief check_Line is a debugging function to check that the line sensor is correctly setup with all the sensors. 
#
    def check_Line(self):
        check_list = list(self.readSensors())
        #print(check_list[6])
        for item in check_list:
            if item > 15:
                return True
        return False 
## @brief check_Indv is a debugging function to look at the data from a single IR sensor if you think it is reading poorly. 
#
    def check_Indv(self, which_sensor):
        return self.ir_adc_array[which_sensor].read()
    
