from pyb import Pin, Timer, ADC
from os import listdir, remove
import array

DATA_AMT = 5
PITCH = 8 # mm
## @brief add description here
class Line_Sensor:
    #A motor driver interface encapsulated in a Python class. Works with
    #motor drivers using separate PWM and direction inputs such as the DRV8838
    #drivers present on the Romi chassis from Pololu.

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
    

        self.ir_array = [self.ir1, self.ir2, self.ir3, self.ir4, self.ir5, self.ir6, self.ir7, self.ir8, self.ir9, self.ir10, self.ir11, self.ir12, self.ir13]
        self.ir_adc_array = [self.ir1.adc, self.ir2.adc, self.ir3.adc, self.ir4.adc, self.ir5.adc, self.ir6.adc, self.ir7.adc, self.ir8.adc, self.ir9.adc, self.ir10.adc, self.ir11.adc, self.ir12.adc, self.ir13.adc]
        self.data_buffer = [self.buf1, self.buf2, self.buf3,self.buf4,self.buf5,self.buf6,self.buf7,self.buf8,self.buf9,self.buf10,self.buf11,self.buf12,self.buf13 ]        
        self.data_avg = [0]*13 
        
        # White value first, Black value second   
        self.act_data = [0.0]*13

        self.sens_dist = [0, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, 0] * PITCH


        
        self.black=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.white=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        

    def check_Calibrate(self):
        return self.calibrated
    
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
    
    def getCentroid(self):
        sum = 0
        idx = 0
        self.readSensors()
        for val in self.act_data:
            
            sum += (val * self.sens_dist[idx])
            idx += 1
        idx += 1
        return sum/(idx-2)
    
    def getCentroidCut(self, start, end):
        sum = 0
        idx = 0
        self.readSensors()
        for val in self.act_data:
            if (idx >= start) and (idx <= end):
                sum += (val * self.sens_dist[idx])
            idx += 1
        idx += 1
        return sum/(end-start)
    
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

    def check_Line(self):
        check_list = list(self.readSensors())
        #print(check_list[6])
        for item in check_list:
            if item > 15:
                return True
        return False 

    def check_Indv(self, which_sensor):
        return self.ir_adc_array[which_sensor].read()
    