## @file line_task.py
#  This file contains a class to create a line_task object which is compatible with
#  the cotask scheduler to run as a cooperative task. 
#  
#  This task is a finite state machine implementation that calibrates the line sensor on startup 
#  and then simply reads the centroid of the line sensor forever once its calibrated. 
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
#  @date   2025-Nov-11 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 
import line_sensor
import os
import gc
SENSOR_START = 0
SENSOR_END = 12
## @brief line_task class that can be initialized to be run with the cotask scheduler. See run for behavior. 
#
class line_task:
## @brief __init__ is the line_task object initializer which takes 6 arguments. 
#         The arguments are a mix of hardware drivers and share flags. 
#  @param line_sensor : line_sensor is the line_sensor object made in main. It is the sensor this task is reading.
#  @param l_state : l_state is not actually used 
#  @param need_Calibrate : need_Calibrate is an integer share flag that tells control task whether the IR sensors are calibrated and control task is ok to start
#                          line following. It is set by the line sensor task. 
#  @param ready_Black : ready_Black is an integer share flag that UI task will set true when the ROMI is over a black surface for calibration. 
#  @param ready_White : ready_White is an integer share flag that UI task will set true when the ROMI is over a white surface for calibration. 
#  @param centroid : centroid is a float share that the line sensor task fills with the current location of the line that ROMI is following. It is used for the
#                    automatic line following mode in control task. 
#
    def __init__(self, lineSensor, l_state, need_Calibrate, ready_Black, ready_White, centroid):
        self.lineSensor = lineSensor
        self.l_state = l_state 
        self.state = 0
        self.need_Cal = need_Calibrate
        self.ready_Black = ready_Black
        self.ready_White = ready_White
        self.centroid = centroid
        self.black_data = [0.0]*13
        self.white_data = [0.0]*13
        self.got_black = False
        self.got_white = False
## @brief run is the generator that the cotask scheduler will run since our tasks are objects. All shares it needs are already given to 
#         the task object on initialization. The generator will only run one state at a time on each run of the task.
#  @detail The finite state machine for control_task has 4 states. 
#          State 0: Check calibration state
#          State 1: Calibrating state. Creates the calibration file
#          State 2: Calibrated/Reading state. This is the final running state that line_task will sit in filling centroid forever once its calibrated
#          State 3: Calibration check state. Come here to check if the sensor is reading the file 
    def run(self):

        while (True):
            # Check calibration state
            if self.state == 0:
                ans = self.lineSensor.calibrate()
                print(ans)
                if ans:
                    print("sensor calibrated")
                    self.state = 2
                else:
                    print("Sensor not calibrated")
                    self.state = 1
                    self.need_Cal.put(1)
                yield 0

            # Calibrating state
            elif self.state == 1:    
                if self.ready_Black.get():
                    #print("Getting Black")
                    self.black_data = list(self.lineSensor.read_Calibrate_Data())
                    self.ready_Black.put(0)
                    self.got_black = True
                    #print(self.black_data)
                if self.ready_White.get():
                    #print("Getting White")
                    self.white_data = list(self.lineSensor.read_Calibrate_Data())
                    self.ready_White.put(0)
                    self.got_white = True
                    #print(self.white_data)
                if self.got_black and self.got_white == True:
                    #print("Got my data trying to make a file")
                    self.need_Cal.put(0)
                    self.state = 3
                    # # Get the folder where *this script* is located
                    # current_folder = os.path.dirname(os.path.abspath(__file__))
                    # # Build the full path to the new text file
                    # file_path = os.path.join(current_folder, "IR_cal.txt")
                    file_path = "/flash/IR_cal.txt"
                    #print("file creation complete")
                    # 1) See where a relative path points
                    
                    # Write both arrays to the file
                    with open(file_path, "w") as file:
                        # for w, b in zip(self.white_data, self.black_data):
                        #     file.write(f"{w},{b}\n")
                        combined = [f"{a},{b}" for a, b in zip(self.black_data, self.white_data)]
                        for line in combined:
                            file.write(line + "\n")
                        # for x, y in zip(self.black_data, self.white_data):
                        #     file.write(f"{x}\t{y}\n")
                yield 1

            # Reading state
            elif self.state == 2:
                gc.collect()
                self.centroid.put(self.lineSensor.getCentroid())
                yield 2
            
            elif self.state == 3:
                ans = self.lineSensor.calibrate()
                print(ans)
                if ans:
                    self.state = 2
                yield 3
            else:
                raise ValueError("Not that many states")
            yield self.state 
