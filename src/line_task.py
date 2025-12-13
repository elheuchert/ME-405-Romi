import line_sensor
import os
import gc
SENSOR_START = 0
SENSOR_END = 12
## @brief add description here
class line_task:
    
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
