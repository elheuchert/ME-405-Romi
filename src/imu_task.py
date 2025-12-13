from pyb import UART
from struct import pack
## @brief Runs the IMU
#
# This task class which contains a generator finite state machine that runs multiple functions.
#  
# It sets up the IMU and collects data from it using its methods  
class imu_task:

    ## @brief Initialization
    # @param P1 IMU object
    # @param P2 IMU flag
    # @param P3 Romi Yaw Share
    # @param P4 Romi Yaw Velocity Share
    def __init__(self, imu, imu_flg, yaw, yaw_velocity):
        
        self.imu=imu
        self.imu_flg = imu_flg
        self.state=0
        self.count = 0
        self.calibrated = 0
        self.substate=0
        self.accel=0
        self.sys=0
        self.gyro=0
        self.mag=0
        self.config_flag = 0
        self.yaw = yaw
        self.yaw_velocity = yaw_velocity

    ## @brief Encoder Right Run
    #
    # <b> State 0 </b> Initialize the IMU by writing to it the coefficients and changing it to the desired fusion mode
    #
    # <b> State 1 </b> Put IMU data in shares for other tasks (note commented out code is for obtaining calibration coefficients)
    #
    # <b> State 2 </b> Delay
    def run(self):
        while (True):
            #init
            if self.state == 0:
                ## calibration mode
                self.imu.change_mode(0x00)
                (mode,) = self.imu._read_reg(self.imu.reg.OPR_MODE)
                print(f'mode: {hex(mode)}')
                self.imu.write_calibration_coeff(pack('<h',-47), pack('<h',166),pack('<h',-34), pack('<h',-341),
                                                     pack('<h',481), pack('<h',480), pack('<h',-2), pack('<h',-3),
                                                       pack('<h',0),pack('<h',1000), pack('<h',480))
                # fusion mode = 0C
                # imu mode =08
                self.imu.change_mode(0x08)
                (mode,) = self.imu._read_reg(self.imu.reg.OPR_MODE)
                print(f'mode: {hex(mode)}')
                self.state = 1
                yield 0

            
            elif self.state == 1:
                
                self.yaw.put(self.imu.get_Euler_x())
                self.yaw_velocity.put(self.imu.get_Roll_Velocity())
                

                #if self.imu_flg.get()==1:
                    # print("imu flag true")
                    # (self.sys,self.gyro,self.accel,self.mag) = self.imu.calibration_check()
                    # print("checked calibration")
                    # print(self.sys, self.gyro, self.accel, self.mag)
                    
                    #print(f"yaw: {self.yaw}")
                    #print(f"yawvelo: {self.yaw_velocity}")
                

                

                    
                    
                
                #     if self.accel==3 and self.gyro==3 and self.mag==3:
                #         self.config_flag = 1
                #         self.imu.change_mode(0x00)

                #     if self.config_flag == 1:
                #         (self.acc_x, self.acc_y, self.acc_z,self.mag_x, self.mag_y, self.mag_z,self.gyr_x, self.gyr_y, self.gyr_z, self.acc_rad, self.mag_rad)=self.imu.get_calibration_coeff()
                #         print(self.acc_x, self.acc_y, self.acc_z,self.mag_x, self.mag_y, self.mag_z,self.gyr_x, self.gyr_y, self.gyr_z, self.acc_rad, self.mag_rad)
                    
                #     (mode,) = self.imu._read_reg(self.imu.reg.OPR_MODE)
                #     print(f'fusion mode: {hex(mode)}')
                
                
                    # if self.substate==0:
                    #     self.sys, self.gyry,self.acc, self.mag=self.imu.calibration_check()
                    #     self.substate+=1
                        
                    #     yield 1
                    # elif self.substate==1:
                    #     #self.mag=self.imu.calibration_check_mag()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==2:
                    #     #self.acc=self.imu.calibration_check_accel()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==3:
                    #     #self.gyr=self.imu.calibration_check_gyro()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==4:
                    #     self.acc_coeff=self.imu.get_calibration_coeff_accel()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==5:
                    #     self.gyr_coeff=self.imu.get_calibration_coeff_gyro()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==6:
                    #     self.mag_coeff=self.imu.get_calibration_coeff_mag()
                    #     self.substate+=1
                    #     yield 1
                    # elif self.substate==7:
                    #     print(f"calib check: {self.sys},{self.gyr},{self.acc},{self.mag}")
                    #     yield 1
                    #     print(f"calib coeff: {self.gyr_coeff},{self.acc_coeff},{self.mag_coeff}")
                    #     self.imu_flg.put(0)
                    #     self.substate=0
                    #     yield 1
                yield 1
                # else:
                #     yield 1    
                    
                    # if gyr == 3:
                    #     if self.calibrated == 1:
                    #         print("gyro calibrated")
                    #     else:
                    #         self.calibrated = 1
                    #         self.state = 2
                    #     if mag == 3:
                    #         if self.calibrated == 2:
                    #             print("mag calibrated")
                    #         self.calibrated = 2
                    #         self.state = 2
                    #         if acc == 3:
                    #             if self.calibrated == 3:
                    #                 print("acc calibrated")
                    #             self.calibrated = 3
                    #             self.state = 2
                    #             if sys == 3:
                    #                 if self.calibrated == 3:
                    #                     print("sys calibrated")
                    #                 self.calibrated = 4
                    #                 self.state = 2
                    #                 self.calibrated = True
                                    

                    # print(f"check calb: {sys},{mag},{acc},{gyr}")
                 
                    # msg = "calib:{},{},{},{}".format(
                    # sys, gyro, accel, mag
                    # )

                    # print(msg)

                    # print(f'calib: {self.imu.calibration_check()}')
                    # print(f'calib coeff: {self.imu.get_calibration_coeff()}')
                    # # print(f'euler heading: {self.imu.get_Euler_Heading()}')
                    # print(f'euler yaw: {self.imu.get_Euler_Heading()}')
                    # #(mode,) = self.imu._read_reg(self.imu.reg.OPR_MODE)
                    # #print(f'fusion mode: {hex(mode)}')

                
            elif self.state == 2:
                self.count += 1
                if self.count == 1000:
                    self.count = 0
                    self.state = 1
            else:
                raise ValueError("Not that many states")
            yield self.state
