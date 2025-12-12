## @package imu
#  @brief Description of this module
#  @brief IMU sensor interface
#
#  More detailed description here.
from pyb import I2C
from struct import calcsize, unpack_from 
from micropython import const

## @brief add description here
class IMU:

    DEV_ADDR = 0x28
    
    ## @brief add description here
    class reg:
        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1B), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"hhh")

        OPR_MODE = (const(0x3D), b"<B")
        CALIB_STAT = (const(0x35), b"<B")

        CALIB_COEFF_START = (const(0x55), b"hhhhhhhhhhh")
        CALIB_COEFF_STOP = (const(0x81), b"b")

        ACC_OFFSET= (const(0x55), b"<hhh")
        MAG_OFFSET=(const(0x5B), b"<hhh")
        GYR_OFFSET=(const(0x61), b"<hhh")

        

        EUL_Data_X= (const(0x01A), b"<h")
        EUL_Data_Y= (const(0x01C), b"<h")
        EUL_Data_Z= (const(0x01E), b"<h")


        GYR_DATA_X = (const(0x014), b"<h")
        GYR_DATA_Y = (const(0x016), b"<h")
        GYR_DATA_Z = (const(0x018), b"<h")
        
        

        
    def __init__(self, i2c): 

        self._buf = bytearray((0 for n in range(22))) 
        self._i2c = i2c
        self._buf_8 = bytearray((0))

        return
    
    def _read_reg(self, reg): 

        # Determine number of bytes to read 
        length = calcsize(reg[1]) 

        # Create a memoryview object of the right size 
        buf = memoryview(self._buf)[:length] 

        # Read from the I2C bus into the memoryview
       
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0]) 

        # Unpack the bytes into a tuple 
        return unpack_from(reg[1], buf) 


    def euler(self):     

        head, roll, pitch = self._read_reg(IMU.reg.EUL_DATA_ALL) 

        return (head/16, roll/16, pitch/16) 
    
    def calibration_check(self):
        data = self._read_reg(self.reg.CALIB_STAT)
        sys=(data[0]>>6) & 0x03
        gyro=(data[0]>>4) & 0x03
        accel=(data[0]>>2) & 0x03
        mag= data[0] & 0x03
        
        print ("returning tuple")
        return (sys,gyro,accel,mag)

    def calibration_check_sys(self):
        self._buf_8[0] = self._read_reg(self.reg.CALIB_STAT)[0]
        sys=(self._buf_8[0]>>6) & 0x03
        return sys
    
    def calibration_check_gyro(self):
        data=self._read_reg(self.reg.CALIB_STAT)[0]
        gyro=(data>>4) & 0x03
        return gyro
    
    def calibration_check_accel(self):
        data=self._read_reg(self.reg.CALIB_STAT)[0]
        accel=(data>>2) & 0x03
        return accel
    
    def calibration_check_mag(self):
        data=self._read_reg(self.reg.CALIB_STAT)[0]
        mag= data & 0x03
        return mag

    def get_calibration_coeff_accel(self):
        (acc_x, acc_y, acc_z)=self._read_reg(self.reg.ACC_OFFSET)
        
        #a= ( f"{acc_x & 0xFFFF:016b}",f"{acc_y & 0xFFFF:016b}",f"{acc_z & 0xFFFF:016b}")
    
        return (acc_x, acc_y, acc_z)
    def get_calibration_coeff_mag(self):
        (mag_x, mag_y, mag_z)=self._read_reg(self.reg.MAG_OFFSET)
        #b= ( f"{mag_x & 0xFFFF:016b}",f"{mag_y & 0xFFFF:016b}",f"{mag_z & 0xFFFF:016b}")
       
        return (mag_x, mag_y, mag_z)
    
    def get_calibration_coeff_gyro(self):
        (gyr_x, gyr_y, gyr_z)=self._read_reg(self.reg.GYR_OFFSET)
        #c= ( f"{gyr_x & 0xFFFF:016b}",f"{gyr_y & 0xFFFF:016b}",f"{gyr_z & 0xFFFF:016b}")
        
        
        return (gyr_x, gyr_y, gyr_z)
    def get_calibration_coeff(self):
        (acc_x, acc_y, acc_z,mag_x, mag_y, mag_z,gyr_x, gyr_y, gyr_z, acc_rad, mag_rad)=self._read_reg(self.reg.CALIB_COEFF_START)
        return (acc_x, acc_y, acc_z,mag_x, mag_y, mag_z,gyr_x, gyr_y, gyr_z, acc_rad, mag_rad)

    def change_mode(self, mode):
        #current_reg = self._read_reg(self.reg.OPR_MODE)[0]
        #current_reg = (current_reg & 0b11110000) | (mode & 0b00001111)
        #current_reg = mode & 0b00001111
        self._i2c.mem_write(bytes([mode & 0b00001111]), self.DEV_ADDR, self.reg.OPR_MODE[0])
        return

    def write_calibration_coeff(self, coef1, coef2, coef3, coef4, coef5, coef6, coef7, coef8, coef9, coef10, coef11):
        coeffs=[coef1,coef2, coef3, coef4, coef5, coef6, coef7, coef8, coef9, coef10, coef11 ]
        coeff_bits=b''.join(coeffs)
        
        self._i2c.mem_write(coeff_bits,self.DEV_ADDR, self.reg.CALIB_COEFF_START[0])
        return
    
    def get_Euler_x(self):
        (eul_x,)=self._read_reg(self.reg.EUL_Data_X)  
        # x= eul_x & 0xFFFF
        # return f"{x:016b}"
        x=(eul_x/900.00)
        return x
    def get_Euler_y(self):
        (eul_y,)=self._read_reg(self.reg.EUL_Data_Y)  
        # x= eul_y & 0xFFFF
        # return f"{x:016b}"
        x=(eul_y/900.00)
        return x
    def get_Euler_z(self):
        (eul_z,)=self._read_reg(self.reg.EUL_Data_Z)  
        # x= eul_z & 0xFFFF
        # return f"{x:016b}"
        x=(eul_z/900.00)
        return x
    def get_Yaw_Velocity(self):
        (gyr_z,) = self._read_reg(self.reg.GYR_DATA_Z)
        x=(gyr_z/900.00)
        return x
    def get_Roll_Velocity(self):
        (gyr_x,) = self._read_reg(self.reg.GYR_DATA_X)
        x=(gyr_x/900.00)
        return x
    def get_Pitch_Velocity(self):
        (gyr_y,) = self._read_reg(self.reg.GYR_DATA_Y)
        x=(gyr_y/900.00)
        return x