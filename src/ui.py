from pyb import USB_VCP
from pyb import UART
## @brief Initializes User Interface
#
# UI task class which contains a generator finite state machine that starts tests and procedures on user input
#  
# This class runs continuously, setting flags that are shared with other tasks. 
class ui:
    ## @brief User Interface Initialization
    # @param P1 Testing Flag
    # @param P2 Left Motor State
    # @param P3 RIght Motor State
    # @param P4 Position
    # @param P5 Velocity
    # @param P6 Encoder Time
    # @param P7 PWM Left
    # @param P8 PWM Right
    # @param P9 Delay Flag
    # @param P10 UART Object
    # @param P11 Forward Reference Velocity
    # @param P12 Arc Reference Velocity
    # @param P13 Pivot Reference Velocity
    # @param P14 Controller State
    # @param P15 Velocity 2
    # @param P16 Need Calibrate Flag
    # @param P17 Black Calibration Flag
    # @param P18 White Calibration Flag
    # @param P19 Automatic Mode Flag
    # @param P20 IMU Flag
    def __init__(self, testing_flg, m_state_l, m_state_r, position, velocity, times, PWM_l, PWM_r,
                  delay, uart_obj,fwd_ref, arc_ref, piv_ref, c_state, velocity2, need_Calibrate, ready_Black, 
                  ready_White, automatic_mode, imu_flg):
        self.testing_flg = testing_flg
        self.m_state_l = m_state_l
        self.m_state_r = m_state_r
        self.position = position
        self.velocity = velocity
        self.times = times
        self.state=0
        self.ser=USB_VCP()
        self.output = 10
        self.last_test = 2
        self.next_test = 0
        self.time_idx = 0
        self.PWM_l = PWM_l
        self.PWM_r = PWM_r
        self.delay=delay
        self.uart = uart_obj
        self.headers_v = 0
        self.headers_p = 0
        self.ui_delay = 0
        self.fwd_ref = fwd_ref
        self.msg_send = 0
        self.arc_ref = arc_ref
        self.piv_ref = piv_ref
        self.c_state = c_state
        self.velocity2 = velocity2
        self.which_test = 0
        self.test_ref = 0
        self.ref_got = False
        self.need_Calibrate = need_Calibrate
        self.ready_Black = ready_Black
        self.ready_White = ready_White
        self.data_requested = 0
        self.c_state.put(0)
        self.automatic_mode = automatic_mode
        self.imu_flg = imu_flg
    
    ## @brief Runs the various tasks in UI
    #
    # <b> State 0 </b> Checking for Bluetooth Command
    #
    # <b> State 1 </b> Motor Step Resposne Test
    #
    # <b> State 2 </b> Wating for Test Data To be collected
    #
    # <b> State 3 </b> Delay State
    #
    # <b> State 4 </b> Move wheels Forward
    #
    # <b> State 5 </b> Turn in an arc
    #
    # <b> State 6 </b> Pivot in Place
    #
    # <b> State 7 </b> Stop wheels
    #
    # <b> State 8 </b> Testing Check
    #
    # <b> State 9 </b> Waiting for Data to be collected
    #
    # <b> State 10 </b> Turn in an arc
    #
    # <b> State 11 </b> Wait for Black Calibration Data
    #
    # <b> State 12 </b> Wait for White Calibration Data
    #
    # <b> State 13 </b> IMU to be used by the user
    #     
    def run(self):
       
        while True:
            # Check for start command from bluetooth
            if self.state==0:
                # Comment out if you want to run tests on the bot
                # Keep in if you want bot to just try to follow a line
                #print("checking if needs calibration")
                #print(self.need_Calibrate.get())
                if self.need_Calibrate.get() == 1:
                    print("checked Need calibrate)")
                    if self.data_requested == 0:
                        self.uart.write(b"Need Calibration Data\r\n")
                        self.data_requested = 1
                        self.state = 10
    
                self.delay.put(0)
                #self.uart.write(b"still running rn")
                if self.uart.any():
                    print("waiting")
                    # char_in = self.uart.read(1).decode()
                    # Full motor step response test 
                    if char_in == "s":
                        self.state=1
                    # Straight drive
                    elif char_in == "1":
                        self.state=4
                        self.ref_got = False
                    # Arced turn
                    elif char_in == "2":
                        self.state=5
                        self.ref_got = False
                    # Pivot turn
                    elif char_in == "3":
                        self.state=6 
                        self.ref_got = False
                    # Stop command
                    elif char_in == "0":
                        self.state = 7
                        self.ref_got = False
                    # Testing state 
                    elif char_in == "t":
                        print("got a t")
                        self.state = 8
                        self.ref_got = False
                        self.which_test = 0
                        self.test_ref = 0
                    elif char_in == "g":
                        self.state = 12
                    elif char_in == "i":
                        self.state = 13
                yield 0     
            # Test Setup    
            elif self.state == 1:
                # Check for motor delays
                if self.delay.get() == 0:
                    if self.testing_flg.get() == 0:
                        self.m_state_l.put(5)
                        self.m_state_r.put(5)
                # Send start commands
                if self.m_state_l.get()==1 and self.m_state_r.get() ==1:
                    # Flush queues
                    while self.velocity.any() == True:
                        self.velocity.get()
                    while self.position.any() == True:
                        self.position.get()
                    # Set motors
                    self.PWM_l.put(self.output)
                    self.PWM_r.put(self.output)
                    # Check for time for next kind of test
                    if self.next_test != 1:
                        self.testing_flg.put(self.last_test)
                    # If next kind of test go to new test
                    else:
                        self.testing_flg.put(self.last_test + 1)
                    # Go to next state and tell motors to update
                    self.state = 2
                    self.delay.put(0)
                    self.m_state_l.put(2)
                    self.m_state_r.put(2)
                yield 1
            # Waiting for data
            elif self.state == 2:
                # Velocity Tests
                if self.testing_flg.get() <= 3:
                    if self.velocity.full() == True:
                        if self.output == 10 and self.headers_v == 0:
                            self.uart.write(b"Times, Velocity(mm/s)\r\n")
                            self.headers_v = 1
                        while self.velocity.any() == True:
                            self.uart.write(str(self.time_idx).encode())
                            self.uart.write(", ")
                            vel = (self.velocity.get())
                            self.uart.write(f"{vel:.3f}".encode())
                            self.uart.write(b"\r\n")
                            self.time_idx += 1
                        self.output += 10
                        #If that was the last test go to next test type
                        if self.output >= 110: 
                            self.last_test = self.testing_flg.get()
                            self.next_test = 1
                            self.output = 10
                        self.time_idx = 0   
                        self.testing_flg.put(0) 
                        self.state = 3
                        yield 2
                # Position Tests
                else:
                    if self.position.full() == True:
                        if self.output == 10 and self.headers_p == 0:
                            self.uart.write(b"Times, Position(mm)\r\n")
                            self.headers_p = 1
                        while self.position.any() == True:
                            self.uart.write(str(self.time_idx).encode())
                            self.uart.write(", ")
                            pos  = (self.position.get())
                            self.uart.write(f"{pos:.3f}".encode())
                            self.uart.write(b"\r\n")
                            self.time_idx += 1
                        self.output += 10 
                        self.time_idx = 0
                        # If that was the last test go to wait state
                        if self.output >= 110:
                            self.last_test = self.testing_flg.get()
                            if self.last_test == 5:
                                # Reset variables for next test 
                                self.testing_flg.put(0)
                                self.next_test = 0
                                self.PWM_l.put(0)
                                self.PWM_r.put(0)
                                self.delay.put(1)
                                self.m_state_l.put(2)
                                self.m_state_r.put(2)
                                self.output = 10
                                self.last_test = 2
                                self.state = 0
                                self.headers_p = 0
                                self.headers_v = 0
                                yield 2 
                            self.next_test = 1 
                            self.output = 10
                        self.testing_flg.put(0)
                        self.state = 3
                        yield 2 
            elif self.state == 3:
                self.ui_delay += 1
                if self.ui_delay >= 10:
                    self.state = 1
                    self.ui_delay = 0
                yield 3
            # Go forward state
            elif self.state == 4:
                if self.uart.any():
                    if self.ref_got == False:
                        self.fwd_ref.put(int(self.uart.readline().decode().strip()))
                        #print(self.fwd_ref.get())
                        self.ref_got = True
                        self.c_state.put(1)
                if self.ref_got == True:
                    self.state = 0
                yield 4
            # Turn in an arc state
            elif self.state == 5:
                if self.uart.any():
                    if self.ref_got == False:
                        self.arc_ref.put(int(self.uart.readline().decode().strip()))
                        self.ref_got = True
                        self.c_state.put(2)
                if self.ref_got == True:
                    self.state = 0
                yield 5

            # Pivot in place state
            elif self.state == 6:
                if self.uart.any():
                    if self.ref_got == False:
                        self.piv_ref.put(int(self.uart.readline().decode().strip()))
                        self.ref_got = True
                        self.c_state.put(3)
                if self.ref_got == True:
                    self.state = 0
                yield 6

            # Stop command 
            elif self.state == 7:
                print("Went to state 7")
                self.arc_ref.put(0)
                self.fwd_ref.put(0)
                self.piv_ref.put(0)
                self.c_state.put(4)
                self.automatic_mode.put(0)
                self.state = 0
                yield 7

            # Testing check     
            elif self.state == 8:
                if self.uart.any():
                    if self.which_test == 0:
                        self.which_test = int(self.uart.readline().decode().strip())
                        print("Got which test")
                        print(self.which_test)
                    else:
                        self.test_ref = int(self.uart.readline().decode().strip())
                        print("Got a test ref")
                        print(self.test_ref)
                        
                # Setting up test 
                if self.test_ref != 0:
                    # Check for motor delays 
                    if self.delay.get() == 0:
                        
                        if self.testing_flg.get() == 0:
                            self.m_state_l.put(5)
                            self.m_state_r.put(5)
                            
                    # Send start commands if delay over
                    if self.delay.get() == 1:
                    #if self.m_state_l.get()==1 and self.m_state_r.get() ==1:
                        # Flush queues
                        print("flsuh queues")
                        while self.velocity.any() == True:
                            self.velocity.get()
                        while self.velocity2.any() == True:
                            self.velocity2.get()
                        print("setting testing flag")
                        self.testing_flg.put(6)
                    else:
                        yield 8 
                        continue
                    # Set up correct controller state 
                    print("setting up controller")
                    if self.which_test == 1:
                        self.fwd_ref.put(self.test_ref)
                        self.c_state.put(1)
                    elif self.which_test == 2:
                        self.arc_ref.put(self.test_ref)
                        self.c_state.put(2)
                    elif self.which_test == 3:
                        self.piv_ref.put(self.test_ref)
                        self.c_state.put(3)
                    self.state = 9
                yield 8
            # Waiting for Data
            elif self.state == 9:
                
                if (self.velocity.full() == True) & (self.velocity2.full() == True):
                    print("About to send headers")
                    self.uart.write(b"Times, Velocity(mm/s)\r\n")
                    while self.velocity.any() == True:
                        print("About to send left motor data")
                        self.uart.write(str(self.time_idx).encode())
                        self.uart.write(", ")
                        vel = (self.velocity.get())
                        self.uart.write(f"{vel:.3f}".encode())
                        self.uart.write(b"\r\n")
                        self.time_idx += 1
                        print("Sent that bitch")
                    self.time_idx = 0
                    while self.velocity2.any() == True:
                        print("About to send right motor data")
                        self.uart.write(str(self.time_idx).encode())
                        self.uart.write(", ")
                        vel = (self.velocity2.get())
                        self.uart.write(f"{vel:.3f}".encode())
                        self.uart.write(b"\r\n")
                        self.time_idx += 1
                        print("Sent it")
                    self.time_idx = 0
                    self.delay.put(0)
                    self.testing_flg.put(0)
                    print("Asking motors to turn off")
                    self.state = 7
            # Wait to get Black Calibration Data
            elif self.state == 10:
                if self.uart.any():
                    char_in = self.uart.read(1).decode()
                    if char_in == "1":
                        self.ready_Black.put(1)
                        self.state = 11
            # Wait to get White Calibration Data
            elif self.state == 11:
                if self.uart.any():
                    char_in = self.uart.read(1).decode()
                    if char_in == "1":
                        self.ready_White.put(1)
                        self.state = 0
            # Wait for go           
            elif self.state == 12:
                print("went to state 12")
                self.automatic_mode.put(1)
                self.state = 0
                yield 12
            elif self.state == 13:
                self.imu_flg.put(1)
                self.state = 0
                yield 13
            else:
                raise ValueError("Not that many states")
            yield self.state

    
