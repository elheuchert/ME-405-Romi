from motor import Motor
from encoder import Encoder
from pyb import Timer, Pin
## @brief Left Motor Encoder Class 
#
# This task class which contains a generator finite state machine that runs multiple functions.
#  
# It drives the motor, reads the encoder and runs continuously, setting flags that are shared with other tasks.  
class motor_encoder_left_class:   
    
    ## @brief User Interface
    # @param 1 Motor Object
    # @param 2 Encoder Object
    # @param 3 Left Motor State in UI
    # @param 4 Left Encoder Position
    # @param 5 Left Motor Position
    # @param 6 Left Motor Velocity
    # @param 7 Times
    # @param 8 Left Motor PWM
    # @param 9 Delay
    def __init__(self, motor, encoder, m_state_l, position_l, velocity_l, times, PWM_l, delay):
        self.my_motor_Left = motor
        self.encoder_Left = encoder
        self.m_state_l = m_state_l
        self.position_l = position_l
        self.velocity_l = velocity_l
        self.times = times
        self.PWM_l = PWM_l
        self.delay_count = 0
        self.state = 0
        self.delay=delay
    
    ## @brief Encoder Left Run
    #
    # <b> State 0 </b> Enable The Left Motor
    #
    # <b> State 1 </b> Wait to get a different command from UI
    #
    # <b> State 2 </b> Set Effort
    #
    # <b> State 3 </b> Disable Motor
    #
    # <b> State 4 </b> Enable Motor and Wait State
    #
    # <b> State 5 </b> Stop the Motor and Zero the Encoder Ticks

    def run(self):
        while True:
        # state 0 initialize

            if self.state == 0:
                self.my_motor_Left.enable()
                
                self.state = 1
                self.m_state_l.put(0)
                yield 0

            # state 1 wait for next state
            elif self.state == 1:

                self.encoder_Left.update()
                self.position_l.put(self.encoder_Left.get_position())
                self.velocity_l.put(self.encoder_Left.get_velocity())
                # Check share from UI for what to do with motor next
                if self.m_state_l.get() != 0:
                    self.state = self.m_state_l.get()

                yield 1 
            # state 2 Left set effort
            elif self.state == 2:
                self.encoder_Left.update()
                self.position_l.put(self.encoder_Left.get_position())
                self.velocity_l.put(self.encoder_Left.get_velocity())
                self.my_motor_Left.set_effort(self.PWM_l.get())
                #self.my_motor_Left.set_effort(100)
                #print(f"PWM LEFT IN MOTOR: {self.PWM_l.get()}")
                #print("PWM LEFT IN MOTOR: 50")
                self.m_state_l.put(0)
                self.state=1
                yield 2 
            # state 3 Left disable
            elif self.state == 3:
                self.my_motor_Left.disable()
                self.m_state_l.put(0)
                self.state=4
                yield 3
            # state 4 left enable wait
            elif self.state == 4:
                if self.m_state_l.get() ==  4:
                    self.my_motor_Left.enable()
                self.m_state_l.put(0)
                self.state = 1 
                yield 4
            elif self.state == 5:
                self.my_motor_Left.set_effort(0)
                self.delay_count += 1
                self.encoder_Left.zero()
                if self.delay_count == 100:
                    self.delay_count = 0
                    self.state = 1
                    self.m_state_l.put(1)
                    self.delay.put(1)
                    # print("m_state_1")
                    # print(self.m_state_l.get())
                    
                yield 5
            else:
                raise ValueError("Not that many states")
            yield self.state
