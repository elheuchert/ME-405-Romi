from motor import Motor
from encoder import Encoder
from pyb import Timer, Pin
## @brief add description here
class motor_encoder_right_class:

    def __init__(self, motor, encoder, m_state_r, position_r, velocity_r, times, PWM_r, delay):
        self.my_motor_Right = motor
        self.encoder_Right = encoder
        self.m_state_r = m_state_r
        self.position_r = position_r
        self.velocity_r = velocity_r
        self.times = times
        self.PWM_r = PWM_r
        self.delay_count = 0
        self.state = 0
        self.delay=delay

    def run(self):
        while True:

        # state 0 initialize
        
            if self.state == 0:
                self.my_motor_Right.enable()
                
                self.state = 1
                self.m_state_r.put(0)
                yield 0

            # state 1 wait for next state
            elif self.state == 1:
                self.encoder_Right.update()
                self.position_r.put(self.encoder_Right.get_position())
                self.velocity_r.put(self.encoder_Right.get_velocity())
                # Check share from UI for what to do with motor next
                if self.m_state_r.get() != 0:
                    self.state= self.m_state_r.get()
                yield 1 
            # state 2 Right set effort
            elif self.state == 2:
                self.encoder_Right.update()
                self.position_r.put(self.encoder_Right.get_position())
                self.velocity_r.put(self.encoder_Right.get_velocity())
                self.my_motor_Right.set_effort(self.PWM_r.get())
                #self.my_motor_Right.set_effort(50)
                #print(f"PWM RIGHT IN MOTOR: {self.PWM_r.get()}")
                #print("PWM RIGHT IN MOTOR: 50")
                self.m_state_r.put(0)
                self.state=1
                yield 2
            # State 3 Rigth disable
            elif self.state == 3:
                self.my_motor_Right.disable()
                self.m_state_r.put(0)
                self.state = 4
                yield 3
            # State 4 Right Enable Wait
            elif self.state == 4:
                if self.m_state_r.get() ==  4:
                    self.my_motor_Right.enable()
                self.m_state_r.put(0)
                self.state = 1 
                yield 4
            # State 5 
            elif self.state == 5:
                self.my_motor_Right.set_effort(0)
                self.encoder_Right.zero()
                self.delay_count += 1
                if self.delay_count == 100:
                    self.delay_count = 0
                    self.state = 1
                    self.m_state_r.put(1)
                    self.delay.put(1)
                    # print("m_state_2")
                    # print(self.m_state_r.get())
                yield 5
            else:
                raise ValueError("Not that many states")
            yield self.state