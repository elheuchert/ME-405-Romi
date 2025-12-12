## @brief add description here
class control_task: 

    def __init__(self, fwd_ref, arc_ref, piv_ref, c_state, my_controller_left, my_controller_right, 
                 position_l, velocity_l, position_r, velocity_r, PWM_l, PWM_r, encoder_Right, encoder_Left, 
                 m_state_l, m_state_r, centroid, line_controller, automatic_mode, line_sensor, need_Calibrate, real_yaw, centroid_goal, yaw_goal, controller_yaw):
        self.fwd_ref = fwd_ref
        self.arc_ref = arc_ref
        self.piv_ref = piv_ref
        self.c_state = c_state
        self.controller_left = my_controller_left
        self.controller_right = my_controller_right
        self.position_l = position_l 
        self.velocity_l = velocity_l
        self.position_r = position_r
        self.velocity_r = velocity_r
        self.PWM_l = PWM_l
        self.PWM_r = PWM_r
        self.state = 0
        self.enc_R = encoder_Right
        self.enc_L = encoder_Left
        self.m_state_l = m_state_l
        self.m_state_r = m_state_r
        self.centroid = centroid
        self.line_controller = line_controller
        self.automatic_mode = automatic_mode
        self.yaw_rate = 0
        self.c_state.put(0)
        self.line_sensor = line_sensor
        self.need_Calibrate = need_Calibrate
        self.real_yaw = real_yaw
        self.centroid_goal = centroid_goal
        self.yaw_goal = yaw_goal
        self.controller_yaw = controller_yaw
        self.idx = 0
        self.startup = 0

    def run(self):
        
        while(True):
            
            if self.state == 0:

                if self.automatic_mode.get() == 1:
                    print("going dark")
                    self.state = 6
                    self.line_controller.change_Ref(0)
                    if self.startup == 0:
                        self.enc_R.zero()
                        self.enc_L.zero()
                        self.enc_R.set_offset(((.141/2))*self.real_yaw.get())
                        self.enc_L.set_offset(-((.141/2))*self.real_yaw.get())
                        self.startup = 1
                elif self.c_state.get() != 0:
                    self.state = self.c_state.get()
                yield 0
            # Forward control state
            elif self.state == 1:
                if self.c_state.get() == 4:
                    self.state = 4
                #print("going forward")
                #print(self.fwd_ref.get())
                self.controller_left.change_Ref(self.fwd_ref.get())
                #self.controller_right.change_Ref(self.fwd_ref.get())
                self.controller_right.change_Ref(self.fwd_ref.get())
                self.state = 0
                self.c_state.put(0)
                yield 1
            # Turn control state
            elif self.state == 2:
                if self.c_state.get() == 4:
                    self.state = 4
                v_r = 15*((2*self.arc_ref.get()+141)/(2*self.arc_ref.get()-141))
                self.controller_left.change_Ref(15)
                self.controller_right.change_Ref(v_r)
                self.state = 0
                self.c_state.put(0)
                yield 2
            # Pivot control state
            elif self.state == 3:
                if self.c_state.get() == 4:
                    self.state = 4
                self.controller_left.change_Ref(-(self.piv_ref.get()*141//2))
                #self.controller_left.change_Ref(100)
                #print(self.piv_ref.get()*141//2)
                self.controller_right.change_Ref((self.piv_ref.get()*141//2))
                #self.controller_right.change_Ref(100)
                #self.state = 0
                #self.c_state.put(0)
                yield 3
            elif self.state == 4:
                print("I tried to turn them off")
                self.controller_left.change_Ref(0)
                self.controller_right.change_Ref(0)
                self.fwd_ref.put(0)
                self.arc_ref.put(0)
                self.piv_ref.put(0)
                self.state = 0
                self.c_state.put(0)
                yield 4
            # Automatic control
            elif self.state == 5:
                #if not self.line_sensor.check_Line():
                # self.controller_left.change_Ref(0)
                # self.controller_right.change_Ref(0)
                # print("No line")
                #else:
                #print(f"got in state 5")
                if self.automatic_mode.get() == 0:
                    self.state = 4
                if self.c_state.get() == 4:
                    self.state = 4

                if self.centroid_goal.get() != self.line_controller.get_Ref():
                    print("Centroid Ref Changed")
                    self.line_controller.change_Ref(self.centroid_goal.get())
                    print(self.line_controller.get_Ref())
                    
                self.yaw_rate = self.line_controller.pi_Control(self.centroid.get(), 15) 
                #print("Yaw Rate")
                #print(f"yaw rate {self.yaw_rate}")
                #print(self.line_controller.get_Error())
                #print(self.yaw_rate)
                #print(f" centroid= {self.centroid.get()}")
                #if self.yaw_rate > 0.3 or self.yaw_rate < -0.3: 
                self.controller_left.change_Ref(100-self.yaw_rate)
                self.controller_right.change_Ref(100+self.yaw_rate)
                #else:
                #self.controller_left.change_Ref(75)
                #self.controller_right.change_Ref(75)
                yield 5

            elif self.state == 6:
                if self.need_Calibrate.get() == 0:
                    self.state = 5
                    print("CALIBRATED")
                yield 6

            elif self.state ==7:
                # Yaw alignment state
                yaw_actuation = self.controller_yaw.pi_Control(self.yaw_goal.get(), 15)
                self.controller_left.change_Ref(-yaw_actuation)
                self.controller_right.change_Ref(yaw_actuation)
                #print(yaw_actuation)
                if self.c_state.get() == 4:
                    self.state = 4
                if yaw_actuation <= 1:
                    self.yaw_goal.put(0)
                    self.controller_left.change_Ref(0)
                    self.controller_right.change_Ref(0)
                    self.state = 0
                yield 7
            else:
                raise ValueError("Not that many states")
            
            # For Proportional Tests
            #cntrl_l = self.controller_left.proportional_Control(self.velocity_l.get())
            #cntrl_r = self.controller_right.proportional_Control(self.velocity_r.get())
            #print(f"cntrl_l : {cntrl_l}")
            #print(f"cntrl_r : {cntrl_r}")
            #self.PWM_l.put(cntrl_l)
            #self.m_state_l.put(2)
            #self.PWM_r.put(cntrl_r)
            #self.m_state_r.put(2)
            # For PI Tests 
            new_left_pwm = self.controller_left.pi_Control(self.velocity_l.get(), self.enc_L.get_dt())
            new_right_pwm = self.controller_right.pi_Control(self.velocity_r.get(), self.enc_R.get_dt())

            #print(f"new_right {new_right_pwm}")
            #print(f"new_left  {new_left_pwm}")
            #self.PWM_l.put(new_left_pwm)
            #self.PWM_r.put(new_right_pwm)
            
            #print(f"new_left after filter {self.PWM_l.get()}")
            if new_left_pwm > 0 and new_right_pwm > 0:
                # Both positive
                self.PWM_l.put(new_left_pwm)
                self.PWM_r.put(new_right_pwm)
            elif new_left_pwm < 0 and new_right_pwm < 0:
                # Both negative
                self.PWM_l.put(new_left_pwm)
                self.PWM_r.put(new_right_pwm)
            elif new_left_pwm > 0 and new_right_pwm < 0:
                # Left positive, right negative
                #print("Flipping them 1")
                self.PWM_l.put(-new_left_pwm)
                self.PWM_r.put(-new_right_pwm)
            elif new_left_pwm < 0 and new_right_pwm > 0:
                # Left negative, right positive
                #print("Flipping them 2")
                self.PWM_l.put(-new_left_pwm)
                self.PWM_r.put(-new_right_pwm)
            elif new_left_pwm != 0 and new_right_pwm == 0:
                # Left non zero, right zero
                #print("Flipping them 3")
                self.PWM_l.put(-new_left_pwm)
                self.PWM_r.put(0)
            elif new_left_pwm == 0 and new_right_pwm != 0:
                # Left zero, right non zero
                #print("Flipping them 4")
                self.PWM_l.put(0)
                self.PWM_r.put(new_right_pwm)
            # print(f"new_right after filter {self.PWM_l.get()}")
            
            # self.PWM_l.put(self.controller_left.pi_Control(self.velocity_l.get(), self.enc_L.get_dt()))
            #self.PWM_l.put(0)
            # print(f"Left PWM{self.PWM_l.get()}")
            #print(self.PWM_l.get())
            #print(f"Left V: {self.velocity_l.get()}")
            #print(f"Right V: {self.velocity_r.get()}")
            #print(f"Left Reference {self.enc_L.get_dt()}")
            #print(f"Right Reference {self.enc_R.get_dt()}")
            #print(f"Left Reference {self.controller_left.get_Ref()}")
            #print(f"Right Reference {self.controller_right.get_Ref()}")
            #print(self.velocity_l.get())
            #print("Left Set Point")
            #print(self.controller_left.get_Ref())
            #self.PWM_r.put(self.controller_right.pi_Control(self.velocity_r.get(), self.enc_R.get_dt()))
            #self.PWM_r.put(-50)
            #print("Right PWM")
            #print(self.PWM_r.get())
            self.m_state_l.put(2)
            self.m_state_r.put(2)
            
            yield self.state