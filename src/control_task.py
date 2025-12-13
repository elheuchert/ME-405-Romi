## @file control_task.py
#  This file contains a class to create a control_task object which is compatible with
#  the cotask scheduler to run as a cooperative task. 
#
#  The active control_task object uses a finite state machine design to control the motion of
#  the Pololu ROMI kit used in ME 405, Fall 2025. The task is given 4 PI controllers made in main
#  that are linked to the left wheel, right wheel, the centroid of the line sensor, and the heading of
#  the IMU sensor. Each state sets the appropriate set point of each controller according the expected 
#  motion of the bot. 
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @author ChatGPT5 was used for debugging purposes but not to generate this code
#  @date   2025-Nov-10 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 
## @brief control_task class that can be initialized to be run with the cotask scheduler. See run for behavior. 
class control_task: 
## @brief __init__ is the control_task object initializer which takes 25 arguments. 
#         The arguments are a mix of controllers, data shares, and flag shares.
#  @param fwd_ref : fwd_ref is an integer share that holds the expected forward reference speed of ROMI in mm.s.
#                   It is set by the UI task for testing purposes or by pathing plan task for course navigation. 
#  @param arc_ref : arc_ref is an integer share that holds the expected turning arc of ROMI in mm of arc radius. 
#                   It is set by the UI task for testing purposes. 
#  @param piv_ref : piv_ref is a float share that holds the expected pivoting reference angular speed of ROMI in rad/s.
#                   It is set by the UI task for testing purposes or by pathing plan task for course navigation. 
#  @param c_state : c_state is an integer share flag that holds the expected state of control_task. 
#                   It is set by UI or pathing plan to tell control_task the expected motion of ROMI. 
#  @param my_controller_left : my_controller_left is a controller object made in main that has gains for controlling
#                              the motion of the left wheel. 
#  @param my_controller_right: my_controller_right is a controller object made in main that has gains for controlling
#                              the motion of the right wheel. 

#  @param position_l: position_l is a float share that the left motor and encoder class fills with the current left motor position.
#  @param velocity_l: velocity_l is a float share that the left motor and encoder class fills with the current left motor velocity.
#  @param position_r: position_r is a float share that the right motor and encoder class fills with the current right motor position.
#  @param velocity_r: velocity_r is a float share that the right motor and encoder class fills with the current right motor velocity.

#  @param PWM_l : PWM_l is a float share that control_task sets to a value -100 to 100 as the duty cycle of the left wheel's enable pin.
#                 The motor_encoder_left_class will use it. 
#  @param PWM_r : PWM_r is a float share that control_task sets to a value -100 to 100 as the duty cycle of the right wheel's enable pin.
#                 The motor_encoder_right_class will use it. 
#  @param encoder_Right: encoder_Right is an encoder object attached to the right encoder. It is used here to save an offset and zero the encoders
#                        for accurate observer behavior. It is also used to get the dt between runs for the integral sums in the motor controllers. 
#  @param encoder_Left :encoder_Left is an encoder object attached to the left encoder. It is used here to save an offset and zero the encoders
#                        for accurate observer behavior. It is also used to get the dt between runs for the integral sums in the motor controllers. 
#  @param m_state_l : m_state_l is an integer share flag used by control task to tell the left motor encoder class which state the left motor needs to be in. 
#  @param m_state_r : m_state_r is an integer share flag used by control task to tell the right motor encoder class which state the right motor needs to be in. 

#  @param centroid : centroid is a float share that the line sensor task fills with the current location of the line that ROMI is following. It is used for the
#                    automatic line following mode in control task. 
#  @param line_controller : line_controller is a controller object made in main that has gains for changing the set point of the wheel controllers slightly to move
#                           the centroid of the line to a certain point. 
#  @param automatic_mode :  automatic_mode is an integer share flag that is set to tell control_task that it should start automatically following a line. 
#                           It is set by UI for testing or pathing plan for course navigation.
#  @param line_sensor : line_sensor is the line_sensor object made in main. It is not used in control task's current implementation but was used before for debugging the line following. 
#  @param need_Calibrate : need_Calibrate is an integer share flag that tells control task whether the IR sensors are calibrated and control task is ok to start
#                          line following. It is set by the line sensor task. 
#  @param real_yaw : real_yaw is a float share that the IMU task fills with the current yaw of ROMI. control_task uses it for heading control. 
#  @param centroid_goal: centroid_goal is a float share that the pathing plan task sets. It is then used as the line following centroid controller set point. 
#  @param yaw_goal : yaw_goal is a float share that is the set point of the heading control. It is the angle ROMI should turn to on the course and is set by
#                    pathing plan for course navigation. 
#  @param controller_yaw : controller_yaw is a controller object made in main. It looks at the current yaw of ROMI to get new wheel speeds for pivoting towards the yaw_goal. 
#                          It is set by the pathing plan for course navigation. 

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
## @brief run is the generator that the cotask scheduler will run since our tasks are objects. All shares it needs are already given to 
#         the task object on initialization. The generator will only run one state at a time on each run of the task, but it will always 
#         update the PWM of the motors to their expected set point on every loop. 
#  @detail The finite state machine for control_task has 8 states. 
#          State 0: Hub State. This state waits for either automatic_mode to be set or c_state to be set for a state change to active controls.
#          State 1: Forward State. If c_state == 1, control task will use the fwd_ref to drive the wheels forward at that speed in mm/s
#          State 2: Turn State. If c_state == 2, control task will use the arc_ref to drive the wheels to that turn radius in mm
#          State 3: Pivot State. If c_state == 3, control task will use the piv_ref to drive the wheels to pivot in place at that angular speed in rad/s
#          State 4: Disable State. If c_state ==4, control task will turn both motors off and clear all references then return to hub
#          State 5: Automatic Control State. If automatic_mode == 1 and the line sensor is calibrated, control task will drive forward at a set 100 mm/s
#                   and made small adjustments to each wheel to place the centroid at centroid_goal based off of the centroid controllers values.
#          State 6: Calibrate Check State. If automatic_mode == 1, control task will check the line sensor is calibrated before following the line. 
#          State 7: Heading Control State. If c_state == 7, control task will use the yaw controller to get a reference pivoting speed to move the motors
#                   unitl the yaw matches close to yaw_goal. 
#          Always : Every loop runs the control loops on the motors to keep them up to date on their reference speeds. 
#
#          Note   : Many of the comments at the bottom of the file are to comment out print statements for debugging purposes
#
#          IMPORTANT PWM Logic Filtering:
#          Unknown why to the authors but testing found that giving both wheels the same sign on PWM will have expected behavior.
#          If either PWM is 0 or the PWM values have different signs the motors will spin in the opposite direction of the sign of the PWM. 
#          The if logic at the bottom of the generator handles this but it is unknown if this is required if a different piece of ROMI hardware was used
#          or if this is a software issue. If wheels immediately saturate on turning it is likely this issue does not exist for users hardware. 

    def run(self):
        
        while(True):
            
            if self.state == 0:
            # Hub State
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
            # Disable State
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
                self.controller_left.change_Ref(100-self.yaw_rate)
                self.controller_right.change_Ref(100+self.yaw_rate)
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

            #IMPORTANT LOGIC CHECKS READ @detail
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
            
            #Ask motors to update their PWM every task cycle
            self.m_state_l.put(2)
            self.m_state_r.put(2)
            
            yield self.state
