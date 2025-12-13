## @file controller.py
#  This file contains a class to create a PID Controller object. These objects are always
#  initialized as PID controllers but can be ran as any combination of P, I, and D controllers. 
#  They are to be treated as classical controls single input, single output controllers. 
#
#  The controller object has gains that are given to it on initialization. The object holds a set point
#  and will use classical control theory of errors from the setpoint for return values when asking for 
#  control. The object also handles saturating the integral error sum and the output effort as set on
#  initialization. 
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @date   2025-Sept-28 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 

## @brief The Controller class is a classical PID controller that implements single input, single output control when called to. 
#         The user gives it gains and setpoints and calls the control function for efforts. 
#
class Controller:
## @brief __init__ is the Controller object initializer which takes 5 arguments. 
#         The arguments the gains and saturation points of the classical controller. 
#  @param K_p : K_p is the proportional gain. 
#  @param K_i : K_i is the integral gain. 
#  @param K_d: K_d is the derivative gain, but it is not implemented as it was not used in the project. 
#  @param err_sat : err_sat is the saturation point for the integral of the error to stop I control from becoming unstable.
#  @param eff_sat : eff_sat is the saturation point for the output effort. 

    def __init__(self, K_p, K_i, K_d, err_sat, eff_sat):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.summed_errors = 0
        self.reference = 0
        self.err_sat=err_sat
        self.effort_saturation=eff_sat

## @brief change_Ref changes the set point of the controller. 
#  @param reference : reference is the new set point of the controller.

    def change_Ref(self, reference):
        self.reference = reference
        # print("Reference set")
        # print(self.reference)
    
## @brief get_Ref returns the current set point of the controller.
# 
    def get_Ref(self):
        return self.reference
## @brief get_Error returns the current Riemann Sum integral of the errors of the controller.
# 
    def get_Error(self):
        return self.summed_errors
## @brief proportional_Control returns a saturated effort value based on the error of the system from the set point using proportional control only. 
#  @param measure : measure is the current state of the system being controlled
# 
    def proportional_Control(self, measure):
        error = self.reference - measure
        cntrl_val = error * self.K_p
        
        if cntrl_val > 100:
            return 100
        elif cntrl_val < -100:
            return -100
        else:
            return cntrl_val
## @brief pi_Control returns a saturated effort value based on the error of the system from the set point using proportional and integral control only.  
#  @detail The integral of the error is done using a simple Riemann sum that looks at how long it has been since the last time the controller was ran. This 
#          integral is saturated to stop the integral from making the motors go insane when some unexpected behavior occurs. The effort is also saturated. 
# @param measure : measure is the current state of the system being controlled
# @param dt : dt is the time since the last run of pi_Control in ms 
# 
    def pi_Control(self, measure, dt):
        #print(f"Measure : {measure}")
        #print(f"Reference : {self.reference}")
        error = self.reference - measure
        #print(f"error : {error}")
        self.summed_errors += error * (dt/1000000)
        self.summed_errors = self.error_sat(self.summed_errors)
        cntrl_val = ((error * self.K_p)+(self.summed_errors * self.K_i))
        return self.effort_sat(cntrl_val)
## @brief error_sat is a function which returns the saturated integral of the error based on this instance of the controller saturation value. 
#  @param err : err is the error sum value being saturated. 
#   
    def error_sat(self, err):
        if err >= self.err_sat:
            return self.err_sat
        elif err <= -self.err_sat:
            return -self.err_sat
        else:
            return err
## @brief effort_sat is a function which returns the saturated effort of the controller based on this instance of the controllers saturation value.  
#  @param eff : eff is the effort value being saturated. 
#         
    def effort_sat(self, eff):
        if eff >= self.effort_saturation:
            return self.effort_saturation
        elif eff <= -self.effort_saturation:
            return -self.effort_saturation
        else:
            return eff
## @brief change_Gains changes this instance's gain values.  
#  @param k_p : k_p is the new proportional gain. 
#  @param k_i : k_i is the new integral gain. 
#  @param k_d : k_d is the new derivative gain. 
# 
    def change_Gains(self, k_p, k_i, k_d):
        self.K_p = k_p
        self.K_i = k_i
        self.K_d = k_d
