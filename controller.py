## @brief add description here
class Controller:

    def __init__(self, K_p, K_i, K_d, err_sat, eff_sat):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.summed_errors = 0
        self.reference = 0
        self.err_sat=err_sat
        self.effort_saturation=eff_sat
    
    def change_Ref(self, reference):
        self.reference = reference
        # print("Reference set")
        # print(self.reference)
    
    def get_Ref(self):
        return self.reference
    
    def get_Error(self):
        return self.summed_errors
    
    def proportional_Control(self, measure):
        error = self.reference - measure
        cntrl_val = error * self.K_p
        
        if cntrl_val > 100:
            return 100
        elif cntrl_val < -100:
            return -100
        else:
            return cntrl_val
    
    def pi_Control(self, measure, dt):
        #print(f"Measure : {measure}")
        #print(f"Reference : {self.reference}")
        error = self.reference - measure
        #print(f"error : {error}")
        self.summed_errors += error * (dt/1000000)
        self.summed_errors = self.error_sat(self.summed_errors)
        cntrl_val = ((error * self.K_p)+(self.summed_errors * self.K_i))
        return self.effort_sat(cntrl_val)
        
    def error_sat(self, err):
        if err >= self.err_sat:
            return self.err_sat
        elif err <= -self.err_sat:
            return -self.err_sat
        else:
            return err
            
    def effort_sat(self, eff):
        if eff >= self.effort_saturation:
            return self.effort_saturation
        elif eff <= -self.effort_saturation:
            return -self.effort_saturation
        else:
            return eff
        
    def change_Gains(self, k_p, k_i, k_d):
        self.K_p = k_p
        self.K_i = k_i
        self.K_d = k_d
            

