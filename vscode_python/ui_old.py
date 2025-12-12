from pyb import USB_VCP

ser=USB_VSP()
side = 0
saved = []

def ui():
    # hub
    if state == 0: 
        if ser.any():
            char_in = ser.read(1).decode()
            if char_in == "l":
                side = 0 #0=left
                state = 1
            elif char_in == "r":
                side = 1 #1=right
                state = 1
            else:
                print("Please enter r for right motor, l for left motor")
        yield 0
    elif state == 1:
        if ser.any():
            char_in = ser.read(1).decode()
            if char_in == "+":
                if side == 0:
                    state = 2
                else:
                    state = 4
            elif char_in == "-":
                if side == 0:
                    state = 3
                else:
                    state = 5
        yield 1

    elif state ==2:      # positive left
        if ser.any(): # wait for any character 
                char_in = ser.read(1).decode() 
        # 10% Duty Cycle
        if char_in == "1": 
            PWM_l.put(10)
            testing_flg = 1
         # 20% Duty Cycle
        elif char_in == "2":
            PWM_l.put(20)
            testing_flg = 1    
        # 30% Duty Cycle
        elif char_in == "3":
            PWM_l.put(30)
            testing_flg = 1
        # 40% Duty Cycle
        elif char_in == "4":
            PWM_l.put(40)
            testing_flg = 1   
        # 50% Duty Cycle
        elif char_in == "5":
            PWM_l.put(50)
            testing_flg = 1    
        # 60% Duty Cycle
        elif char_in == "6":
            PWM_l.put(60)
            testing_flg = 1     
        # 70% Duty Cycle
        elif char_in == "7":
            PWM_l.put(70)
            testing_flg = 1    
        # 80% Duty Cycle
        elif char_in == "8":
            PWM_l.put(80)
            testing_flg = 1    
        # 90% Duty Cycle
        elif char_in == "9":
            PWM_l.put(90)
            testing_flg = 1                
        # 100% Duty Cycle
        elif char_in == "0":
            PWM_l.put(100)  
            testing_flg = 1
        yield 2  
                
    elif state == 3: # positive right 
        if ser.any(): # wait for any character 
                char_in = ser.read(1).decode() 
        # 10% Duty Cycle
        if char_in == "1":     
            PWM_r.put(10)
            testing_flg = 1
         # 20% Duty Cycle
        elif char_in == "2":
            PWM_r.put(20)
            testing_flg = 1   
        # 30% Duty Cycle
        elif char_in == "3":
            PWM_r.put(30)
            testing_flg = 1
        # 40% Duty Cycle
        elif char_in == "4":
            PWM_r.put(40)
            testing_flg = 1   
        # 50% Duty Cycle
        elif char_in == "5":
            PWM_r.put(50)
            testing_flg = 1    
        # 60% Duty Cycle
        elif char_in == "6":
            PWM_r.put(60)
            testing_flg = 1      
        # 70% Duty Cycle
        elif char_in == "7":
            PWM_r.put(70)
            testing_flg = 1    
        # 80% Duty Cycle
        elif char_in == "8":
            PWM_r.put(80)
            testing_flg = 1    
        # 90% Duty Cycle
        elif char_in == "9":
            PWM_r.put(90)
            testing_flg = 1                
        # 100% Duty Cycle
        elif char_in == "0":
            PWM_r.put(100) 
            testing_flg = 1
        yield 3

    elif state == 4: # negative left
        if ser.any(): # wait for any character 
                char_in = ser.read(1).decode() 
        # 10% Duty Cycle
        if char_in == "1": 
            PWM_l.put(-10)
            testing_flg = 1
         # 20% Duty Cycle
        elif char_in == "2":
            PWM_l.put(-20)
            testing_flg = 1    
        # 30% Duty Cycle
        elif char_in == "3":
            PWM_l.put(-30)
            testing_flg = 1
        # 40% Duty Cycle
        elif char_in == "4":
            PWM_l.put(-40)
            testing_flg = 1   
        # 50% Duty Cycle
        elif char_in == "5":
            PWM_l.put(-50)
            testing_flg = 1    
        # 60% Duty Cycle
        elif char_in == "6":
            PWM_l.put(-60)
            testing_flg = 1      
        # 70% Duty Cycle
        elif char_in == "7":
            PWM_l.put(-70)
            testing_flg = 1    
        # 80% Duty Cycle
        elif char_in == "8":
            PWM_l.put(-80)
            testing_flg = 1   
        # 90% Duty Cycle
        elif char_in == "9":
            PWM_l.put(-90)
            testing_flg = 1               
        # 100% Duty Cycle
        elif char_in == "0":
            PWM_l.put(-100)  
            testing_flg = 1
        yield 4  
    elif state == 5: # negative right   
        if ser.any(): # wait for any character 
                char_in = ser.read(1).decode() 
        # 10% Duty Cycle
        if char_in == "1":    
            PWM_r.put(-10)
            testing_flg = 1
         # 20% Duty Cycle
        elif char_in == "2":
            PWM_r.put(-20)
            testing_flg = 1    
        # 30% Duty Cycle
        elif char_in == "3":
            PWM_r.put(-30)
            testing_flg = 1
        # 40% Duty Cycle
        elif char_in == "4":
            PWM_r.put(-40)
            testing_flg = 1  
        # 50% Duty Cycle
        elif char_in == "5":
            PWM_r.put(-50)
            testing_flg = 1    
        # 60% Duty Cycle
        elif char_in == "6":
            PWM_r.put(-60)
            testing_flg = 1      
        # 70% Duty Cycle
        elif char_in == "7":
            PWM_r.put(-70)
            testing_flg = 1    
        # 80% Duty Cycle
        elif char_in == "8":
            PWM_r.put(-80)
            testing_flg = 1    
        # 90% Duty Cycle
        elif char_in == "9":
            PWM_r.put(-90)
            testing_flg = 1               
        # 100% Duty Cycle
        elif char_in == "0":
            PWM_r.put(-100) 
            testing_flg = 1
        yield 5  
    elif state == 6:
        if testing_flag == 0
            p
    yield state
