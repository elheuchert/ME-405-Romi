from serial import Serial
from matplotlib import pyplot
from time import sleep 
from datetime import datetime
import csv

NUMBERS_OF_TESTS = 10
VALUES_SENT = 50
period=13/1000
motor_gains_l = []
time_constnats_l = []
motor_gains_r = []
time_constnats_r = []
test_end=1
arc_lim = 100  #arc radius in mm
velo_lim = 300 #velocity in mm/s
angular_lim = 10 #angular velocity in rad/s
test_done=True

# Reads the header line from the data
def saveHeaders():
    thead, dhead = ser.readline().decode().strip().split(",")
    #print(thead, dhead)
    return thead, dhead

# Saves the data sent from a single test
def saveData():
    t,d = map(float, ser.readline().decode().strip().split(","))
    return t, d

# Plots passed data
def plotData(times, data, thead, dhead, legend):
    pyplot.plot(times,data, label = legend)
    pyplot.xlabel(thead)
    pyplot.ylabel(dhead)

def average(data_list):
    sum = 0
    for i in data_list:
        sum += i
    return sum/len(data_list)

def timeConstant(data):
    steady_state = data[-1]
    tau_val= 0.63*steady_state
    for i, value in enumerate(data):
        if value > tau_val:
            tau= i*period
            return tau
    return
    
def motorGain(test_num, data):
    steady_state = data[-1]
    if test_num <= NUMBERS_OF_TESTS:
        motor_gain = steady_state/(test_num/10)
    elif test_num <= 2* NUMBERS_OF_TESTS:
        motor_gain = steady_state/((test_num-NUMBERS_OF_TESTS)/10)
    return motor_gain

def get_test_plots():
    
    values_read = 0
    times = []
    data = []
    times2 = []
    data2 = []
    while not ser.in_waiting: continue
    thead, dhead = saveHeaders()
    while True:
        if values_read < VALUES_SENT:    
            if ser.in_waiting:
                t, d = saveData()
                times.append(t)
                data.append(d)
                values_read+=1
        elif values_read>=VALUES_SENT:
            if ser.in_waiting:
                t2, d2 = saveData()
                times2.append(t2)
                data2.append(d2)
                values_read+=1
        if values_read==2*VALUES_SENT:
            #print('plotting values')
            pyplot.figure()
            # print(times, data)
            # print(times2, data2)
            plotData(times, data, thead, dhead, "Left")
            plotData(times2, data2, thead, dhead, "Right")
            timestamp=datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            pyplot.title(f"Velocity_{timestamp}")
            pyplot.legend()
            pyplot.xlim(0,VALUES_SENT)
            pyplot.ylim(0,400)
            pyplot.savefig(f'Velocity_{timestamp}.svg')
            pyplot.clf()
            break
    
    

    # while ser.in_waiting:
        
    #     print(ser.readline().decode().strip())
    
    # ser.write(str(0).encode())
    # while not ser.in_waiting: continue
    # while ser.in_waiting:

    #     print(ser.readline().decode().strip())
    
    # print(f'{values_read} values_read')
    # if values_read == 0:
    #     thead, dhead = saveHeaders()
    
    
    # if values_read < VALUES_SENT:
    #     times = []
    #     data = []
    #     times, data = saveData(times, data)  
        
    # elif values_read < 2* VALUES_SENT:
    #     times2 = []
    #     data2 = []
    #     times2, data2 = saveData(times, data)  
        
    # elif values_read == 2* VALUES_SENT:
    #     plotData(times, data, thead, dhead, "Right")
    #     plotData(times2, data2, thead, dhead, "Left")
    #     timestamp=datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    #     pyplot.title("Velocity")
    #     pyplot.legend()
    #     pyplot.savefig(f'Velocity_{timestamp}.svg')
    #     pyplot.clf()
    # values_read += 1
    return

def run_tests():


        while True:
                
            user_input = input("Send a command: s=full motor step response, t=test, 0=stop romi, 1=straight, 2=Arc turn, 3= pivot, q=quit: ")

            if user_input=='q':
                print('exiting test loop')
                ser.write(str(0).encode())
                break
            
            if user_input =='s':
                ser.write(str(user_input).encode())
                continue
            if user_input =='t':
                ser.write(str(user_input).encode())
                while True:
                    test_input=input("Send a command: 0=stop romi, 1=straight, 2=Arc turn, 3= pivot: ")
                    try:
                        test_input=int(test_input)
                    except ValueError:
                        #print("please select: 0=stop romi, 1=straight, 2=Arc turn, 3= pivot")
                        continue
                    if test_input==1:
                        ser.write(str(test_input).encode())
                        while True:
                            j=input(f"please select a velocity in mm/s from {-velo_lim} to {velo_lim}: ")
                            try:
                                j=int(j)
                                if j>=-velo_lim and j<=velo_lim and j!=0:
                                    
                                    ser.write(str(j).encode())
                                    
                                    get_test_plots()
                                    break  
                                #else:
                                    #print(f"please select a non zeroint from {-velo_lim} to {velo_lim}")    
                            except ValueError:
                                #print(f"please select a non zeroint from {-velo_lim} to {velo_lim}")
                                continue
                        break    
                    elif test_input==2:
                        ser.write(str(test_input).encode())
                        while True:
                            j = input(f"Please enter a non zero integer arc radius in mm from {-arc_lim} to {arc_lim}: ")
                            try:
                                j = int(j)
                                if j>=-arc_lim and j<= arc_lim and j!=0:
                                    ser.write(str(j).encode())
                                    get_test_plots()
                                    break
                                # else:
                                #     print(f"Please enter a non zero integer from {-arc_lim} to {arc_lim}")
                            except ValueError:
                                #print(f"Please select a non zero integer from {-arc_lim} to {arc_lim}")
                                continue
                        break
                    elif test_input==3:
                        ser.write(str(test_input).encode())
                        while True:
                            j = input(f"Please enter an integer arc radius in mm from {-angular_lim} to {angular_lim}: ")
                            try:
                                j = int(j)
                                if j>=-angular_lim and j<= angular_lim and j!=0: 
                                    ser.write(str(j).encode())
                                    get_test_plots()
                                    break
                                # else:
                                #     print(f"Please enter a non zero integer from {-angular_lim} to {angular_lim}")
                            except ValueError:
                                #print(f"Please select a non zero integer from {-angular_lim} to {angular_lim}")
                                continue
                        break
                    elif test_input==0:
                        ser.write(str(test_input).encode())
                        continue
                continue        
            try:
                user_input= int(user_input) 
            except ValueError:
                #print("please select: s=full motor step response, t=test, 0=stop romi, 1=straight, 2=Arc turn, 3= pivot, q=quit\n1")
                continue
            if user_input==1:
                ser.write(str(user_input).encode())
                while True:
                    n=input(f"please select a velocity in mm/s from {-velo_lim} to {velo_lim}: ")
                    try:
                        n=int(n)
                        if n>=-velo_lim and n<=velo_lim and n!=0:
                            ser.write(str(n).encode())
                            break   
                        # else:
                        #     print(f"Please enter a non zero integer from {-velo_lim} to {velo_lim}")
                    except ValueError:
                        #print(f"please select a non zeroint from {-velo_lim} to {velo_lim}")
                        continue
                continue
            elif user_input==2:
                ser.write(str(user_input).encode())
                while True:
                    n = input(f"Please enter a non zero integer arc radius in mm from {-arc_lim} to {arc_lim}: ")
                    try:
                        n = int(n)
                        if n>=-arc_lim and n<= arc_lim and n!=0:
                            ser.write(str(n).encode())
                            break
                        # else:
                        #     print(f"Please enter a non zero integer from {-arc_lim} to {arc_lim}")
                    except ValueError:
                        #print(f"Please select a non zero integer from {-arc_lim} to {arc_lim}")
                        continue
                continue
            elif user_input==3:
                ser.write(str(user_input).encode())
                while True:
                    n = input(f"Please enter an integer arc radius in mm from {-angular_lim} to {angular_lim}: ")
                    try:
                        n = int(n)
                        if n>=-angular_lim and n<= angular_lim and n!=0: 
                            ser.write(str(n).encode())
                            break
                        # else:
                        #     print(f"Please enter a non zero integer from {-angular_lim} to {angular_lim}")
                    except ValueError:
                        #print(f"Please select a non zero integer from {-angular_lim} to {angular_lim}")
                        continue
                continue
            elif user_input==0:
                ser.write(str(user_input).encode())
                continue
        return

def line_test():
    
            print ("Opening serial port")
            sleep(0.5)

            print ("Flushing serial port")
            ser.reset_input_buffer()
            
            while True:

                black_test=input("select b when on black: ")
                
                if black_test=="b":
                    ser.write(str(1).encode())
                    print("Black test done")

                    white_test=input("select w when on white: ")
                    if white_test=="w":
                        ser.write(str(1).encode())
                        print("White test done: ")
                
                        break
                    else:
                        continue
                else:
                    
                    continue
            return "Black and white test complete"

while True:
    with Serial(port ="COM4", baudrate = 115200, timeout=1) as ser:
        user_input=input("press c for calibrate, t for step test, q to quit, g to go, i for imu data")
        if user_input=="c":
            line_test()
            continue
        elif user_input=='t':
            
            run_tests()
        elif user_input=='g':
            ser.write(str('g').encode())
            continue
        elif user_input=='q':
            ser.write(str(0).encode())
        elif user_input=="i":
            ser.write(str('i').encode())
    
    
    
    # test_num = 0
    # values_read = 0
    # open("output_LP.csv", "w").close()
    # open("output_RP.csv", "w").close()
    # open("output_LV.csv", "w").close()
    # open("output_RV.csv", "w").close()

    # # Forever loop
    
        
    # while 1:   
    #     # Block until something comes from bluetooth
    #     while not ser.in_waiting: continue
    #     test_num += 1
    #     # sleep(.5)

    #     # First ten tests
    #     if test_num == 1:
    #         thead, dhead = saveHeaders()
    #     if test_num <= NUMBERS_OF_TESTS:
    #         if values_read < 20:
    #             times = []
    #             data = []
                
    #             times, data = saveData(times, data)
    #             time_constnats_l.append(timeConstant(data))
    #             motor_gains_l.append(motorGain(test_num, data))
                
                
    #             with open("output_LV.csv", "a", newline="") as f:
    #                     writer = csv.writer(f)
    #                     writer.writerow(["Times", "Data"])   
    #                     for t, d in zip(times, data):
    #                         writer.writerow([t, d])
                
    #             plotData(times, data, thead, dhead)
    #             if test_num == NUMBERS_OF_TESTS:
    #                 pyplot.title("Left Velocity")
    #                 pyplot.savefig("Velocity_Left.svg")
    #                 pyplot.clf()
    #                 print(f" average motor gain left: {average(motor_gains_l)}")
    #                 print(f" average time const left: {average(time_constnats_l)}")
    #             values_read += 1
    #         else:
    #             values_read = 0
    #     # Second ten tests
    #     elif test_num <= 2*NUMBERS_OF_TESTS:
    #         if values_read < VALUES_SENT:
    #             times = []
    #             data = []
    #             times, data = saveData(times, data)
                
    #             time_constnats_r.append(timeConstant(data))
    #             motor_gains_r.append(motorGain(test_num, data))

    #             with open("output_RV.csv", "a", newline="") as f:
    #                     writer = csv.writer(f)
    #                     writer.writerow(["Times", "Data"])   
    #                     for t, d in zip(times, data):
    #                         writer.writerow([t, d])
    #             plotData(times, data, thead, dhead)

    #             if test_num == 2 * NUMBERS_OF_TESTS:
    #                 pyplot.title("Right Velocity")
    #                 pyplot.savefig("Velocity_Right.svg")
    #                 pyplot.clf()
                    
    #                 print(f"average motor gain right:{average(motor_gains_r)}")
    #                 print(motor_gains_r)
    #                 print(f"average time const right:{average(time_constnats_r)}")
    #             values_read += 1
    #         else:
    #             values_read = 0
    #     # Third ten tests
    #     elif test_num == (2 * NUMBERS_OF_TESTS) + 1:
    #         print("About to save them headers")
    #         thead, dhead = saveHeaders()
    #         print("headers saved")

    #     elif test_num <= 3 * NUMBERS_OF_TESTS:
    #         if values_read < VALUES_SENT:
    #             times = []
    #             data = []
    #             times, data = saveData(times, data)
    #             with open("output_LP.csv", "a", newline="") as f:
    #                     writer = csv.writer(f)
    #                     writer.writerow(["Times", "Data"])   
    #                     for t, d in zip(times, data):
    #                         writer.writerow([t, d])
    #             plotData(times, data, thead, dhead)
    #             if test_num == 3 * NUMBERS_OF_TESTS:
    #                 pyplot.title("Left Position")
    #                 pyplot.savefig("Position_Left.svg")
    #                 pyplot.clf()
                    
    #             values_read += 1
    #         else:
    #             values_read = 0
    #     # Fourth ten tests 
    #     elif test_num <= 4 * NUMBERS_OF_TESTS:
    #         if values_read < VALUES_SENT:
    #             times = []
    #             data = []
    #             times, data = saveData(times, data)
    #             with open("output_RP.csv", "a", newline="") as f:
    #                     writer = csv.writer(f)
    #                     # writer.writerow(["Times", "Data"])   
    #                     for t, d in zip(times, data):
    #                         writer.writerow([t, d])
    #             plotData(times, data, thead, dhead)
    #             if test_num == 4 * NUMBERS_OF_TESTS:
    #                 pyplot.title("Right Position")
    #                 pyplot.savefig("Position_Right.svg")
    #                 pyplot.clf()
                    
    #             values_read += 1
    #         else:
    #             values_read = 0