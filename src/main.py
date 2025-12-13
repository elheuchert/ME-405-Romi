## @file main.py
#  This file contains creates all shares, queues, hardware driver constants, controllers, and
#  task objects for the ROMI program. It also starts the scheduler from cotask. 
#
#  In order main imports all files, sets defines, creates all shares, creates all queues,
#  creates all hardware driver objects, creates all task class objects, creates all tasks, 
#  adds all tasks to the scheduler, and runs the scheduler. 
#
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @date   2025-Nov-10 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made

#---------------------------------------------------------------------------------
# Import All Program Files                                                       #
#---------------------------------------------------------------------------------
import task_share
import cotask
import os
from motor import Motor
from encoder import Encoder
from pyb import Timer, Pin, UART, ExtInt, I2C
from data_collector import data_collector
from ui import ui
from motor_encoder_left_class import motor_encoder_left_class
from motor_encoder_right_class import motor_encoder_right_class
from controller import Controller
from control_task import control_task
from line_sensor import Line_Sensor
from ir_sensor import IR_Sensor
from line_task import line_task
from imu import IMU
from imu_task import imu_task
from observer import observer
from pathing_plan_task import pathing_plan
from bump_task import Bump_Task
from bump__sensor import Bump_Sensor
import machine


# def RC_Input(line):
#     machine.soft_reset()

# User_Button = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, RC_Input)

# Controller Saturation Defines 
ERR_SAT_LEFT=50.0
EFF_SAT_LEFT=85.0

ERR_SAT_RIGHT=50.0
EFF_SAT_RIGHT=85.0

ERR_SAT_LINE=1
EFF_SAT_LINE=40

ERR_SAT_YAW=.8
EFF_SAT_YAW=100
#---------------------------------------------------------------------------------
# Create All Shares                                                              #
#---------------------------------------------------------------------------------

m_state_l = task_share.Share('b', thread_protect = False, name = "m_state_l")   # 1=waiting but enabled
m_state_r = task_share.Share('b', thread_protect = False, name = "m_state_r")   # 1=waiting but enabled

PWM_l = task_share.Share('f', thread_protect = False, name = "PWM_l")
PWM_r = task_share.Share('f', thread_protect = False, name = "PWM_r")

position_r = task_share.Share('f', thread_protect = False, name = "position_r")
velocity_r = task_share.Share('f', thread_protect = False, name = "velocity_r")

position_l = task_share.Share('f', thread_protect = False, name = "position_l")
velocity_l = task_share.Share('f', thread_protect = False, name = "velocity_l")

testing_flg = task_share.Share('b', thread_protect = False, name = "testing_flg")
delay = task_share.Share('b', thread_protect = False, name = "delay")

fwd_ref = task_share.Share('l', thread_protect = False, name = "fwd_ref")
arc_ref = task_share.Share('l', thread_protect = False, name = "arc_ref")
piv_ref = task_share.Share('f', thread_protect = False, name = "piv_ref")
c_state = task_share.Share('b', thread_protect = False, name = "c_state")

l_state = task_share.Share('b', thread_protect = False, name = "l_state")
ready_Black = task_share.Share('b', thread_protect = False, name = "ready_Black")
ready_White = task_share.Share('b', thread_protect = False, name = "ready_White")

need_Calibrate = task_share.Share('b', thread_protect = False, name = "need calibrate")
centroid = task_share.Share('f', thread_protect = False, name ="centroid")

automatic_mode = task_share.Share('b', thread_protect = False, name = "automatic_mode" )

imu_flg = task_share.Share('b', thread_protect= False, name = "imu_flg")

yaw = task_share.Share('f', thread_protect = False, name = "yaw")
yaw_velocity = task_share.Share('f', thread_protect = False, name = "yaw_velocity")

total_dist = task_share.Share("f", thread_protect = False, name = "S")

centroid_goal = task_share.Share("f", thread_protect = False, name = "Goal Centroid")
yaw_goal = task_share.Share("f", thread_protect = False, name = "Yaw Goal")

bump_on_off = task_share.Share("b", thread_protect = False, name = "Bump Flag")
bump_flg = task_share.Share("b", thread_protect = False, name = "Bump Flag")

#---------------------------------------------------------------------------------
# Create All Queues                                                              #
#---------------------------------------------------------------------------------
velocity = task_share.Queue('f', 50, thread_protect = False, overwrite = False, name = "velocity")
velocity2 = task_share.Queue('f', 50, thread_protect = False, overwrite = False, name = "velocity2")

position = task_share.Queue('f', 50, thread_protect = False, overwrite = False, name = "position")

times = task_share.Queue('l', 50, thread_protect = False, overwrite = False, name = "times")

#---------------------------------------------------------------------------------
# Create All Hardware Driver Objects                                             #
#---------------------------------------------------------------------------------
tim1 = Timer(1, prescaler=0, period=65535)
tim3 = Timer(3, prescaler=0, period=65535)
tim4 = Timer(4, freq=20000)

my_motor_Left = Motor(Pin.cpu.B7, Pin.cpu.C13, Pin.cpu.C14, tim4, 1)
encoder_Right = Encoder(tim3, Pin.cpu.C6, Pin.cpu.C7)

my_motor_Right = Motor(Pin.cpu.B6, Pin.cpu.B10, Pin.cpu.A10, tim4, 2)
encoder_Left = Encoder(tim1, Pin.cpu.A8, Pin.cpu.A9)

bump_sensor_right = Bump_Sensor(Pin.cpu.B13, Pin.cpu.B3, Pin.cpu.B5)
bump_sensor_left = Bump_Sensor(Pin.cpu.B11, Pin.cpu.B15, Pin.cpu.B14)

ir1 = IR_Sensor(Pin.cpu.A6)
ir2 = IR_Sensor(Pin.cpu.A7)
ir3 = IR_Sensor(Pin.cpu.C4)
ir4 = IR_Sensor(Pin.cpu.A0)
ir5 = IR_Sensor(Pin.cpu.A1)
ir6 = IR_Sensor(Pin.cpu.C4)
ir7 = IR_Sensor(Pin.cpu.B0)
ir8 = IR_Sensor(Pin.cpu.C0)
ir9 = IR_Sensor(Pin.cpu.C1)
ir10 = IR_Sensor(Pin.cpu.C2)
ir11 = IR_Sensor(Pin.cpu.C3)
ir12 = IR_Sensor(Pin.cpu.C5)
ir13 = IR_Sensor(Pin.cpu.B1)

line_sensor_obj = Line_Sensor(ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8, ir9, ir10, ir11, ir12, ir13)

uart_obj = UART(5, 115200)
uart_obj.init(115200, bits=8, parity = None, stop = 1, timeout = 50)

# create i2c and imu objects
i2c_obj = I2C(1,I2C.CONTROLLER,baudrate = 400000)
imu_obj = IMU(i2c_obj)

#---------------------------------------------------------------------------------
# Create All Controller Objects                                                  #
#---------------------------------------------------------------------------------
controller_obj_left = Controller(.2,1.95,1, ERR_SAT_LEFT, EFF_SAT_LEFT)
controller_obj_right = Controller(.2,1.95,1, ERR_SAT_RIGHT, EFF_SAT_RIGHT)
controller_obj_line = Controller(150, 10, 1, ERR_SAT_LINE, EFF_SAT_LINE)
controller_obj_yaw = Controller(.8,.3,1, ERR_SAT_YAW, EFF_SAT_YAW)

#---------------------------------------------------------------------------------
# Create Task Class Objects with Shares.                                         #
#---------------------------------------------------------------------------------
motor_Left_class = motor_encoder_left_class(my_motor_Left, encoder_Left, m_state_l, position_l, velocity_l, times, PWM_l, delay)
motor_Right_class = motor_encoder_right_class(my_motor_Right, encoder_Right, m_state_r, position_r, velocity_r, times, PWM_r, delay)
ui_obj = ui(testing_flg, m_state_l, m_state_r, position, velocity, times, PWM_l, PWM_r, delay, uart_obj, fwd_ref, arc_ref, piv_ref, c_state, velocity2, need_Calibrate, ready_Black, ready_White, automatic_mode, imu_flg)
data_collector_obj = data_collector(testing_flg, position, velocity, times, position_l, velocity_l, position_r, velocity_r, velocity2)
control_task_obj = control_task(fwd_ref, arc_ref, piv_ref, c_state, controller_obj_left, controller_obj_right, position_l, velocity_l, position_r, velocity_r, PWM_l, PWM_r, encoder_Right, encoder_Left, m_state_l, m_state_r, centroid, controller_obj_line, automatic_mode, line_sensor_obj, need_Calibrate, yaw, centroid_goal, yaw_goal, controller_obj_yaw)
line_task_obj = line_task(line_sensor_obj, l_state, need_Calibrate, ready_Black, ready_White,centroid)
imu_task_obj = imu_task(imu_obj, imu_flg, yaw, yaw_velocity)
observer_obj = observer(position_l, position_r, yaw, yaw_velocity, PWM_l, PWM_r, velocity_l, velocity_r, total_dist)
pathing_obj = pathing_plan(total_dist, automatic_mode, c_state, fwd_ref, piv_ref, yaw, centroid_goal, yaw_goal, bump_on_off, bump_flg)
bump_obj = Bump_Task(c_state, bump_sensor_right, bump_sensor_left, bump_on_off, bump_flg)

#---------------------------------------------------------------------------------
# Create All Tasks with Class Object Generator Function                          #
#---------------------------------------------------------------------------------
task1 = cotask.Task(data_collector_obj.run, name="data_collector", priority=2, period=18, profile=True, trace=False, shares=())
task2 = cotask.Task(motor_Left_class.run, name="motor_encoder_left", priority=7, period=13, profile=True, trace=False, shares=())
task3 = cotask.Task(motor_Right_class.run, name="motor_encoder_right", priority=7, period=13, profile=True, trace=False, shares=())
task4 = cotask.Task(ui_obj.run, name="ui", priority=1, period=1, profile=True, trace=False, shares=())
task5 = cotask.Task(control_task_obj.run, name="control_task", priority=6, period=15, profile = True, trace=False, shares=())
task6 = cotask.Task(line_task_obj.run, name="line_task", priority = 5, period = 22, profile = True, trace = False, shares=())
task7 = cotask.Task(imu_task_obj.run, name = "imu_task", priority = 6, period = 20, profile= True, trace= False, shares = ())
task8 = cotask.Task(observer_obj.run, name = "observer", priority = 4, period = 20, profile = True, trace = False, shares = ())
task9 = cotask.Task(pathing_obj.run, name= "pathing", priority = 3, period = 30, profile = True, trace = False, shares = ())
task10 = cotask.Task(bump_obj.run, name = "bump task", priority = 2, period = 50, profile = True, trace = False, shares=())

#---------------------------------------------------------------------------------
# Add Tasks to Scheduler                                                         #
#---------------------------------------------------------------------------------
cotask.task_list.append(task1)
cotask.task_list.append(task2)
cotask.task_list.append(task3)
cotask.task_list.append(task4)
cotask.task_list.append(task5)
cotask.task_list.append(task6)
cotask.task_list.append(task7)
cotask.task_list.append(task8)
cotask.task_list.append(task9)
cotask.task_list.append(task10)

#---------------------------------------------------------------------------------
# Run the Scheduler until Interrupt                                              #
#---------------------------------------------------------------------------------
try:
    while True:
        cotask.task_list.pri_sched()

except KeyboardInterrupt:
    # Task Profile once hit Ctr + C
    print("\nProfiler results:")
    print(cotask.task_list)
    raise