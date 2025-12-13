## @file observer.py
#  This file contains a class to create a observer object which is compatible with
#  the cotask scheduler to run as a cooperative task. 
#  
#  This task always estimates the state of ROMI given its sensor inputs using a discretized state space model. 
#
#  NOTE: These values have errors and the errors get worse as ROMI runs it is only an estimation.
#
#  IMPORTANT: This observer will reach a null space if the encoder values and yaw value are not possible with each other. 
#             If the encoder positions can't equal the yaw value of the sensors this will fail. This happens on start because the encoders start
#             at zero but the yaw does not necessarily start at 0 as well. The correction for this is in state 0 of the control task where the 
#             encoders are given an offset to make them match whatever the starting yaw is. 
#
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @date   2025-Nov-11 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 

from ulab import numpy as np
import array
## @brief observer class that can be initialized to be run with the cotask scheduler. See run for behavior. 
#
class observer:
## @brief __init__ is the observer object initializer which takes 6 arguments. 
#         The arguments are data shares. 
#  @param left_pos : left_pos is a float share that holds the current position of the left encoder. Used as an input. 
#  @param right_pos : right_pos is a float share that holds the current position of the right encoder. Used as an input.
#  @param yaw : yaw is a float share that holds the IMU reading of the yaw. 
#  @param yaw_velocity : yaw_velocity is the yaw_velocity as measured by the IMU task. 
#  @param left_voltage : left_voltage is the PWM value of the left motor which observer scales to the expected voltage.
#  @param right_voltage : right_voltage is the PWM value of the right motor which observer scales to the expected voltage.
#  @param left_vel : left_vel is the velocity of the left motor. 
#  @param right_vel : right_vel is the velocity of the right motor. 
#  @param total_dist : total_dist is a float share that the observer will fill with how far it thinks ROMI has travelled in mm. 
#
    def __init__(self, left_pos, right_pos, yaw, yaw_velocity, left_voltage, right_voltage, left_vel, right_vel, total_dist):
        self.left_pos = left_pos
        self.right_pos = right_pos
        self.yaw = yaw
        self.yaw_velocity = yaw_velocity
        self.left_voltage = left_voltage
        self.right_voltage = right_voltage
        self.left_vel = left_vel
        self.right_vel = right_vel
        self.total_dist = total_dist
        
        #Allocating space for matrices
        self.x = np.array([[0], [0], [0], [0]])
        self.u = np.array([[0], [0]])
        self.y = np.array([[0], [0], [0], [0]])
        self.u_star = np.array([[0], [0], [0], [0], [0], [0]])

        #Allocating space for the guessed state matrices
        self.x_estimated = np.array([[0], [0], [0], [0]])
        self.u_estimated = np.array([[0], [0]])
        self.y_estimated = np.array([[0], [0], [0], [0]])
        self.u_star_estimated = np.array([[0], [0], [0], [0], [0], [0]])
        
        #Continuous model observer gain --> doesn't work fast enough
        self.L = np.array([[2.2683e+05, 2.2683e+05, 1.2069e-08, -1.9921e+03],
                        [2.2683e+05, 2.2683e+05, 1.2068e-08,  1.9921e+03],
                        [9.5000e+01, 9.5000e+01, -2.0531e-11, 9.8455e-10],
                        [-6.98061e+01, 6.98061e+01, 9.901573e+02, 1.0000]])
        
        #Discretized model A matrix
        self.A_d = np.array([
        [0.1668, 0.1668, -659.9169, -0.0000],
        [0.1668, 0.1668, -659.9169, 0.0000],
        [0.0000, 0.0000, -0.1193,   0.0000],
        [0.0000, 0.0000, 0.0000,   0.0000]
        ])
        
        #Discretized model B matrix 
        self.B_d = np.array([
        [0.4216, 0.0363, 329.9584, 329.9584, 0.0000, -1.9941],
        [0.3635, 0.0422, 329.9584, 329.9584, 0.0000,  1.9941],
        [0.0001, 0.0000, 0.5597,  0.5597,  0.0000, 0.0000],
        [0.0000, 0.0000, -0.0698, 0.0698, 0.9902,  0.0010]
        ])

        #Continuous/Discretized (Doesnt Change) model C matrix
        self.C = np.array([[0, 0, 1, -.141/2], 
                    [0, 0, 1, .141/2], 
                    [0, 0, 0, 1], 
                    [-.035/.141, .035/.141, 0, 0]])
        
        self.state = 0
        self.feedback = 1 #0 = continuous with no feedback, 1 = discrete with feedback 
    
## @brief update_x updates the state vector with real values and the last guess total distance
#
    def update_x(self):
        self.x[0, 0] = self.left_vel.get()/.035
        self.x[1, 0] = self.right_vel.get()/.035
        self.x[2, 0] = self.x_estimated[2,0]
        self.x[3, 0] = self.yaw.get()
        
## @brief update_u updates the input vector with real values assuming full battery voltage
#
    def update_u(self):
        self.u[0, 0] = self.left_voltage.get()*12
        self.u[1, 0] = self.right_voltage.get()*12

## @brief update_y updates the output vector with real values
#
    def update_y(self):
        self.y[0, 0] = self.left_pos.get()
        self.y[1, 0] = self.right_pos.get()
        self.y[2, 0] = self.yaw.get()
        self.y[3, 0] = self.yaw_velocity.get()
## @brief update_u_star updates the discetized combined input and output vectors 
#
    def update_u_star(self):
        self.u_star[0, 0] = self.left_voltage.get()*12
        self.u_star[1, 0] = self.right_voltage.get()*12
        self.u_star[2, 0] = self.left_pos.get()
        self.u_star[3, 0] = self.right_pos.get()
        self.u_star[4, 0] = self.yaw.get()
        self.u_star[5, 0] = self.yaw_velocity.get()

## @brief calc_derivative calculates the derivative state vector using the continuous model with no feedback
#  @param x : x is the [4,1] state vector
#  @param u : u is the [2,1] input vector
#
    def calc_derivative(self,x, u):
        # x exspected as a [4,1]
        # u expected as a [2,1] 
        
        t_l = 0.1
        t_r = 0.1
        r = 0.35
        w = .141
        K_l = 5.81
        K_r = 5.81
        
        A = np.array([[-1/t_l, 0, 0, 0], 
                    [0, -1/t_r, 0, 0], 
                    [r/2, r/2, 0, 0], 
                    [-r/w, r/w, 0, 0]])
        
        B = np.array([[K_l/t_l, 0], 
                    [0, K_r/t_r], 
                    [0, 0], 
                    [0, 0]])

        dx = np.dot(A,x) + np.dot(B,u)

        return dx
## @brief calc_output calculates the output vector using the continuous model with no feedback
#  @param x : x is the [4,1] state vector
#  @param u : u is the [2,1] input vector
#
    def calc_output(self, x, u):
        # x expected as a [4,1]
        # u expected as a [2,1]
        r = 0.35
        w = .141
    
        C = np.array([[0, 0, 1, -w/2], 
                    [0, 0, 1, w/2], 
                    [0, 0, 0, 1], 
                    [-r/w, r/w, 0, 0]])
        
        D = np.array([[0, 0], 
                    [0, 0], 
                    [0, 0], 
                    [0, 0]])
        
        y = np.dot(C,x) + np.dot(D,u)

        return y
## @brief calc_discrete_state calculates the next state vector using the discretized model
#  @param x : x is the [4,1] state vector
#  @param u_star : u_star is the [6,1] combined input and output vector
#
    def calc_discrete_state(self, x, u_star):
        x_next = np.dot(self.A_d,x) + np.dot(self.B_d,u_star)
        return x_next 
    
## @brief calc_discrete_state calculates the current output vector using the discretized model
#  @param x : x is the [4,1] state vector
#
    def calc_discrete_output(self, x):
        y_next = np.dot(self.C, x)
        return y_next
    
## @brief RK4_solver is an implementation of the RK4_solver algorithm that will be used to estimate the state if using a continuous model. Returns next state and current output 
#  @param x : x is the current state vector
#  @param t_step : t_step is the time step between the current state and the estimated state. Should be the time when this task next runs
#  @param u : u is the input vector
#
    def RK4_solver(self, x, t_step, u):

        k1 = self.calc_derivative(x, u)
        y1 = self.calc_output(x, u)

        k2 = self.calc_derivative(x + 0.5*k1*t_step, u)
        y2 = self.calc_output(x + 0.5*k1*t_step, u)

        k3 = self.calc_derivative(x + 0.5*k2*t_step, u)
        y3 = self.calc_output(x + 0.5*k2*t_step, u)

        k4 = self.calc_derivative(x + k3*t_step, u)
        y4 = self.calc_output(x + k3*t_step, u)
        
        x_next = x + 1/6 * (k1 + 2 * k2 + 2 * k3 + k4) * t_step
        y = 1/6 * (y1 + 2 * y2 + 2 * y3 + y4)
    
        return x_next, y
## @brief run is the generator that the cotask scheduler will run since our tasks are objects. All shares it needs are already given to 
#         the task object on initialization. The generator will only run one state at a time on each run of the task.
#  @detail The observer will always update its current states then guess at the next state. The only state vector it does not have direct data for 
#          is total distance travelled so it will always use its estimation for the next state. If feedback is turned off the observer will run the simple
#          state space continuous model that has no feedback. This is not really an observer it is just an estimator. If feedback is turned on the observer will 
#          run the discretized model of the system. Typically the total distance travelled is useful but the others are not. The value is predicts is also pretty consistent
#          each run, but it is always negative and off by a factor of about 2.1 so we simply scalled it by -2.1 so that the value is more useful. It is in mm. 
    def run(self):
        while (True):
            # Run the observer 
            if self.state == 0:
                self.update_x()
                self.update_y()
                self.update_u()
                self.update_u_star()
                if self.feedback == 0:
                #Continuous
                    self.x_estimated, self.y_estimated = self.RK4_solver(self.x, .02, self.u)
                    print(self.x_estimated[2, 0])
                    #print(self.yaw.get())
                else: 
                #Discretized
                    self.x_estimated = self.calc_discrete_state(self.x, self.u_star)
                    self.y_estimated = self.calc_discrete_output(self.x)
                    self.total_dist.put(self.x_estimated[2,0]*-2.1)
                    #print(f"\rLeft Velocity: {self.x_estimated[0,0]}  \n")
                    #print(f"Right Velocity: {self.x_estimated[1,0]}  \n")
                    #print(f"Total Distance: {self.x_estimated[2,0]*(-2.1)}  \n")
                    #print(f"Yaw: {self.x[3,0]}  \n")
                    #print("\033[4A")s
            else:
                raise ValueError("Not that many states")
            yield self.state

    

    
