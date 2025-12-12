from ulab import numpy as np
import array
## @brief add description here
class observer:

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
        
        self.x = np.array([[0], [0], [0], [0]])
        self.u = np.array([[0], [0]])
        self.y = np.array([[0], [0], [0], [0]])
        self.u_star = np.array([[0], [0], [0], [0], [0], [0]])

        self.x_estimated = np.array([[0], [0], [0], [0]])
        self.u_estimated = np.array([[0], [0]])
        self.y_estimated = np.array([[0], [0], [0], [0]])
        self.u_star_estimated = np.array([[0], [0], [0], [0], [0], [0]])
        
        self.L = np.array([[2.2683e+05, 2.2683e+05, 1.2069e-08, -1.9921e+03],
                        [2.2683e+05, 2.2683e+05, 1.2068e-08,  1.9921e+03],
                        [9.5000e+01, 9.5000e+01, -2.0531e-11, 9.8455e-10],
                        [-6.98061e+01, 6.98061e+01, 9.901573e+02, 1.0000]])
        
        self.A_d = np.array([
        [0.1668, 0.1668, -659.9169, -0.0000],
        [0.1668, 0.1668, -659.9169, 0.0000],
        [0.0000, 0.0000, -0.1193,   0.0000],
        [0.0000, 0.0000, 0.0000,   0.0000]
        ])
        
        self.B_d = np.array([
        [0.4216, 0.0363, 329.9584, 329.9584, 0.0000, -1.9941],
        [0.3635, 0.0422, 329.9584, 329.9584, 0.0000,  1.9941],
        [0.0001, 0.0000, 0.5597,  0.5597,  0.0000, 0.0000],
        [0.0000, 0.0000, -0.0698, 0.0698, 0.9902,  0.0010]
        ])

        self.C = np.array([[0, 0, 1, -.141/2], 
                    [0, 0, 1, .141/2], 
                    [0, 0, 0, 1], 
                    [-.035/.141, .035/.141, 0, 0]])
                
        self.state = 0
        self.feedback = 1
    
    def update_x(self):
        self.x[0, 0] = self.left_vel.get()/.035
        self.x[1, 0] = self.right_vel.get()/.035
        self.x[2, 0] = self.x_estimated[2,0]
        self.x[3, 0] = self.yaw.get()
        

    def update_u(self):
        self.u[0, 0] = self.left_voltage.get()*12
        self.u[1, 0] = self.right_voltage.get()*12

    def update_y(self):
        self.y[0, 0] = self.left_pos.get()
        self.y[1, 0] = self.right_pos.get()
        self.y[2, 0] = self.yaw.get()
        self.y[3, 0] = self.yaw_velocity.get()
    
    def update_u_star(self):
        self.u_star[0, 0] = self.left_voltage.get()*12
        self.u_star[1, 0] = self.right_voltage.get()*12
        self.u_star[2, 0] = self.left_pos.get()
        self.u_star[3, 0] = self.right_pos.get()
        self.u_star[4, 0] = self.yaw.get()
        self.u_star[5, 0] = self.yaw_velocity.get()

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
    
    def calc_discrete_state(self, x, u_star):
        x_next = np.dot(self.A_d,x) + np.dot(self.B_d,u_star)
        return x_next 

    def calc_discrete_output(self, x):
        y_next = np.dot(self.C, x)
        return y_next
    
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
    
    def run(self):
        while (True):
            # Run the observer 
            if self.state == 0:
                self.update_x()
                self.update_y()
                self.update_u()
                self.update_u_star()
                if self.feedback == 0:
                    self.x_estimated, self.y_estimated = self.RK4_solver(self.x, .02, self.u)
                    print(self.x_estimated[2, 0])
                    #print(self.yaw.get())
                else: 
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

    

    
