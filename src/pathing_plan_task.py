## @file pathing_plan_task.py
#  This file contains a class to create a pathing_plan_task object which is compatible with
#  the cotask scheduler to run as a cooperative task. 
#
#  The pathing_plan object uses a finite state machine design to dictate the motion of
#  the Pololu ROMI kit used in ME 405, Fall 2025 to follow the term project course. The task
#  sets share flags and goals to order how control task moves the ROMI to reach the 6 checkpoints 
#  and the two bonus cups on the course. A long list of defines are used at the beginning to make tuning
#  path simpler. The state names are used to make adding new states inbetween states easier so that the file
#  reads in the order of how ROMI will move. The goal defines are what are tuned for the course. When moving forward
#  the task uses the total distance travelled as found by the observer to know when it needs to change states. 
#  When turning, the task waits for the yaw from the IMU to be within an acceptable range to change states. 
#
#  NOTE: The intention of the angle goals was to be done in a global sense where 0 is the direction ROMI faces 
#        at the start. However, the IMU has a lot of drift as ROMI goes through the course so these values are found
#        experimentally and start to lose meaning when read in the code until you see how ROMI actually moves. 
#  NOTE: Yaw drift is inconsistent so this path is not consistent do not run ROMI unsupervised. 
# 
#  @author Alex Power, Lucas Heuchert, Erik Heuchert
#  @author ChatGPT5 was used for debugging purposes but not to generate this code
#  @date   2025-Nov-10 Approximate date of creation of file
#  @date   2025-Dec-12 Final alterations made
# 
#Yaw Goal Defines
PI = 3.1415
YAWGOAL1 = (82)*(PI/180) #Fork angle
YAWGOAL2 = (329)*(PI/180)#CP1 to CP2
YAWGOAL3 = (90)*(PI/180)#CP2 to Cup 
YAWGOAL4 = (166)*(PI/180)#CP3 to CP4
YAWGOAL5 = (163.5)*(PI/180)#CP4 to garage
YAWGOAL6 = (255)*(PI/180)#Garage to CP5
YAWGOAL7 = (345)*(PI/180)#Wall to Cup
YAWGOAL8 = (240)*(PI/180)#Cup to Finish

#State Names in Order
LINE_FOLLOW_STATE_0 = 0
CENTROID_OFFSET_STATE = 1
ALIGNMENT_GOAL_1 = 2
ALIGNMENT_CONTROL_1 = 3
FORK = 4
ALIGNMENT_GOAL_2 = 5
ALIGNMENT_CONTROL_2 = 6
FREE_TRAVERSE_1 = 7
CP2_PIVOT_1 = 29
CP2_PIVOT_2 = 31
LINE_FOLLOWING_2 = 8
ALIGNMENT_GOAL_3 = 9
ALIGNMENT_CONTROL_3 = 10
FREE_TRAVERSE_2 = 11
LINE_FOLLOW_3 = 12
ALIGNMENT_GOAL_4 = 13
ALIGNMENT_CONTROL_4 = 14
FREE_TRAVERSE_3 = 15
LINE_FOLLOW_GRG = 32
ALIGNMENT_GOAL_5 = 16
ALIGNMENT_CONTROL_5 = 17
FREE_TRAVERSE_4 = 18
ALIGNMENT_GOAL_6 = 19
ALIGNMENT_CONTROL_6 = 20
FREE_TRAVERSE_5 = 21
WALL = 22
ALIGNMENT_GOAL_7 = 23
ALIGNMENT_CONTROL_7 = 24
CUP = 25
ALIGNMENT_GOAL_8 = 26
ALIGNMENT_CONTROL_8 = 27
FINISH_LINE = 33
FINISH = 28

STOP = 40
TESTING = 30

#Total Distance Check Points 
CP1_2_DISTANCE = 1800
LF2_DIST = 2025
CUP_CP3_DIST = 2725
CP3_CP4 = 3150
CP4_LINE = 3275
CP4_GARAGE = 3600
OUT_GARAGE = 3900
BACK_UP_DIST = OUT_GARAGE + 100
TO_CUP = 4300
RUN_TO_LINE = 4500
FINISH_DIST = 5000

## @brief pathing_plan class that can be initialized to be run with the cotask scheduler. See run for behavior. 
class pathing_plan:
## @brief __init__ is the pathing_plan_task object initializer which takes 10 arguments. 
#         The arguments are a mix of data shares and flag shares.
#  @param total_dist : total_dist is a float share that the observer sets with how far ROMI has travelled. It is used to 
#                      figure out when ROMI reaches checkpoints. It does go negative when ROMI reverses which is handled in the WALL state. 
#  @param automatic_mode :  automatic_mode is an integer share flag that is set to tell control_task that it should start automatically following a line. 
#                           pathing plan sets this whenever it wants to line follow. 
#  @param c_state : c_state is an integer share flag that holds the expected state of control_task. 
#                   It is set by UI or pathing plan to tell control_task the expected motion of ROMI. 
#  @param fwd_ref : fwd_ref is an integer share that holds the expected forward reference speed of ROMI in mm.s.
#                   It is set by the UI task for testing purposes or by pathing plan task for course navigation. 
#  @param piv_ref : piv_ref is a float share that holds the expected pivoting reference angular speed of ROMI in rad/s.
#                   It is set by the UI task for testing purposes or by pathing plan task for course navigation. 
#  @param yaw : yaw is a float share that the IMU task fills with the current yaw of ROMI. pathing_plan uses it for setting correct yaw goals. 
#  @param centroid_goal: centroid_goal is a float share that the pathing plan task sets. The control task will  use it as the line following centroid controller set point. 
#  @param yaw_goal : yaw_goal is a float share that is the set point of the heading control. It is the angle ROMI should turn to on the course and is set by
#                    pathing plan for course navigation. 
#  @param bump_on : bump_on is a integer share flag that pathing plan task sets to turn on the bump sensors.
#  @param bump_flg : bump_flg is an integer share flag that the bump task will set if it is on and the bump sensors touch something. It is used for detecting the wall. 
    def __init__(self, total_dist, automatic_mode, c_state, fwd_ref, piv_ref, yaw, centroid_goal, yaw_goal, bump_on, bump_flg):
        self.state = LINE_FOLLOW_STATE_0
        self.total_dist = total_dist
        self.automatic_mode = automatic_mode
        self.c_state = c_state
        self.fwd_ref = fwd_ref
        self.yaw = yaw
        self.yaw_init = 0
        self.piv_ref=piv_ref
        self.centroid_goal = centroid_goal
        self.yaw_goal = yaw_goal
        self.bump_on = bump_on
        self.bump_flg = bump_flg
        self.bump_off = 0
        self.iteration = 0
        self.yaw_goals_5 = [YAWGOAL5, EXPYAWGOAL1, EXPYAWGOAL2, EXPYAWGOAL3]
        self.direction = [1,1,-1,1]
        self.dist = [CP4_GARAGE, 3600, 3800, 4000]
## @brief wrapper will take a yaw value that is typically a goal to be set and wrap it around the 0-2PI range the IMU uses. This stops goals from being 
#         negative or larger than the yaw reading will ever be. 
#  @param val : val is the yaw that is being wrapped
    def wrapper(self, val):
        if val > 2*PI:
            return val-2*PI
        elif val < 0:
            return 2*PI + val
        else:
            return val
## @brief line_following sets all flags necessary to put control task into a line following mode and it checks if ROMI has moved far enough. 
#         It returns True if ROMI has moved far enough and sets the next state, and it returns False if ROMI has further to go. 
#  @param final_dist : final_dist is the value to compare the total distance to to see if its time to go to the next state
#  @param next_state : next_state is the state number of the next_state for pathing plan to go to
    def line_following(self, final_dist, next_state):
        #real_yaw = self.wrapper(self.yaw.get() - self.yaw_init)
        #print(f"Real Yaw {real_yaw}")
        self.automatic_mode.put(1)
        self.centroid_goal.put(0)
        if self.total_dist.get() >= final_dist:
            self.state = next_state
            return True
        return False
## @brief alignment_goal sets all flags necessary to put control task into a pivoting mode to make large inaccurate turns.
#         It returns True if ROMI if within 0.5 rad of the goal and sets the next state, and the yaw_goal for alignment control
#         and it returns False if ROMI has to turn more
#  @param yaw_goal : yaw_goal is the yaw value that is checked to see if pathing plan should go to the next state
#  @param rot_direction : rot_direction should be either 1 or -1 to set which direction to turn ROMI. 1 is CCW, -1 is CW
#  @param next_state : next_state is the state number of the next_state for pathing plan to go to 
    def alignment_goal(self, yaw_goal, rot_direction, next_state):
        self.c_state.put(3)
        self.piv_ref.put(0.75*rot_direction)
        real_yaw = self.wrapper(self.yaw.get() - self.yaw_init)
        #print(f"Real Yaw {real_yaw}")
        if real_yaw >= yaw_goal and real_yaw <= ((yaw_goal)+.5):
            self.c_state.put(4)
            self.piv_ref.put(0)
            self.yaw_goal.put(self.wrapper(yaw_goal + self.yaw_init))
            #print(f"Yaw Goal {self.yaw_goal.get()}")
            self.state = next_state
            self.automatic_mode.put(0)
            return True
        return False
## @brief alignment_control sets all flags necessary to put control task into a controlled turning mode for minute, accurate yaw changes. 
#         It returns True if control_task says that the yaw is aligned properly, and it will set the next state. It is expected to be used after alignment_goal. 
#         It will not set the yaw_goal itself.
#  @param next_state : next_state is the state number of the next_state for pathing plan to go to 
    def alignment_control(self, next_state):
        self.c_state.put(7)
        if self.yaw_goal.get() == 0:
            self.state = next_state
            return True
        return False
## @brief free_traverse sets all flags necessary to put control task into a forward motion mode and it checks if ROMI has moved far enough. 
#         It returns True if ROMI has moved far enough and sets the next state, and it returns False if ROMI has further to go. 
#  @param final_dist : final_dist is the value to compare the total distance to to see if its time to go to the next state
#  @param next_state : next_state is the state number of the next_state for pathing plan to go to  
#  @param ref : ref is the reference speed for ROMI to traverse at in mm/s
    def free_traverse(self, final_dist, next_state, ref):
        self.c_state.put(1)
        self.fwd_ref.put(ref)
        if self.total_dist.get() >= final_dist:
            self.c_state.put(4)
            self.fwd_ref.put(0)
            self.state = next_state
            return True
        return False
## @brief run is the generator that the cotask scheduler will run since our tasks are objects. All shares it needs are already given to 
#         the task object on initialization. The generator will only run one state at a time on each run of the task. 
#  @detail The finite state machine for control_task has 34 states. These states always go in order. Each state runs once and is never returned to. 
#          The order of states is as follows:
#          LINE_FOLLOW_STATE_0, CENTROID_OFFSET_STATE, ALIGNMENT_GOAL_1, ALIGNMENT_CONTROL_1, FORK, ALIGNMENT_GOAL_2, ALIGNMENT_CONTROL_2, FREE_TRAVERSE_1, CP2_PIVOT_1, CP2_PIVOT_2, 
#          LINE_FOLLOWING_2, ALIGNMENT_GOAL_3, ALIGNMENT_CONTROL_3, FREE_TRAVERSE_2, LINE_FOLLOW_3, ALIGNMENT_GOAL_4, ALIGNMENT_CONTROL_4, FREE_TRAVERSE_3, LINE_FOLLOW_GRG, 
#          ALIGNMENT_GOAL_5, ALIGNMENT_CONTROL_5, FREE_TRAVERSE_4, ALIGNMENT_GOAL_6, ALIGNMENT_CONTROL_6, FREE_TRAVERSE_5, WALL, ALIGNMENT_GOAL_7, ALIGNMENT_CONTROL_7, CUP, 
#          ALIGNMENT_GOAL_8, ALIGNMENT_CONTROL_8, FINISH, FINISH_LINE
#
#          Fulling describing each state's exact behavior will probably be more confusing than helpful but here's a general overview of the path:
#
#          Start line following to the Y, move the centroid to the left a bit to keep right, face CP1, drive directly there, face CP2, drive directly there, line follow a bit, 
#          turn to CP3, drive there pushing cup out of the way, turn to CP4, drive there and line follow a bit at the end to align, align at garage entrance, drive through, 
#          turn towards wall, drive out, hit wall and backup, turn towards cup, drive over it, turn towards finish, drive back to the line, use line following to get into
#          CP6. 
#
#          Note: The way alignments are done is with a goal state then a control state. The goal states makes large turns at set pivoting speeds. The control state uses
#          a controller to get precise alignment with our goal heading. Goals are set by the goal state though so a controller align must always be ran after a goal align. 
#
#          Note: Yaw drift is inconsistent so this path is not consistent do not run ROMI unsupervised. 
    def run(self):
        while (True):

#---------------------------------------------------------#
#   To CP 1                                               #
#---------------------------------------------------------#
            if self.yaw_init == 0:
                self.yaw_init = self.yaw.get()
                print(f"Yaw Offset : {self.yaw_init}")

            if self.state == LINE_FOLLOW_STATE_0:
            # Follow the line to the fork
                
                if self.line_following(550, CENTROID_OFFSET_STATE):
                    print("Going to CENTROID OFFSET STATE")
                yield 0

            elif self.state == CENTROID_OFFSET_STATE:
            # Follow right fork
                self.centroid_goal.put(-0.4)
                real_yaw = self.wrapper(self.yaw.get() - self.yaw_init)
                #print(f"Real Yaw {real_yaw}")
                if self.total_dist.get() >= 950:
                    print("Going to ALIGNMENT GOAL 1")
                    self.centroid_goal.put(0)
                    self.automatic_mode.put(0)
                    self.state = ALIGNMENT_GOAL_1
                yield 1

            elif self.state == ALIGNMENT_GOAL_1:
            # Align to 90 degrees
                if self.alignment_goal(YAWGOAL1, -1, ALIGNMENT_CONTROL_1):
                    print("Going to ALIGNMENT CONTROL 1")
                yield ALIGNMENT_GOAL_1
                yield 2

            elif self.state == ALIGNMENT_CONTROL_1:
            # Align to 90 degrees
                if self.alignment_control(FORK):
                    print("Going to FORK")
                yield ALIGNMENT_CONTROL_1

            elif self.state == FORK:
            # Traverse the fork to CP 1 
                if self.free_traverse(1175, ALIGNMENT_GOAL_2, 100):
                    print("Going to ALIGNMENT GOAL 2")
                yield FORK

#---------------------------------------------------------#
#   To CP 2                                               #
#---------------------------------------------------------#

            elif self.state == ALIGNMENT_GOAL_2:
            # Turn towards CP 2
                if self.alignment_goal(YAWGOAL2, 1, ALIGNMENT_CONTROL_2):
                    print("Going to ALIGNMENT CONTROL 2")
                yield ALIGNMENT_GOAL_2

            elif self.state == ALIGNMENT_CONTROL_2:
            #Controller align to CP 2
                if self.alignment_control(FREE_TRAVERSE_1):
                    print("Going to FREE_TRAVERSE_1")
                yield ALIGNMENT_CONTROL_2

            elif self.state == FREE_TRAVERSE_1:
            # Move to CP 2
                if self.free_traverse(CP1_2_DISTANCE, CP2_PIVOT_1, 100):
                    print("Going to CP2_PIVOT_1")
                yield FREE_TRAVERSE_1

            elif self.state == CP2_PIVOT_1:
            # Turn towards cup
                if self.alignment_goal(YAWGOAL7, -1, CP2_PIVOT_2):
                    print("Going to CP2_PIVOT_2")
                yield CP2_PIVOT_1

            elif self.state == CP2_PIVOT_2:
            #Controller align to cup
                if self.alignment_control(LINE_FOLLOWING_2):
                    print("Going to LINE_FOLLOWING_2")
                yield CP2_PIVOT_2

            elif self.state == LINE_FOLLOWING_2:
                self.automatic_mode.put(1)
                if self.total_dist.get() >= LF2_DIST:
                    print("Going to ALIGNMENT GOAL 3 / START OF FUNCTION TESTS")
                    self.automatic_mode.put(0)
                    self.state = ALIGNMENT_GOAL_3
                yield LINE_FOLLOWING_2

#---------------------------------------------------------#
#   To CP 3/Cup                                           #
#---------------------------------------------------------#

            elif self.state == ALIGNMENT_GOAL_3:
            # Turn towards cup
                if self.alignment_goal(YAWGOAL3, -1, ALIGNMENT_CONTROL_3):
                    print("Going to ALIGNMENT CONTROL 3")
                yield ALIGNMENT_GOAL_3
                
            elif self.state == ALIGNMENT_CONTROL_3:
            #Controller align to cup
                if self.alignment_control(FREE_TRAVERSE_2):
                    print("Going to FREE_TRAVERSE_2")
                yield ALIGNMENT_CONTROL_3
                            
            elif self.state == FREE_TRAVERSE_2:
            # Run over cup
                if self.free_traverse(CUP_CP3_DIST, ALIGNMENT_GOAL_4, 100):
                    print("Going to ALIGNMENT_GOAL_4")
                yield FREE_TRAVERSE_2
            
            elif self.state == LINE_FOLLOW_3:
                if self.line_following(2700, STOP):
                    print("Going to ALIGNMENT GOAL 4")
                yield LINE_FOLLOW_3

#---------------------------------------------------------#
#   To CP 4                                               #
#---------------------------------------------------------#

            elif self.state == ALIGNMENT_GOAL_4:
            # Turn towards CP4
                if self.alignment_goal(YAWGOAL4, -1, ALIGNMENT_CONTROL_4):
                    print("Going to ALIGNMENT CONTROL 4")
                yield ALIGNMENT_GOAL_3

            elif self.state == ALIGNMENT_CONTROL_4:
            #Controller align to CP4
                if self.alignment_control(FREE_TRAVERSE_3):
                    print("Going to FREE_TRAVERSE_3")
                yield ALIGNMENT_CONTROL_4

            elif self.state == FREE_TRAVERSE_3:
            #Move ot CP 4
                if self.free_traverse(CP3_CP4, LINE_FOLLOW_GRG, 100):
                    print("Going to LINE_FOLLOW_GRG")
                yield FREE_TRAVERSE_3
            
            elif self.state == LINE_FOLLOW_GRG:
            #Line follow a bit to CP 4
                if self.line_following(CP4_LINE, ALIGNMENT_GOAL_5):
                    print("Going to ALIGNMENT GOAL 5")
                yield LINE_FOLLOW_GRG
            
# #---------------------------------------------------------#
# #   To CP 5                                               #
# #---------------------------------------------------------#
            
            elif self.state == ALIGNMENT_GOAL_5:
#           # Align into the garage
                # if self.iteration != 4:
                #     if self.alignment_goal(self.yaw_goals_5[self.iteration], self.direction[self.iteration], ALIGNMENT_CONTROL_5):
                #         print("Going to ALIGNMENT CONTROL 5")
                #         #print(f"iteration: {self.iteration}")
                #     else:
                #         self.iteration += 1
                # else:
                #     print("Going to ALIGNMENT CONTROL 5")
                #     self.state = ALIGNMENT_GOAL_6
                if self.alignment_goal(YAWGOAL5, 1, ALIGNMENT_CONTROL_5):
                    print("Going to Alignment Control 5")
                yield ALIGNMENT_GOAL_5

            elif self.state == ALIGNMENT_CONTROL_5:
            #Refine garage align
                if self.alignment_control(FREE_TRAVERSE_4):
                    print("Going to FREE_TRAVERSE_4")
                yield ALIGNMENT_CONTROL_5
            
            elif self.state == FREE_TRAVERSE_4:
            # Go through garage 
                if self.free_traverse(CP4_GARAGE, ALIGNMENT_GOAL_6, 50):
                    print("Going to ALIGNMENT GOAL 5")
                    print(f"Total Dist: {self.total_dist.get()}")
                yield FREE_TRAVERSE_4   
            
            elif self.state == ALIGNMENT_GOAL_6:
            # Align into the garage
                if self.alignment_goal(YAWGOAL6, -1, ALIGNMENT_CONTROL_6):
                    print("Going to ALIGNMENT CONTROL 6")
                yield ALIGNMENT_GOAL_6

            elif self.state == ALIGNMENT_CONTROL_6:
            #Refine garage align
                if self.alignment_control(FREE_TRAVERSE_5):
                    print("Going to FREE_TRAVERSE_5")
                yield ALIGNMENT_CONTROL_6

            elif self.state == FREE_TRAVERSE_5:
            # Go through garage to CP 5
                if self.free_traverse(OUT_GARAGE, WALL, 100):
                    print("Going to WALL")
                yield FREE_TRAVERSE_5

# #---------------------------------------------------------#
# #   To Finish                                             #
# #---------------------------------------------------------#
            elif self.state == WALL:
                #NEEDS BUMP SENSOR STUFF
                self.c_state.put(1)
                if self.bump_off == 0:
                    self.bump_on.put(1)
                    now = 0
                if self.bump_flg.get() == 1:
                    print("Bump flag on")
                    self.fwd_ref.put(-100)
                    if now == 0:
                        now = self.total_dist.get()
                    if self.total_dist.get() <= now - 100:
                        #self.state = ALIGNMENT_GOAL_7
                        self.state = ALIGNMENT_GOAL_7
                        self.bump_on.put(0)
                        self.bump_off = 1
                else:
                    print("No bump yet")
                    self.fwd_ref.put(100)
                print(f"Total dist: {self.total_dist.get()}")
                yield WALL

            elif self.state == ALIGNMENT_GOAL_7:
            # Align to the second cup
                if self.alignment_goal(YAWGOAL7, -1, ALIGNMENT_CONTROL_7):
                    print("Going to ALIGNMENT CONTROL 7")
                yield ALIGNMENT_GOAL_7

            elif self.state == ALIGNMENT_CONTROL_7:
            #Align to the second cup
                if self.alignment_control(CUP):
                    print("Going to CUP")
                yield ALIGNMENT_CONTROL_7

            elif self.state == CUP:
            # Go to cup
                if self.free_traverse(TO_CUP, ALIGNMENT_GOAL_8, 100):
                    print("Going to ALIGNMENT GOAL 8")
                yield CUP

            elif self.state == ALIGNMENT_GOAL_8:
            # Align to the finish
                if self.alignment_goal(YAWGOAL8, 1, ALIGNMENT_CONTROL_8):
                    print("Going to ALIGNMENT CONTROL 8")
                yield ALIGNMENT_GOAL_8

            elif self.state == ALIGNMENT_CONTROL_8:
            #Refine finish align
                if self.alignment_control(FINISH):
                    print("Going to FINISH")
                yield ALIGNMENT_CONTROL_8

            elif self.state == FINISH:
            #Go to the finish
                if self.free_traverse(RUN_TO_LINE, FINISH_LINE, 100):
                    print("Going to STOP")
                yield FINISH

            elif self.state == FINISH_LINE:
                if self.line_following(FINISH_DIST, STOP):
                    print("FINISHED")
                yield FINISH_LINE

            elif self.state == STOP:
                self.c_state.put(4)
                self.automatic_mode.put(0)
                yield STOP
            elif self.state == 40:
                self.c_state.put(3)
                self.piv_ref.put(.5)
                self.automatic_mode.put(0)
            else:
                raise ValueError("Not that many states")
            yield self.state

