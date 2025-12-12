PI = 3.1415
YAWGOAL1 = (82)*(PI/180) #Fork angle
YAWGOAL2 = (329)*(PI/180)#CP1 to CP2
YAWGOAL3 = (90)*(PI/180)#CP2 to Cup 
YAWGOAL4 = (166)*(PI/180)#CP3 to CP4
YAWGOAL5 = (163.5)*(PI/180)#CP4 to garage
# #With yaw resets
# YAWGOAL6 = (88)*(PI/180)#Garage to CP5
# YAWGOAL7 = (178)*(PI/180)#Wall to Cup
# YAWGOAL8 = (88)*(PI/180)#Cup to Finish

YAWGOAL6 = (255)*(PI/180)#Garage to CP5
YAWGOAL7 = (345)*(PI/180)#Wall to Cup
YAWGOAL8 = (240)*(PI/180)#Cup to Finish

EXPYAWGOAL1 = 167
EXPYAWGOAL2 = 167
EXPYAWGOAL3 = 167

LINE_FOLLOW_STATE_0 = 0
CENTROID_OFFSET_STATE = 1
ALIGNMENT_GOAL_1 = 2
ALIGNMENT_CONTROL_1 = 3
FORK = 4
ALIGNMENT_GOAL_2 = 5
ALIGNMENT_CONTROL_2 = 6
FREE_TRAVERSE_1 = 7
LINE_FOLLOWING_2 = 8
ALIGNMENT_GOAL_3 = 9
ALIGNMENT_CONTROL_3 = 10
FREE_TRAVERSE_2 = 11
LINE_FOLLOW_3 = 12
ALIGNMENT_GOAL_4 = 13
ALIGNMENT_CONTROL_4 = 14
FREE_TRAVERSE_3 = 15
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
FINISH = 28
CP2_PIVOT_1 = 29
CP2_PIVOT_2 = 31
LINE_FOLLOW_GRG = 32
FINISH_LINE = 33

STOP = 40
TESTING = 30

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

## @brief add description here
class pathing_plan:


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

    def wrapper(self, val):
        if val > 2*PI:
            return val-2*PI
        elif val < 0:
            return 2*PI + val
        else:
            return val
        
    def line_following(self, final_dist, next_state):
        #real_yaw = self.wrapper(self.yaw.get() - self.yaw_init)
        #print(f"Real Yaw {real_yaw}")
        self.automatic_mode.put(1)
        self.centroid_goal.put(0)
        if self.total_dist.get() >= final_dist:
            self.state = next_state
            return True
        return False
    
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
    
    def alignment_control(self, next_state):
        self.c_state.put(7)
        if self.yaw_goal.get() == 0:
            self.state = next_state
            return True
        return False
    
    def free_traverse(self, final_dist, next_state, ref):
        self.c_state.put(1)
        self.fwd_ref.put(ref)
        if self.total_dist.get() >= final_dist:
            self.c_state.put(4)
            self.fwd_ref.put(0)
            self.state = next_state
            return True
        return False

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

