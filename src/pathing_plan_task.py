"""!
@file pathing_plan_task.py
@brief Pathing plan task for cooperative scheduler

This file contains a class to create a pathing_plan_task object which is compatible with
the cotask scheduler to run as a cooperative task. 

The pathing_plan object uses a finite state machine design to dictate the motion of
the Pololu ROMI kit used in ME 405, Fall 2025 to follow the term project course. The task
sets share flags and goals to order how control task moves the ROMI to reach the 6 checkpoints 
and the two bonus cups on the course. A long list of defines are used at the beginning to make tuning
path simpler. The state names are used to make adding new states inbetween states easier so that the file
reads in the order of how ROMI will move. The goal defines are what are tuned for the course. When moving forward
the task uses the total distance travelled as found by the observer to know when it needs to change states. 
When turning, the task waits for the yaw from the IMU to be within an acceptable range to change states. 

@note The intention of the angle goals was to be done in a global sense where 0 is the direction ROMI faces 
      at the start. However, the IMU has a lot of drift as ROMI goes through the course so these values are found
      experimentally and start to lose meaning when read in the code until you see how ROMI actually moves. 
@note Yaw drift is inconsistent so this path is not consistent do not run ROMI unsupervised. 

@author Alex Power, Lucas Heuchert, Erik Heuchert
@author ChatGPT5 was used for debugging purposes but not to generate this code
@date 2025-Nov-10 Approximate date of creation of file
@date 2025-Dec-12 Final alterations made
"""

# Yaw Goal Defines
PI = 3.1415  ##< Pi constant for angle conversions
YAWGOAL1 = (82)*(PI/180)  ##< Fork angle
YAWGOAL2 = (329)*(PI/180)  ##< CP1 to CP2
YAWGOAL3 = (90)*(PI/180)  ##< CP2 to Cup 
YAWGOAL4 = (166)*(PI/180)  ##< CP3 to CP4
YAWGOAL5 = (163.5)*(PI/180)  ##< CP4 to garage
YAWGOAL6 = (255)*(PI/180)  ##< Garage to CP5
YAWGOAL7 = (345)*(PI/180)  ##< Wall to Cup
YAWGOAL8 = (240)*(PI/180)  ##< Cup to Finish

# State Names in Order
LINE_FOLLOW_STATE_0 = 0  ##< Initial line following state
CENTROID_OFFSET_STATE = 1  ##< Centroid offset adjustment state
ALIGNMENT_GOAL_1 = 2  ##< First alignment goal state
ALIGNMENT_CONTROL_1 = 3  ##< First alignment control state
FORK = 4  ##< Fork navigation state
ALIGNMENT_GOAL_2 = 5  ##< Second alignment goal state
ALIGNMENT_CONTROL_2 = 6  ##< Second alignment control state
FREE_TRAVERSE_1 = 7  ##< First free traverse state
CP2_PIVOT_1 = 29  ##< CP2 first pivot state
CP2_PIVOT_2 = 31  ##< CP2 second pivot state
LINE_FOLLOWING_2 = 8  ##< Second line following state
ALIGNMENT_GOAL_3 = 9  ##< Third alignment goal state
ALIGNMENT_CONTROL_3 = 10  ##< Third alignment control state
FREE_TRAVERSE_2 = 11  ##< Second free traverse state
LINE_FOLLOW_3 = 12  ##< Third line following state
ALIGNMENT_GOAL_4 = 13  ##< Fourth alignment goal state
ALIGNMENT_CONTROL_4 = 14  ##< Fourth alignment control state
FREE_TRAVERSE_3 = 15  ##< Third free traverse state
LINE_FOLLOW_GRG = 32  ##< Garage line following state
ALIGNMENT_GOAL_5 = 16  ##< Fifth alignment goal state
ALIGNMENT_CONTROL_5 = 17  ##< Fifth alignment control state
FREE_TRAVERSE_4 = 18  ##< Fourth free traverse state
ALIGNMENT_GOAL_6 = 19  ##< Sixth alignment goal state
ALIGNMENT_CONTROL_6 = 20  ##< Sixth alignment control state
FREE_TRAVERSE_5 = 21  ##< Fifth free traverse state
WALL = 22  ##< Wall detection state
ALIGNMENT_GOAL_7 = 23  ##< Seventh alignment goal state
ALIGNMENT_CONTROL_7 = 24  ##< Seventh alignment control state
CUP = 25  ##< Cup navigation state
ALIGNMENT_GOAL_8 = 26  ##< Eighth alignment goal state
ALIGNMENT_CONTROL_8 = 27  ##< Eighth alignment control state
FINISH_LINE = 33  ##< Finish line state
FINISH = 28  ##< Finish state

STOP = 40  ##< Stop state
TESTING = 30  ##< Testing state

# Total Distance Check Points 
CP1_2_DISTANCE = 1800  ##< Distance from CP1 to CP2
LF2_DIST = 2025  ##< Line follow 2 distance
CUP_CP3_DIST = 2725  ##< Distance from cup to CP3
CP3_CP4 = 3150  ##< Distance from CP3 to CP4
CP4_LINE = 3275  ##< Distance to CP4 line
CP4_GARAGE = 3600  ##< Distance from CP4 to garage
OUT_GARAGE = 3900  ##< Distance out of garage
BACK_UP_DIST = OUT_GARAGE + 100  ##< Backup distance
TO_CUP = 4300  ##< Distance to cup
RUN_TO_LINE = 4500  ##< Distance to run to line
FINISH_DIST = 5000  ##< Finish distance


class pathing_plan:
    """!
    @brief Pathing plan task class for ROMI navigation
    
    This class implements a finite state machine for autonomous navigation of the Pololu ROMI
    robot through a predefined course with checkpoints and obstacles.
    """
    
    def __init__(self, total_dist, automatic_mode, c_state, fwd_ref, piv_ref, yaw, centroid_goal, yaw_goal, bump_on, bump_flg):
        """!
        @brief Initialize the pathing_plan_task object
        
        @param total_dist Float share containing total distance traveled by ROMI (set by observer)
        @param automatic_mode Integer share flag to enable automatic line following mode
        @param c_state Integer share flag holding the expected state of control_task
        @param fwd_ref Integer share holding forward reference speed in mm/s
        @param piv_ref Float share holding pivoting reference angular speed in rad/s
        @param yaw Float share containing current yaw from IMU task
        @param centroid_goal Float share for line following centroid controller set point
        @param yaw_goal Float share for heading control set point (target angle)
        @param bump_on Integer share flag to turn on bump sensors
        @param bump_flg Integer share flag set by bump task when sensors are triggered
        """
        self.state = LINE_FOLLOW_STATE_0
        self.total_dist = total_dist
        self.automatic_mode = automatic_mode
        self.c_state = c_state
        self.fwd_ref = fwd_ref
        self.yaw = yaw
        self.yaw_init = 0
        self.piv_ref = piv_ref
        self.centroid_goal = centroid_goal
        self.yaw_goal = yaw_goal
        self.bump_on = bump_on
        self.bump_flg = bump_flg
        self.bump_off = 0
        self.iteration = 0
        self.yaw_goals_5 = [YAWGOAL5, EXPYAWGOAL1, EXPYAWGOAL2, EXPYAWGOAL3]
        self.direction = [1, 1, -1, 1]
        self.dist = [CP4_GARAGE, 3600, 3800, 4000]
        
    def wrapper(self, val):
        """!
        @brief Wrap a yaw value to the 0-2PI range
        
        This method ensures yaw values stay within the valid range used by the IMU,
        preventing negative values or values exceeding 2π.
        
        @param val The yaw value to be wrapped
        @return The wrapped yaw value in the range [0, 2π]
        """
        if val > 2*PI:
            return val - 2*PI
        elif val < 0:
            return 2*PI + val
        else:
            return val
            
    def line_following(self, final_dist, next_state):
        """!
        @brief Configure control task for line following and check progress
        
        Sets all necessary flags to put control_task into line following mode and monitors
        whether ROMI has traveled far enough to transition to the next state.
        
        @param final_dist Distance threshold to compare against total distance traveled
        @param next_state State number to transition to when distance is reached
        @return True if ROMI has moved far enough and state is updated, False otherwise
        """
        self.automatic_mode.put(1)
        self.centroid_goal.put(0)
        if self.total_dist.get() >= final_dist:
            self.state = next_state
            return True
        return False
        
    def alignment_goal(self, yaw_goal, rot_direction, next_state):
        """!
        @brief Configure control task for large pivoting turns
        
        Sets all necessary flags to put control_task into pivoting mode for making large,
        inaccurate turns. Transitions to the next state when within 0.5 rad of goal.
        
        @param yaw_goal Target yaw value to reach
        @param rot_direction Direction to turn (1 for CCW, -1 for CW)
        @param next_state State number to transition to when goal is reached
        @return True if ROMI is within tolerance and state is updated, False otherwise
        """
        self.c_state.put(3)
        self.piv_ref.put(0.75*rot_direction)
        real_yaw = self.wrapper(self.yaw.get() - self.yaw_init)
        if real_yaw >= yaw_goal and real_yaw <= ((yaw_goal) + 0.5):
            self.c_state.put(4)
            self.piv_ref.put(0)
            self.yaw_goal.put(self.wrapper(yaw_goal + self.yaw_init))
            self.state = next_state
            self.automatic_mode.put(0)
            return True
        return False
        
    def alignment_control(self, next_state):
        """!
        @brief Configure control task for precise controlled turning
        
        Sets all necessary flags for controlled turning mode to achieve precise yaw alignment.
        Expected to be used after alignment_goal(). Does not set yaw_goal itself.
        
        @param next_state State number to transition to when alignment is complete
        @return True if control_task indicates proper alignment, False otherwise
        """
        self.c_state.put(7)
        if self.yaw_goal.get() == 0:
            self.state = next_state
            return True
        return False
        
    def free_traverse(self, final_dist, next_state, ref):
        """!
        @brief Configure control task for forward motion and check progress
        
        Sets all necessary flags to put control_task into forward motion mode and monitors
        whether ROMI has traveled far enough to transition to the next state.
        
        @param final_dist Distance threshold to compare against total distance traveled
        @param next_state State number to transition to when distance is reached
        @param ref Reference speed for ROMI to traverse at in mm/s
        @return True if ROMI has moved far enough and state is updated, False otherwise
        """
        self.c_state.put(1)
        self.fwd_ref.put(ref)
        if self.total_dist.get() >= final_dist:
            self.c_state.put(4)
            self.fwd_ref.put(0)
            self.state = next_state
            return True
        return False
        
    def run(self):
        """!
        @brief Main task generator for cooperative scheduler
        
        Implements a 34-state finite state machine for autonomous course navigation.
        Each state executes once in sequence and never returns. The generator yields
        control after processing each state.
        
        @details State sequence overview:
        
        States execute in the following order: LINE_FOLLOW_STATE_0, CENTROID_OFFSET_STATE, 
        ALIGNMENT_GOAL_1, ALIGNMENT_CONTROL_1, FORK, ALIGNMENT_GOAL_2, ALIGNMENT_CONTROL_2, 
        FREE_TRAVERSE_1, CP2_PIVOT_1, CP2_PIVOT_2, LINE_FOLLOWING_2, ALIGNMENT_GOAL_3, 
        ALIGNMENT_CONTROL_3, FREE_TRAVERSE_2, LINE_FOLLOW_3, ALIGNMENT_GOAL_4, 
        ALIGNMENT_CONTROL_4, FREE_TRAVERSE_3, LINE_FOLLOW_GRG, ALIGNMENT_GOAL_5, 
        ALIGNMENT_CONTROL_5, FREE_TRAVERSE_4, ALIGNMENT_GOAL_6, ALIGNMENT_CONTROL_6, 
        FREE_TRAVERSE_5, WALL, ALIGNMENT_GOAL_7, ALIGNMENT_CONTROL_7, CUP, ALIGNMENT_GOAL_8, 
        ALIGNMENT_CONTROL_8, FINISH, FINISH_LINE
        
        Path overview: Start line following to the Y, move the centroid to the left to keep right,
        face CP1, drive directly there, face CP2, drive directly there, line follow briefly, 
        turn to CP3, drive there pushing cup out of the way, turn to CP4, drive there and line 
        follow at the end to align, align at garage entrance, drive through, turn towards wall, 
        drive out, hit wall and backup, turn towards cup, drive over it, turn towards finish, 
        drive back to the line, use line following to get into CP6.
        
        @note Alignments use two-stage process: goal state for large turns at set speeds,
              then control state for precise alignment using controller
        @note Yaw drift is inconsistent making the path unreliable - do not run unsupervised
        
        @return Yields state number after each iteration
        """
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
                # Controller align to CP 2
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
                # Controller align to cup
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
                # Controller align to cup
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
                # Controller align to CP4
                if self.alignment_control(FREE_TRAVERSE_3):
                    print("Going to FREE_TRAVERSE_3")
                yield ALIGNMENT_CONTROL_4

            elif self.state == FREE_TRAVERSE_3:
                # Move to CP 4
                if self.free_traverse(CP3_CP4, LINE_FOLLOW_GRG, 100):
                    print("Going to LINE_FOLLOW_GRG")
                yield FREE_TRAVERSE_3
            
            elif self.state == LINE_FOLLOW_GRG:
                # Line follow a bit to CP 4
                if self.line_following(CP4_LINE, ALIGNMENT_GOAL_5):
                    print("Going to ALIGNMENT GOAL 5")
                yield LINE_FOLLOW_GRG
            
            #---------------------------------------------------------#
            #   To CP 5                                               #
            #---------------------------------------------------------#
            
            elif self.state == ALIGNMENT_GOAL_5:
                # Align into the garage
                if self.alignment_goal(YAWGOAL5, 1, ALIGNMENT_CONTROL_5):
                    print("Going to Alignment Control 5")
                yield ALIGNMENT_GOAL_5

            elif self.state == ALIGNMENT_CONTROL_5:
                # Refine garage align
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
                # Refine garage align
                if self.alignment_control(FREE_TRAVERSE_5):
                    print("Going to FREE_TRAVERSE_5")
                yield ALIGNMENT_CONTROL_6

            elif self.state == FREE_TRAVERSE_5:
                # Go through garage to CP 5
                if self.free_traverse(OUT_GARAGE, WALL, 100):
                    print("Going to WALL")
                yield FREE_TRAVERSE_5

            #---------------------------------------------------------#
            #   To Finish                                             #
            #---------------------------------------------------------#
            elif self.state == WALL:
                # Wall detection and backup using bump sensors
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
                # Align to the second cup
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
                # Refine finish align
                if self.alignment_control(FINISH):
                    print("Going to FINISH")
                yield ALIGNMENT_CONTROL_8

            elif self.state == FINISH:
                # Go to the finish
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
                self.piv_ref.put(0.5)
                self.automatic_mode.put(0)
            else:
                raise ValueError("Not that many states")
            yield self.state