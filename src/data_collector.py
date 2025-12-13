## @brief Collects data from shares
#
# This task class which contains a generator finite state machine that collects data from shares and puts them into ques to be read
#
# This class runs continuously
class data_collector:
    
    ## @brief Data Collector Initialization
    # @param P1 Testing Flag
    # @param P2 Position
    # @param P3 Velocity
    # @param P4 Times
    # @param P5 Left Motor Position
    # @param P6 Left Motor Velocity
    # @param P7 Right Motor Position
    # @param P8 Right Motor Velocity
    # @param P9 Velocity 2
    def __init__(self, testing_flg, position, velocity, times, position_l, velocity_l,position_r, velocity_r, velocity2):
        self.testing_flg=testing_flg
        self.position=position
        self.velocity = velocity
        self.times = times
        self.position_l = position_l
        self.velocity_l = velocity_l
        self.position_r = position_r
        self.velocity_r = velocity_r
        self.velocity2 = velocity2 
        self.state = 1
    ## @brief Runs the various tasks Data Collector
    #
    # <b> State 1 </b> Wait for data collection call
    #
    # <b> State 2 </b> Load the Queue for left motor velocity
    #
    # <b> State 3 </b> Load the Queue for right motor velocity
    #
    # <b> State 4 </b> Load the Queue for left motor position
    #
    # <b> State 5 </b> Load the Queue for right motor position
    #
    # <b> State 6 </b> Load the Queue for both motor velocities
    def run(self):
        
        while True:
            # Wait for data collection call
            if self.state == 1:
                if self.testing_flg.get() != 0:
                    self.state = self.testing_flg.get()
                yield 1
            # Queue velocity of left motor
            elif self.state ==2:
                if self.testing_flg.get() == 0:
                    self.state = 1
                self.velocity.put(self.velocity_l.get())
                yield 2
            # Queue velocity of right motor
            elif self.state == 3:   
                if self.testing_flg.get() == 0:
                    self.state = 1
                self.velocity.put(self.velocity_r.get())
                yield 3
            # Queue position of left motor
            elif self.state == 4:
                if self.testing_flg.get() == 0:
                    self.state = 1
                self.position.put(self.position_l.get())
                yield 4
            # Queue position of right motor 
            elif self.state == 5:    
                if self.testing_flg.get() == 0:
                    self.state = 1
                self.position.put(self.position_r.get())
                yield 5
            # Queue both motor velocities at the same time
            elif self.state == 6:
                if self.testing_flg.get() == 0:
                    self.state = 1
                self.velocity.put(self.velocity_l.get())
                self.velocity2.put(self.velocity_r.get())
                yield 6
            else:
                raise ValueError("Not that many states")
            yield self.state
