## @brief add description here
class data_collector:
    
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
    # Initialize the collector
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