from pyb import Pin, Timer, ADC
## @brief Collision detector for cooperative scheduler
#
# This class runs continuously, setting flags that are shared with other tasks.
#
# The Bump task uses two sensor objects, for left and right side of the Romi. Any collision with either side will be treated the same.
class Bump_Task:
    ## @brief Shares are passed as parameters, used to communicate between tasks in the cooperative scheduler.  
    # @param c_state A flag used to tell which control state is desired
    # @param bump_sensor_right The right bump Sensor Object
    # @param bump_sensor_left The left bump Sensor Object
    # @param bump_on_off A flag to tell bump_task to read if the sensors are pressed
    # @param bump_flg A flag set if a sensor was pressed while being read
    def __init__(self, c_state, bump_sensor_right, bump_sensor_left,bump_on_off, bump_flg):
        self.c_state = c_state
        self.bump_sensor_right=bump_sensor_right
        self.bump_sensor_left=bump_sensor_left
        self.state=0
        self.bump_on_off=bump_on_off
        self.bump_flg=bump_flg
        self.bump_flg.put(0)

    ## @brief This function runs via the cooperative scheduler.
    # 
    # <b> State 1 </b> Not reading sensors
    #
    # <b> State 1 </b> Reading sensors
    #
    # <b> Effects </b> 
    #  <blockquote>
    # Sets bump_flg to 1 when a sensor is pressed from left or right side
    def run(self):
        while (True):
            # bump sensor off
            if self.state==0:
                if self.bump_on_off.get() ==1:
                    self.state=1
                    
                yield
            # bump sensor on
            elif self.state==1:
                
                if self.bump_on_off.get() ==0:
                    self.state=0
                if True in (self.bump_sensor_left.is_pressed(), self.bump_sensor_right.is_pressed()):
                    print('bumped')
                    self.bump_flg.put(1)
                
                
            else:
                    raise ValueError("Not that many states")
            yield self.state

    