from time import ticks_us, ticks_diff  # Use to get dt value in update()
from pyb import Pin, Timer
## @brief The Encoder Diver Class
# 
#A quadrature encoder decoding interface encapsulated in a Python class
class Encoder:
   
    ## @brief Encoder Initialization
    # @param P1 Timer Used
    # @param P2 Channel A pin
    # @param P3 Channel B pin
    def __init__(self, tim, chA_pin, chB_pin):
        #Initializes an Encoder object
        self.position = 0  # Total accumulated position of the encoder
        self.prev_count = 0  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.dt = 0  # Amount of time between last two updates
        self.last_time = 0
        self.current_time = 0
        self.tim = tim
        self.offset = 0
        self.tim.init(prescaler = 0, period = 65535)
        self.timCh_A = tim.channel(1, pin=chA_pin, mode=Timer.ENC_A)
        self.timCh_B = tim.channel(2, pin=chB_pin, mode=Timer.ENC_B)

    ## @brief Encoder Update
    # 
    # Runs one update step on the encoder's timer counter to keep
    # track of the change in count and check for counter reload
    def update(self):
        self.delta = self.tim.counter() - self.prev_count

        self.current_time=ticks_us()
        self.dt = ticks_diff(self.current_time, self.last_time)
        self.last_time = ticks_us()
        
        if self.delta > 65536 / 2:
            self.delta -= 65536
        elif self.delta < -65536 / 2:
            self.delta += 65536

        self.prev_count = self.tim.counter()
        self.position += self.delta
        
        
    ## @brief Encoder Position Get
    #
    # Returns the most recently updated value of position as determined within the update() method
    def get_position(self):
        return (self.position * 1/1437.1 * 2 * 3.14159 * 35) + self.offset

    ## @brief Encoder Velocity Get
    #
    #Returns a measure of velocity using the most recently updated value of delta as determined within the update() method
    def get_velocity(self):
        return (self.delta / self.dt) * 1/1437.1 *2 * 3.14159 * 35 * -1000000

    ## @brief Zero the Encoder
    #
    # Sets the present encoder position to zero and causes future updates to measure with respect to the new zero position
    def zero(self):
        self.position = 0
        self.prev_count = self.tim.counter()
    
    ## @brief Get dt
    #
    # gets the elapsed time in between encoder updates
    def get_dt(self):
        return self.dt
    
    ## @brief Set Offset
    # @param P1 Offset
    def set_offset(self, offset):
        self.offset = offset 




