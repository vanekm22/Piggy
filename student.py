#!/usr/bin python3
from teacher import PiggyParent
import sys
import time

class Piggy(PiggyParent):

    '''
    *************
    SYSTEM SETUP
    *************
    '''

    def __init__(self, addr=8, detect=True):
        PiggyParent.__init__(self) # run the parent constructor

        ''' 
        MAGIC NUMBERS <-- where we hard-code our settings
        '''
        self.LEFT_DEFAULT = 81
        self.RIGHT_DEFAULT = 80
        self.MIDPOINT = 1475  # what servo command (1000-2000) is straight forward for your bot?
        self.set_motor_power(self.MOTOR_LEFT + self.MOTOR_RIGHT, 0)
        self.load_defaults()

    def load_defaults(self):
        """Implements the magic numbers defined in constructor"""
        self.set_motor_limits(self.MOTOR_LEFT, self.LEFT_DEFAULT)
        self.set_motor_limits(self.MOTOR_RIGHT, self.RIGHT_DEFAULT)
        self.set_servo(self.SERVO_1, self.MIDPOINT)

    def menu(self):
        """Displays menu dictionary, takes key-input and calls method"""
        ## This is a DICTIONARY, it's a list with custom index values. Python is cool.
        # Please feel free to change the menu and add options.
        print("\n *** MENU ***") 
        menu = {"n": ("Navigate", self.nav),
                "d": ("Dance", self.dance),
                "o": ("Obstacle count", self.obstacle_count),
                "s": ("Shy", self.shy),
                "f": ("Follow", self.follow),
                "c": ("Calibrate", self.calibrate),
                "q": ("Quit", self.quit),
                "v": ("Vanek Test", self.vanek)
                }
        # loop and print the menu...
        for key in sorted(menu.keys()):
            print(key + ":" + menu[key][0])
        # store the user's answer
        ans = str.lower(input("Your selection: "))
        # activate the item selected
        menu.get(ans, [None, self.quit])[1]()

    '''
    ****************
    STUDENT PROJECTS
    ****************
    '''

    def vanek(self):
        
        #print( self.scan(50) )
        for i in range(4):
          self.gyro_pid(90)
          time.sleep(2)
        for i in range(4):
          self.gyro_pid(-90)
          time.sleep(2)
        
        



        '''
        ## CLOSER!!
        
        # check gyro drift
        # take aways == gyro gets crazy errors when printing, but not without.
        last_reading = self.get_heading(True)
        while True:
          current_reading = self.get_heading(False)
          if abs(current_reading - last_reading) > 100:
            while abs(current_reading - last_reading) > 100:
              current_reading = self.get_heading(False)
              print ("Found crazy reading")
          last_reading = current_reading 
        '''
        
    def square(self):
      self.fwd()
      time.sleep(2)
      self.stop()

      self.right()
      time.sleep(0.75)
      self.stop()

    def dance(self):
        """A higher-ordered algorithm to make your robot dance"""
        if safe_to_dance():
          pass
          
          # TODO: check to see if it's safe before dancing
        
        # lower-ordered example...
        
        # create dance
        self.right(primary=50, counter=50)
        time.sleep(2)
        self.stop()
        

    def safe_to_dance(self):
        """ Does a 360 distance check and returns true if safe """
        return True

    def shake(self):
        """ Another example move """
        self.deg_fwd(720)
        self.stop()

    def example_move(self):
        """this is an example dance move that should be replaced by student-created content"""
        self.right() # start rotating right
        time.sleep(1) # turn for a second
        self.stop() # stop
        self.servo(1000) # look right
        time.sleep(.25) # give your head time to move
        self.servo(2000) # look left

    def scan(self, resolution = 100, endpoints = 500):
        #scan_data = []
        scan_data = { }
        """Sweep the servo and populate the scan_data list"""
        for angle in range( self.MIDPOINT-endpoints, self.MIDPOINT+endpoints, resolution ):
            self.servo(angle)
            time.sleep(0.25)
            scan_data[angle] =  self.read_distance() 
        self.servo(self.MIDPOINT)
        time.sleep(0.5)
        return scan_data


    def obstacle_count(self):
        """Does a 360 scan and returns the number of obstacles it sees"""
        pass

    def nav(self):
        print("-----------! NAVIGATION ACTIVATED !------------\n")
        print("-------- [ Press CTRL + C to stop me ] --------\n")
        print("-----------! NAVIGATION ACTIVATED !------------\n")
        
        # TODO: build self.quick_check() that does a fast, 3-part check instead of read_distance
        while self.read_distance() > 250:  # TODO: fix this magic number
            self.fwd()
            time.sleep(.01)
        self.stop()
        # TODO: scan so we can decide left or right
        # TODO: average the right side of the scan dict
        # TODO: average the left side of the scan dict
        


###########
## MAIN APP
if __name__ == "__main__":  # only run this loop if this is the main file

    p = Piggy()

    if sys.version_info < (3, 0):
        sys.stdout.write("Sorry, requires Python 3.x\n")
        p.quit()

    try:
        while True:  # app loop
            p.menu()

    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        p.quit()  
