# PYTHON3 ONLY
# Based on:
# https://www.dexterindustries.com/GoPiGo/
# https://github.com/DexterInd/GoPiGo3
# Copyright (c) 2017 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information see https://github.com/DexterInd/GoPiGo3/blob/master/LICENSE.md
# Distance sensor and IMU both plugged into I2C
import gopigo3, sys, time
from di_sensors.easy_distance_sensor import EasyDistanceSensor
from di_sensors import inertial_measurement_unit

class PiggyParent(gopigo3.GoPiGo3):

    '''
    UTILITIES
    '''

    def __init__(self, addr=8, detect=True):
        gopigo3.GoPiGo3.__init__(self)
        self.MIDPOINT = 1500
        self.scan_data = {}
        # mutex sensors on IC2
        self.distance_sensor = EasyDistanceSensor(port="RPI_1", use_mutex=True)
        self.imu = inertial_measurement_unit.InertialMeasurementUnit(bus="RPI_1") # mutex crashes this
        # buffer for reading the gyro
        self.gyro_buffer = 0
        self.stop()

    def calibrate(self):
        """allows user to experiment on finding centered midpoint and even motor speeds"""
        print("Calibrating...")
        self.servo(self.MIDPOINT)
        response = str.lower(input("Am I looking straight ahead? (y/n): "))
        if response == 'n':
            while True:
                response = str.lower(input("Turn right, left, or am I done? (r/l/d): "))
                if response == "r":
                    self.MIDPOINT -= 25
                    print("Midpoint: " + str(self.MIDPOINT))
                    self.servo(self.MIDPOINT)
                elif response == "l":
                    self.MIDPOINT += 25
                    print("Midpoint: " + str(self.MIDPOINT))
                    self.servo(self.MIDPOINT)
                else:
                    print("Midpoint temporarily saved to: " + str(self.MIDPOINT) + "\nYou'll need to update your magic number.")
                    break
        else:
            print('Cool, %d is the correct self.MIDPOINT' % self.MIDPOINT)
        response = str.lower(input("Do you want to check if I'm driving straight? (y/n)"))
        if 'y' in response:
            while True:
                self.set_motor_limits(self.MOTOR_LEFT, self.LEFT_DEFAULT)
                self.set_motor_limits(self.MOTOR_RIGHT, self.RIGHT_DEFAULT)
                print("LEFT: {} // RIGHT: {} ".format(self.LEFT_DEFAULT, self.RIGHT_DEFAULT))
                self.fwd()
                time.sleep(1)
                self.stop()
                response = str.lower(input("Reduce left, Increase left or drive? (r/i/d): "))
                if response in "r" or response in "i":
                  amount = int(input("by how much would you like to change the value? (number)"))
                if response == 'r':
                    self.LEFT_DEFAULT -= amount
                elif response == 'i':
                    self.LEFT_DEFAULT += amount
                elif response == 'd':
                    self.fwd()
                    time.sleep(1)
                    self.stop()
                else:
                    break

    def quit(self):
        """Terminates robot movement and settings then closes app"""
        print("\nIt's been a pleasure.\n")
        self.reset_all()
        sys.exit(1)

    '''
    MOVEMENT
    '''

    def deg_fwd(self, deg):
        """Zeroes current encoder values then moves forward based on degrees given"""
        self.offset_motor_encoder(self.MOTOR_LEFT, self.get_motor_encoder(self.MOTOR_LEFT))
        self.offset_motor_encoder(self.MOTOR_RIGHT, self.get_motor_encoder(self.MOTOR_RIGHT))

        while (self.get_motor_encoder(self.MOTOR_RIGHT) < deg):
          self.set_motor_position(self.MOTOR_LEFT + self.MOTOR_RIGHT, deg)
        self.stop()

    def gyro_pid(self, 
                 target_angle, 
                 top_speed = 80,
                 low_speed = 18,
                 kP  = 0.4,
                 kI = 0.001, 
                 kD = 0.0,
                 acceptable_ending_error = 1):

        # Logfile of power added to my turn each frame 
        powerlist = []  

        # Set start points of variables
        error = 0
        error_total = 0
        turning = True
        starting_angle = self.get_heading(False)

        # If I'm at an endpoint for the loop, bring my starting angle to a number
        # above or below my turn (depending on if my target angle is + or -)
        
        '''
        if (starting_angle + target_angle > 360):
            starting_angle -= 360
        elif (starting_angle + target_angle < 0):
            starting_angle += 360
        '''
        # determine what angle I am going to -- important if it wraps.
        # destination_angle = starting_angle + target_angle
        destination_angle = (starting_angle + target_angle) %360
        # determine the direction of the turn
        if starting_angle % 360 < target_angle + starting_angle:
            turn_direction = "right"
        else:
            turn_direction = "left"

        # find initial error.
        error = abs(destination_angle - starting_angle)
        
        # Do the actual turn
        while(turning):
            # Get current gyro position
            current_heading = self.get_heading(False)

            # Wrap the number if we are going across the endpoint.
            # current_heading = (current_heading + error) %360
            '''
            if ( current_heading + error > 360):
              current_heading -= 360
            elif ( current_heading + error < 0):
              current_heading += 360
            '''
            error = (destination_angle - current_heading ) % 360 
            #print (error)
            
            if ( error <= acceptable_ending_error ):
                self.set_motor_power(self.MOTOR_LEFT, 0.0)
                self.set_motor_power(self.MOTOR_RIGHT, 0.0)
                turning = False

            else:
              error_total += error
              last_error = error

              pTerm = error * kP
              iTerm = kI * error_total 
              dTerm = kD * (error - last_error)
              

              power = pTerm + iTerm + dTerm
              
              if (power > top_speed):
                  power = top_speed
              elif (power < low_speed):
                  power = low_speed

               
              if (current_heading > destination_angle):# and "right" in turn_direction:
                print("Went Past Angle")
                self.stop()
                print ("starting_angle: "+ str(starting_angle))
                print ("Current angle: " + str(current_heading))
                print ("target angle: " + str(destination_angle))
                break
                power = -low_speed
              powerlist.append(power)
          

              #turn wheels
              if ("right" in turn_direction):
                  self.set_motor_power(self.MOTOR_LEFT, power)
                  self.set_motor_power(self.MOTOR_RIGHT, -power)
              else:
                  self.set_motor_power(self.MOTOR_LEFT, -power)
                  self.set_motor_power(self.MOTOR_RIGHT, power)

            
        
        time.sleep(1.5)
        print(powerlist)
        final_heading = self.get_heading(False)
        print( "Started at: "+ str(starting_angle) )
        print ( "Final heading: "+ str(final_heading) )
        print( "Trying to get to: " + str(destination_angle) )
        print ( " Off by: "+ str(abs(final_heading - destination_angle)) )


    def gyro_turn(self, deg):
        """Rotates robot relative to it's current heading. If told -20, it will rotate left by 20 degrees."""

        # get our current angle
        current = self.get_heading()

        # calculate delta
        goal = current + deg

        # LOOP AROUND THE 360 marker
        goal %= 360

        # call turn to deg on the delta
        self.execute_gyro_turn(goal)


    def execute_gyro_turn(self, deg):
        """Turns to a degree relative to the gyroscope's readings. If told 20, it will rotate until the gyroscope reads 20."""

        # error check
        error = 0.5
        lowest_speed = 19
        highest_speed = 50
        close = False
        checks = 5
        goal = abs(deg) % 360
        current = self.get_heading()
        time.sleep(0.1)
        print ("AT: " + str(current))
        print ("Heading to: " + str(goal))

        turn = self.right  # connect it to the method without the () to activate
        if (current - goal > 0 and current - goal < 180) or \
           (current - goal < 0 and (360 - goal) + current < 180):
            turn = self.left

        
        # while loop - keep turning until my gyro says I'm there
        while(checks > 0):
          while abs(goal - self.get_heading(False)) > error:
              if (goal > self.get_heading(printing = False)):
                turn_speed = abs(goal - self.get_heading(printing = False))
              elif (self.get_heading(printing = False) > goal):
                turn_speed = abs(360 - self.get_heading(printing = False) + goal)   
              if (turn_speed > highest_speed):
                turn_speed = highest_speed
              if (turn_speed < lowest_speed):
                turn_speed = lowest_speed
              if (turn_speed == lowest_speed):
                close = True
              if ( close and self.get_heading(printing = False) >= goal ):
                turn_speed = lowest_speed
                turn = self.left
              elif( close and self.get_heading(printing = False) <= goal ):
                turn_speed = lowest_speed
                turn = self.right

              turn(primary=turn_speed, counter=-turn_speed)
          self.stop()
          checks -= 1


        # once out of the loop, hit the brakes
        self.stop()
        # report out to the user
        print("\n{} is close enough to {}.\n".format(self.get_heading(), deg))


    def turn_by_deg(self, deg):
        """Rotates robot relative to it's current heading. If told -20, it
        will rotate left by 20 degrees."""

        # get our current angle
        current = self.get_heading()

        # calculate delta
        goal = current + deg

        # LOOP AROUND THE 360 marker
        goal %= 360

        # call turn to deg on the delta
        self.turn_to_deg(goal)


    def time_fwd (self, seconds):
      self.fwd()
      time.sleep(seconds)
      self.stop()


    def turn_to_deg(self, deg):

        """Turns to a degree relative to the gyroscope's readings. If told 20, it
        will rotate until the gyroscope reads 20."""

        # error check
        error = 4
        lowest_speed = 20
        goal = abs(deg) % 360
        current = self.get_heading()

        turn = self.right  # connect it to the method without the () to activate
        if (current - goal > 0 and current - goal < 180) or \
            (current - goal < 0 and (360 - goal) + current < 180):
            turn = self.left

        
        # while loop - keep turning until my gyro says I'm there
        while abs(deg - self.get_heading()) > error:
            turn_speed = abs(goal - self.get_heading())
            if turn_speed < lowest_speed:
              turn_speed = lowest_speed
            turn(primary=turn_speed, counter=-turn_speed)
            
            #time.sleep(.05) # avoid spamming the gyro

        # once out of the loop, hit the brakes
        self.stop()
        # report out to the user
        print("\n{} is close enough to {}.\n".format(self.get_heading(), deg))

    def fwd(self, left=50, right=50):
        """Blindly charges your robot forward at default power which needs to be configured in child class"""
        if self.LEFT_DEFAULT and left == 50:
            left = self.LEFT_DEFAULT
        if self.RIGHT_DEFAULT and right == 50:
            right = self.RIGHT_DEFAULT
        self.set_motor_power(self.MOTOR_LEFT, left)
        self.set_motor_power(self.MOTOR_RIGHT, right)

    def right(self, primary=90, counter=0):
        """Rotates right by powering the left motor to default"""
        self.set_motor_power(self.MOTOR_LEFT, primary)
        self.set_motor_power(self.MOTOR_RIGHT, counter)

    def left(self, primary=90, counter=0):
        """Rotates left by powering the left motor to 90 by default and counter motion 0"""
        self.set_motor_power(self.MOTOR_LEFT, counter)
        self.set_motor_power(self.MOTOR_RIGHT, primary)      

    def back(self, left=-50, right=-50):
        if self.LEFT_DEFAULT and left == -50:
            left = -self.LEFT_DEFAULT
        if self.RIGHT_DEFAULT and right == -50:
            right = -self.RIGHT_DEFAULT
        self.set_motor_power(self.MOTOR_LEFT, left)
        self.set_motor_power(self.MOTOR_RIGHT, right)

    def servo(self, angle):
        """Moves the servo to the given angle"""
        print("Servo moving to: {} ".format(angle))
        self.set_servo(self.SERVO_1, angle)
        time.sleep(.05)

    def stop(self):
        """Cut power to the motors"""
        print("\n--STOPPING--\n")
        self.set_motor_power(self.MOTOR_LEFT + self.MOTOR_RIGHT, 0)

    '''
    SENSORS
    '''

    def read_distance(self):
        """Returns the GoPiGo3's distance sensor in MM over IC2"""
        d = self.distance_sensor.read_mm()
        print("Distance Sensor Reading: {} mm ".format(d))
        return d

    def get_heading(self, printing = True):
        """Returns the heading from the IMU sensor, or if there's a sensor exception, it returns
        the last saved reading"""
        try:
            self.gyro_buffer = self.imu.read_euler()[0]
            if (printing):
              print("Gyroscope sensor is at: {} degrees ".format(self.gyro_buffer))
        except Exception as e:
            print("----- PREVENTED GYRO SENSOR CRASH -----")
            print(e)
        return self.gyro_buffer

        

    '''
    SHOWOFF SCRIPTS
    '''

    def shy(self):
        """Responds to a close reading on the distance sensor by backing up"""
        while True:
            for ang in range(self.MIDPOINT-400, self.MIDPOINT+401, 100):
                self.servo(ang)
                time.sleep(.1)
                if self.read_distance() < 250:
                    self.back()
                    time.sleep(.3)
                    self.stop()
                    for x in range(3):
                        self.servo(self.MIDPOINT + 300)
                        time.sleep(.15)
                        self.servo(self.MIDPOINT - 300)
                        time.sleep(.15)
                    self.servo(self.MIDPOINT)
                    self.fwd()
                    time.sleep(.2)
                    self.stop()

    def follow(self):
        """Responds to a close reading on the distance sensor by attempting to maintain heading"""
        while True:
            for ang in range(self.MIDPOINT-400, self.MIDPOINT+401, 100):
                self.servo(ang)
                time.sleep(.1)
                if self.read_distance() < 250:
                    self.look_excited()

    def look_excited(self):
        for x in range(2):
            self.turn_by_deg(30)
            time.sleep(.25)
            self.turn_by_deg(-30)
            time.sleep(.25)
            self.stop()


