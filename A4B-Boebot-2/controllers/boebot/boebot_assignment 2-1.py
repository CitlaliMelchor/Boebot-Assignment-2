# Initial Webot controller for Boebot training in Automation for Bioproduction
# Assignement 2
# Sam Blaauw - v0.1 - 01-05-2020

# load libraries
from controller import Robot, LED, Motor, PositionSensor, DistanceSensor
import math

# define constants
TIME_STEP = 20 #[ms], must be a multiple of "Worlds basicTimeStemp"
PRINT_INTERVAL = 500 #[ms], time between console messages
BLINK_INTERVAL = 333 #[ms], time LED stays on and off
MAX_SPEED = 20 #[rad/s], maximum rotaional wheel speed
DRIVING_VELOCITY = 0.15 #[m/s], desired driving velocity of Boebot
WHEEL_RADIUS = 0.034 #[m], radius of Boebot wheel
TURN_RADIUS = 0.25 #[m], radius of the turn on the headland
TRACK_WIDTH = 0.095 #[m], space between wheels
LENGTH_BOARD = 1.0 #[m], length of the board

# define user functions
def Toggle(status):
    # change from 1 to 0 and back
    if status == 1:
        value = 0
    else:
        value = 1
    return value

# create the Robot instance.
boebot = Robot()

# assign sensors
left_position_sensor = boebot.getPositionSensor("left wheel sensor")
right_position_sensor = boebot.getPositionSensor("right wheel sensor")

#initialize sensors
left_position_sensor.enable(TIME_STEP)
right_position_sensor.enable(TIME_STEP)

# assign actuators
# LEDs
left_led = boebot.getLED("left_led")
right_led = boebot.getLED("right_led")

# Ultrasonic sensors
left_distance_sensor = boebot.getDistanceSensor('left front sensor')
right_distance_sensor = boebot.getDistanceSensor('right front sensor')
#initialize sensors
left_distance_sensor.enable(TIME_STEP)
right_distance_sensor.enable(TIME_STEP)

# motors
left_motor = boebot.getMotor("left wheel motor")
right_motor = boebot.getMotor("right wheel motor")
# Initilaize motors - set positiont to infinity to control speed and set velocity to 0.
left_motor.setPosition(float('inf')) 
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# initialize variables
steps_counted = 0
time_running = 0.0 #[s]
left_position = 0.0 #[rad]
right_position = 0.0 #[rad]
state = "DRIVING"
speed = DRIVING_VELOCITY / WHEEL_RADIUS #[rad/s]
left_speed = speed #[rad/s]
right_speed = speed #[rad/s]
turning_distance = math.pi * TURN_RADIUS #[m] length of turn
left_led_value = 0
right_led_value = 0
distance_limit = LENGTH_BOARD #[m] set first limit to take action in state machine


# main loop
while boebot.step(TIME_STEP) != -1:
    # read sensors
    left_led_status = left_led.get()
    right_led_status = right_led.get()
    left_position = left_position_sensor.getValue()
    right_position = right_position_sensor.getValue()
    left_distance = left_distance_sensor.getValue()
    right_distance = right_distance_sensor.getValue()
        
    # create information from inputs (user functions)
    steps_counted += 1
    time_running = (steps_counted * TIME_STEP /1000) #[s]
    average_traveled_distance = (left_position * WHEEL_RADIUS + right_position * WHEEL_RADIUS) / 2 #[m]
    
    # take decisions (state machine)
    if state == "DRIVING":
        if average_traveled_distance >= distance_limit: # passed board
            state = "TURNING"
            left_led_value = 0
            # set speed for left and right wheel during turn [rad/s]
            left_speed = speed * ((TURN_RADIUS + TRACK_WIDTH /2) / TURN_RADIUS)
            right_speed = speed * ((TURN_RADIUS - TRACK_WIDTH /2) / TURN_RADIUS)
            # set new limit, add length of turn to current limit
            distance_limit += turning_distance
        else:
            left_led_value = Toggle(left_led_status)
    elif state == "TURNING":
        right_led_value = 1
        if average_traveled_distance >= distance_limit:
            state = "DRIVING"
            right_led_value = 0
            # set speed for both wheels to drive straight
            left_speed = speed
            right_speed = speed
            # set new limit, add length of board to current limit
            distance_limit += LENGTH_BOARD
        else:
            left_led_value = Toggle(left_led_status)
    else:
        break
    
    # compute behavior (user functions)
    
    
    # actuate console, LED and motors
    if steps_counted % (PRINT_INTERVAL / TIME_STEP) < 1:
        print("Step %4d, Time %.2fs, %s, Speed L %.2f, R %.2frad/s, Traveled %.3fm, Limit: %.3fm., Distance: L %.3fm., R %.3fm." 
            % (steps_counted, time_running, state, left_speed, right_speed, average_traveled_distance, 
            distance_limit, left_distance, right_distance))
     
    if steps_counted % (BLINK_INTERVAL / TIME_STEP) < 1:
        left_led.set(left_led_value)
        right_led.set(right_led_value)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)