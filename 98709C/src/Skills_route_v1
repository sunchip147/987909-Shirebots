
# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Sunch                                                        #
# 	Created:      9/27/2025, 11:32:50 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# --- VEX V5 API ---
# The vex module should be automatically available in a VEXcode V5 Python project.
from vex import *
import math
import random
import time

# --- Robot Setup ---
brain = Brain()
controller = Controller(PRIMARY)

# Drivetrain Motors
left_motor = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)
left_motor_2 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, True)
left_motor_3 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
right_motor = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
right_motor_2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
right_motor_3 = Motor(Ports.PORT21, GearSetting.RATIO_6_1, False)

#Intake/scoring motors
lift = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
claw_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
intake = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)

#toggle
toggle_piston = DigitalOut(brain.three_wire_port.a)

# Odometry Sensors
inertial_sensor = Inertial(Ports.PORT18)
forward_tracker = Rotation(Ports.PORT5, False)
sideways_tracker = Rotation(Ports.PORT17, False)

# --- Constants ---
PI = 3.14159265359

"""## Helper Functions"""

# --- Helper Functions ---
def reduce_angle_0_to_360(angle_degrees):
    """ Corrects an angle to be within the 0 to 360-degree range. """
    while not (0 <= angle_degrees < 360):
        if angle_degrees < 0:
            angle_degrees += 360
        elif angle_degrees >= 360:
            angle_degrees -= 360
    return angle_degrees

def reduce_angle_negative_180_to_180(angle_degrees):
    """ Corrects an angle to be within the -180 to 180-degree range. """
    while not (-180 <= angle_degrees < 180):
        if angle_degrees < -180:
            angle_degrees += 360
        elif angle_degrees >= 180:
            angle_degrees -= 360
    return angle_degrees

def reduce_angle_negative_90_to_90(angle_degrees):
    """ Corrects an angle to be within the -90 to 90-degree range. """
    while not (-90 <= angle_degrees < 90):
        if angle_degrees < -90:
            angle_degrees += 180
        elif angle_degrees >= 90:
            angle_degrees -= 180
    return angle_degrees

def cubic(input_val):
    """ A cubic function for smoother joystick control. """
    if input_val != 0:
        return (input_val ** 3) / (100 * abs(input_val))
    return 0

def throttle(input_val):
    if input_val != 0:
        return (input_val ** 5) / (10000 * input_val * input_val)
    return 0

def convert_to_degrees(angle_radians):
    """ Converts radians to degrees. """
    return angle_radians * 180 / PI

def convert_to_radians(angle_degrees):
    """ Converts degrees to radians. """
    return angle_degrees * PI / 180

def limit_input_min_and_max(input_val, min_val, max_val):
    """ Limits an input value to a specified minimum and maximum. """
    if input_val < min_val:
        return min_val
    elif input_val > max_val:
        return max_val
    return input_val

"""# PID Control"""

# --- PID Control ---
# Drive PID Variables
accumulated_drive_error = 0
previous_drive_error = 0
drive_settle_time_passed = 0
drive_pid_runtime = 0

# Drive PID Constants
drive_kp = 0.75
drive_ki = 0.002
drive_kd = 2

def drive_pid(drive_error, custom_drive_kp=drive_kp, custom_drive_ki=drive_ki, custom_drive_kd=drive_kd, drive_integral_threshold=10, drive_settle_error=2):
    global accumulated_drive_error, previous_drive_error, drive_settle_time_passed, drive_pid_runtime

    if abs(drive_error) < drive_integral_threshold:
        accumulated_drive_error += drive_error
    if drive_error * previous_drive_error < 0:
        accumulated_drive_error = 0

    drive_output = custom_drive_kp * drive_error + custom_drive_ki * accumulated_drive_error + custom_drive_kd * (drive_error - previous_drive_error)
    previous_drive_error = drive_error

    if abs(drive_error) < drive_settle_error:
        drive_settle_time_passed += 10
    else:
        drive_settle_time_passed = 0

    drive_pid_runtime += 10
    return drive_output

# Turn PID Variables
accumulated_turn_error = 0
previous_turn_error = 0
turn_settle_time_passed = 0
turn_pid_runtime = 0

# Turn PID Constants
turn_kp = 0.1
turn_ki = 0.0001
turn_kd = 0.65

def turn_pid(turn_error, custom_turn_kp=turn_kp, custom_turn_ki=turn_ki, custom_turn_kd=turn_kd, turn_integral_threshold=15, turn_settle_error=2):
    global accumulated_turn_error, previous_turn_error, turn_settle_time_passed, turn_pid_runtime

    if abs(turn_error) < turn_integral_threshold:
        accumulated_turn_error += turn_error
    if turn_error * previous_turn_error < 0:
        accumulated_turn_error = 0

    turn_output = custom_turn_kp * turn_error + custom_turn_ki * accumulated_turn_error + custom_turn_kd * (turn_error - previous_turn_error)
    previous_turn_error = turn_error

    if abs(turn_error) < turn_settle_error:
        turn_settle_time_passed += 10
    else:
        turn_settle_time_passed = 0

    turn_pid_runtime += 10
    return turn_output

#lift PID Constants
lift_kp = 0.5
lift_ki = 0.002
lift_kd = 1

def lift_pid(lift_error, lift_integral_threshold=10, lift_settle_error=2):
    global accumulated_lift_error, previous_lift_error, lift_settle_time_passed, lift_pid_runtime

    if abs(lift_error) < lift_integral_threshold:
        accumulated_lift_error += lift_error
    if lift_error * previous_lift_error < 0:
        accumulated_lift_error = 0

    lift_output = lift_kp * lift_error + lift_ki * accumulated_lift_error + lift_kd * (lift_error - previous_lift_error)
    previous_lift_error = lift_error

    if abs(lift_error) < lift_settle_error:
        lift_settle_time_passed += 10
    else:
        lift_settle_time_passed = 0

    lift_pid_runtime += 10
    return lift_output

# --- Drivetrain Control Functions ---
def drive_hold():
    left_motor.stop(HOLD)
    left_motor_2.stop(HOLD)
    left_motor_3.stop(HOLD)
    right_motor.stop(HOLD)
    right_motor_2.stop(HOLD)
    right_motor_3.stop(HOLD)

def drive_brake():
    left_motor.stop(BRAKE)
    left_motor_2.stop(BRAKE)
    left_motor_3.stop(BRAKE)
    right_motor.stop(BRAKE)
    right_motor_2.stop(BRAKE)
    right_motor_3.stop(BRAKE)

def drive_coast():
    left_motor.stop(COAST)
    left_motor_2.stop(COAST)
    left_motor_3.stop(COAST)
    right_motor.stop(COAST)
    right_motor_2.stop(COAST)
    right_motor_3.stop(COAST)

def set_drive_voltage(left_drive_volt, right_drive_volt=None):
    if right_drive_volt is None:
        right_drive_volt = left_drive_volt

    left_motor.spin(FORWARD, left_drive_volt, VOLT)
    left_motor_2.spin(FORWARD, left_drive_volt, VOLT)
    left_motor_3.spin(FORWARD, left_drive_volt, VOLT)
    right_motor.spin(FORWARD, right_drive_volt, VOLT)
    right_motor_2.spin(FORWARD, right_drive_volt, VOLT)
    right_motor_3.spin(FORWARD, right_drive_volt, VOLT)

"""## Odometry"""

# --- Odometry ---
# Tracking Sensor Variables
forwards_tracker_distance = 0.625
sideways_tracker_distance = -1.3125
forwards_tracker_inches_per_degree = 0.01745
sideways_tracker_inches_per_degree = 0.01745
previous_forward_tracker_reading = 0
previous_sideways_tracker_reading = 0

# Positioning Variables
absolute_global_X = 0
absolute_global_Y = 0
previous_global_X = 0
previous_global_Y = 0
absolute_heading_degrees = 0
absolute_heading_radians = 0
previous_heading_radians = 0

def update_tracking_sensors():
    global previous_forward_tracker_reading, previous_sideways_tracker_reading
    global absolute_heading_degrees, absolute_heading_radians, previous_heading_radians

    current_forward_tracker_reading = forward_tracker.position(DEGREES) * forwards_tracker_inches_per_degree
    current_sideways_tracker_reading = sideways_tracker.position(DEGREES) * sideways_tracker_inches_per_degree
    absolute_heading_degrees = reduce_angle_0_to_360(inertial_sensor.heading(DEGREES))
    absolute_heading_radians = convert_to_radians(absolute_heading_degrees)

    delta_forward_tracker = current_forward_tracker_reading - previous_forward_tracker_reading
    delta_sideways_tracker = current_sideways_tracker_reading - previous_sideways_tracker_reading
    delta_heading_radians = absolute_heading_radians - previous_heading_radians

    previous_forward_tracker_reading = current_forward_tracker_reading
    previous_sideways_tracker_reading = current_sideways_tracker_reading
    previous_heading_radians = absolute_heading_radians

    return delta_forward_tracker, delta_sideways_tracker, delta_heading_radians

def update_robot_position():
    global absolute_global_X, absolute_global_Y, previous_global_X, previous_global_Y, previous_heading_radians

    delta_forward_tracker, delta_sideways_tracker, delta_heading_radians = update_tracking_sensors()

    if delta_heading_radians == 0:
        local_x_position = delta_sideways_tracker
        local_y_position = delta_forward_tracker
    else:
        local_x_position = (2 * math.sin(delta_heading_radians / 2)) * ((delta_sideways_tracker / delta_heading_radians) + sideways_tracker_distance)
        local_y_position = (2 * math.sin(delta_heading_radians / 2)) * ((delta_forward_tracker / delta_heading_radians) + forwards_tracker_distance)

    if local_x_position == 0 and local_y_position == 0:
        local_polar_angle = 0
        local_polar_length = 0
    else:
        local_polar_angle = math.atan2(local_y_position, local_x_position)
        local_polar_length = math.sqrt(local_x_position**2 + local_y_position**2)

    global_polar_angle = local_polar_angle - previous_heading_radians - (delta_heading_radians / 2)

    delta_global_x = local_polar_length * math.cos(global_polar_angle)
    delta_global_y = local_polar_length * math.sin(global_polar_angle)

    absolute_global_X = previous_global_X + delta_global_x
    absolute_global_Y = previous_global_Y + delta_global_y

    previous_global_X = absolute_global_X
    previous_global_Y = absolute_global_Y

def set_position(current_x, current_y, current_heading_degrees):
    global previous_forward_tracker_reading, previous_sideways_tracker_reading
    global previous_global_X, previous_global_Y, absolute_global_X, absolute_global_Y
    global absolute_heading_degrees, absolute_heading_radians, previous_heading_radians

    forward_tracker.set_position(0, DEGREES)
    sideways_tracker.set_position(0, DEGREES)
    inertial_sensor.set_heading(current_heading_degrees, DEGREES)

    previous_forward_tracker_reading = 0
    previous_sideways_tracker_reading = 0

    update_tracking_sensors()
    update_robot_position()

    previous_global_X = current_x
    previous_global_Y = current_y
    absolute_global_X = current_x
    absolute_global_Y = current_y
    absolute_heading_degrees = current_heading_degrees
    absolute_heading_radians = convert_to_radians(current_heading_degrees)
    previous_heading_radians = absolute_heading_radians

def track_position():
    while True:
        update_robot_position()
        brain.screen.set_cursor(2, 2)
        brain.screen.print("X Position: {:.2f}".format(absolute_global_X))
        brain.screen.set_cursor(4, 2)
        brain.screen.print("Y Position: {:.2f}".format(absolute_global_Y))
        time.sleep(0.01)

def drive_to(desired_x_position, desired_y_position, drive_max_voltage=8, turn_max_voltage=6, drive_settle_time=500, drive_timeout=2000):
    global drive_settle_time_passed, drive_pid_runtime

    drive_settle_time_passed = 0
    drive_pid_runtime = 0

    while drive_settle_time_passed < drive_settle_time and drive_pid_runtime < drive_timeout:
        drive_error = math.sqrt((desired_x_position - absolute_global_X)**2 + (desired_y_position - absolute_global_Y)**2)
        turn_error = reduce_angle_negative_180_to_180(convert_to_degrees(math.atan2(desired_x_position - absolute_global_X, desired_y_position - absolute_global_Y)) - absolute_heading_degrees)

        drive_output = drive_pid(drive_error)

        heading_scale_factor = math.cos(convert_to_radians(turn_error))
        drive_output *= heading_scale_factor

        turn_error = reduce_angle_negative_90_to_90(turn_error)
        turn_output = turn_pid(turn_error)

        drive_settle_error = 2  # This should be defined or passed as an argument
        if drive_error < drive_settle_error:
            turn_output = 0

        drive_output = limit_input_min_and_max(drive_output, -abs(heading_scale_factor) * drive_max_voltage, abs(heading_scale_factor) * drive_max_voltage)
        turn_output = limit_input_min_and_max(turn_output, -turn_max_voltage, turn_max_voltage)

        set_drive_voltage(drive_output + turn_output, drive_output - turn_output)
        time.sleep(0.01)

    drive_hold()

def turn_to_angle(desired_angle, turn_max_voltage=6, turn_settle_time=500, turn_timeout=2000):
    global turn_settle_time_passed, turn_pid_runtime

    turn_settle_time_passed = 0
    turn_pid_runtime = 0

    while turn_settle_time_passed < turn_settle_time and turn_pid_runtime < turn_timeout:
        turn_error = reduce_angle_negative_180_to_180(desired_angle - absolute_heading_degrees)
        turn_output = turn_pid(turn_error)
        turn_output = limit_input_min_and_max(turn_output, -turn_max_voltage, turn_max_voltage)
        set_drive_voltage(turn_output, -turn_output)
        time.sleep(0.01)

    drive_hold()

def turn_to_point(desired_x_position, desired_y_position, turn_max_voltage=6, turn_settle_time=500, turn_timeout=2000):
    global turn_settle_time_passed, turn_pid_runtime

    turn_settle_time_passed = 0
    turn_pid_runtime = 0

    while turn_settle_time_passed < turn_settle_time and turn_pid_runtime < turn_timeout:
        turn_error = reduce_angle_negative_180_to_180(convert_to_degrees(math.atan2(desired_x_position - absolute_global_X, desired_y_position - absolute_global_Y)) - absolute_heading_degrees)
        turn_output = turn_pid(turn_error)
        turn_output = limit_input_min_and_max(turn_output, -turn_max_voltage, turn_max_voltage)
        set_drive_voltage(turn_output, -turn_output)
        time.sleep(0.01)

    drive_hold()

def claw(state):
    if state:
        claw_motor.spin(FORWARD, 12, PERCENT)
    else:
        claw_motor.spin(REVERSE, 12, PERCENT)

def toggle(toggle_count):
    for _ in range(toggle_count):
        toggle_piston.set(True)
        time.sleep(0.5)
        toggle_piston.set(False)
        time.sleep(0.5)

def lift_to_position(target_position, lift_max_voltage=8, lift_settle_time=500, lift_timeout=2000):
    global accumulated_lift_error, previous_lift_error, lift_settle_time_passed, lift_pid_runtime

    accumulated_lift_error = 0
    previous_lift_error = 0
    lift_settle_time_passed = 0
    lift_pid_runtime = 0

    while lift_settle_time_passed < lift_settle_time and lift_pid_runtime < lift_timeout:
        lift_error = target_position - lift.position(DEGREES)
        lift_output = lift_pid(lift_error)
        lift_output = limit_input_min_and_max(lift_output, - lift_max_voltage, lift_max_voltage)
        lift.spin(FORWARD, lift_output, VOLT)
        time.sleep(0.01)

    lift.stop(HOLD)

def scoreGroup(groupNumber):
    lift_to_position(90*groupNumber) #Placeholder values, will need to be tuned
    claw(True)
    time.sleep(0.5)
    lift_to_position(0)

def dropPin():
    lift_to_position(45) #Placeholder value, will need to be tuned, used for the general lifting position for dropping just a pin into a goal.
    time.sleep(0.5) 
    claw(False) #Opens the claw to drop the pin, will need to be tuned to make sure it drops the pin without dropping the whole group.
    time.sleep(0.5)

def grabGroup():
    time.sleep(0.5)
    claw(True)
    lift.spin(FORWARD, 12, PERCENT)

"""## Autonomous Code"""

# --- Main Program ---
def pre_autonomous():
    brain.screen.clear_screen()
    brain.screen.print("Pre-auton setup")
    inertial_sensor.calibrate()

def autonomous(preset):
    brain.screen.print("yolo")
    
    intake.spin(FORWARD,95,PERCENT)
# Toggles bottom toggle to red   
    toggle(2)
# Drives to red goal and deposits yellow preload
    turn_to_point(-24, 24)
    drive_to(-20,20)
    dropPin()
# Drives to yellow pin & cone  and picks it up
    turn_to_point(-24,0)
    drive_to(-20, 4)
    grabGroup()
# Scores yellow pin and cone
    turn_to_point(-20, 20)
    drive_to(-20,20)
    turn_to_point(-24, 24)
    scoreGroup(1)
# Drives to other yellow pin & cone and grabs it
    turn_to_point(-44,22)
    drive_to(-44,22)
    turn_to_point(-48,24)
    grabGroup()
# Turns back to goal, drives to it, then scores
    turn_to_point(-28,20)
    drive_to(-28,20)
    turn_to_point(-24,24)
    scoreGroup(2)
# Moves to last yellow pin & cone and grabs it
    turn_to_point(-28,48)
    drive_to(-28,44)
    turn_to_point(-24,48)
    grabGroup()
# Drives to goal and scores pin & cone set
    turn_to_point(-24,24)
    drive_to(-26,26)
    turn_to_point(24,-24)
    scoreGroup(3)
# Drives to matchload to grab pin, yellow up
    #matchload
    turn_to_point(-72,12)
    drive_to(-70,12)
    grabPin()
# Turns to next goal and drives to it
    turn_to_point(-48,48)
    drive_to(-50,46)
# Scores and drives to next pin & cone
    dropPin()
    turn_to_point(-72,48)
    drive_to(-70,48)
# Grabs the pin & cone and scores   
    grabGroup()
    turn_to_point(-70,48)
    drive_to(-70,48) 
    turn_to_point(-72,48)
    scoreGroup(1)
# Toggles!
    turn_to_point(-72,65)
    drive_to(-72,65)
    toggle(2)
# grabs pin & cup and scores it
    turn_to_point(-72, 96)
    grabPin()
    turn_to_point(-48,96)
    drive_to(-50,96)
    dropPin()
# grabs cup then pin and scores it
    turn_to_point(-50,130)
    drive_to(-50,126)
    turn_to_point(-50, 120)
    drive_to(-50,130)
    turn_to_point(-50,96)
    drive_to(-50,96)
    turn_to_point(-48,96)
    dropPin()
# grabs another group of pins & cups and scores
    drive_to(-50, 126)
    turn_to_point(-58,130)
    drive_to(-48,130)
    turn_to_point(-48,96)
    drive_to(-72,130)
    drive_to(-72,100)
    scoreGroup(1)
# grabs pin & cone to the right of the goal.
    drive_to(-26,100)
    turn_to_point(-26,120)
    drive_to(-26,96)
    turn_to_point(-48,96) 
    drive_to(-24,96)
    drive_to(-44,96)
    scoreGroup(2)
# crosses to blue and grabs the top pin and scores
    turn_to_point(-24,100) 
    drive_to(-22,100)
    turn_to_angle(180)
    drive_to(-24,100)
    turn_to_point(-24,130)
    drive_to(-22,130)
    dropPin()
# goes around the goal and grabs pin then a cone on wall
    drive_to(-36,130)
    drive_to(-24,150)
    grabGroup()
    turn_to_point(-24,130)
    drive_to(-20,134)
    turn_to_point(-24,130)
    scoreGroup(1)
# grabs cone then last pin from plus and scores
    turn_to_point(-23,154)
    drive_to(-23,152)
    grabGroup()
    turn_to_point(-16, 96)
    drive_to(-16, 96)
    turn_to_point(-20,96)
    turn_to_angle(180)
    drive_to(-20,96)
    turn_to_point(-24,130)
    drive_to(-22,130)
    scoreGroup()
# Toggles!
    turn_to_point(0,144)
    drive_to(0, 139)
    toggle(2)
# Grab center pin and score in blue goal
    turn_to_point(0, 96)
    drive_to(0, 100)
    grabGroup()
    turn_to_point(24,120)
    drive_to(22,120)
    scoreGroup(1)
# Drive up and grab pin group to score
    turn_to_point(24,144)
    drive_to(22,140)
    grabGroup()
    turn_to_point(24,120)
    drive_to(22,116)
    scoreGroup(2)
# Drive down and grab pin group to score
    turn_to_point(24,96)
    drive_to(22,96)
    grabGroup()
    turn_to_point(24,120)
    drive_to(22,96)
    scoreGroup(2)
# Drive right and grab pin group to score
    drive_to(28,116)
    turn_to_point(48,120)
    drive_to(46,116)
    grabGroup()
    turn_to_point(24, 120)
    drive_to (26, 116)
    scoreGroup(3)
# Drive to top right toggle and grab match load
    turn_to_point(72,134)
    drive_to(72,134)
    grabGroup
# Drive to blue goal and score pin
    turn_to_point(48,96)
    drive_to(48,96)
    scoreGroup(0)
# Drive to wall to grab group then score
    turn_to_point(72,96)
    drive_to(72,96)
    grabGroup
    turn_to_point(48,96)
    drive_to(48,96)
    scoreGroup
# Toggles!
    turn_to_point(72,72)
    drive_to(70,70)
    toggle(2)
# Drive to right plus pin to pick it up and score
    turn_to_point(48,24)
    drive_to(48,28)
    grabGroup
    turn_to_point(48,48)
    drive_to(44,44)
    scoreGroup(0)
# Grab pin group off wall and score
    turn_to_point(72,48)
    drive_to(72,48)
    grabGroup
    turn_to_point(48,48)
    drive_to(48,44)
    scoreGroup(1)
# Grab remaining cup off wall and grab top plus pin
    turn_to_point(72,50)
    drive_to(72,48)
    grabGroup
    turn_to_point(24,48)
    drive_to(24,50)
    grabGroup
    turn_to_point(48,48)
    drive_to(48,46)
    scoreGroup(2)
# Grab right blue off top plus and score
    turn_to_point(24,48)
    drive_to(28,48)
    grabGroup
    turn_to_point(24,24)
    drive_to(24,28)
    scoreGroup(0)
# Grab bottom red group and score
    turn_to_point(24,48)
    drive_to(24,44)
    grabGroup
    turn_to_point(24,24)
    drive_to(24,28)
    scoreGroup(1)
# Grab group by the wall and score
    turn_to_point(24,0)
    drive_to(24,4)
    grabGroup
    turn_to_point(24,24)
    drive_to(24,20)
    scoreGroup(2)
# DONE
    







        
"""### User Control"""

def user_control():
    # Make sure to start the position tracking thread
    track_position_thread = Thread(track_position)
    set_position(0, 0, 0)

    while True:
        drive_forward = cubic(controller.axis3.position(PERCENT))
        drive_turn = 0.6 * cubic(controller.axis1.position(PERCENT))

        left_speed = 0.75 * (drive_forward + drive_turn)
        right_speed = 0.75 * (drive_forward - drive_turn)

        left_motor.spin(FORWARD, left_speed, PERCENT)
        right_motor.spin(FORWARD, right_speed, PERCENT)
        left_motor_2.spin(FORWARD, left_speed, PERCENT)
        right_motor_2.spin(FORWARD, right_speed, PERCENT)
        left_motor_3.spin(FORWARD, left_speed, PERCENT)
        right_motor_3.spin(FORWARD, right_speed, PERCENT)

        time.sleep(0.02)

"""#### Comp Code * ***Don't Touch*** *"""

# Competition setup
competition = Competition(user_control, autonomous)
pre_autonomous()