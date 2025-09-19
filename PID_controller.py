#region VEXcode Generated Robot Configuration
from vex import *

# wait for rotation sensor to fully initialize
wait(30, MSEC)

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# auton selector
auton_started = False
autons = [
    "onboarding"
]
auton_num = len(autons)
current_auton_selection = 0

# Brain should be defined by default
brain = Brain()

# The controller
controller = Controller()

# Pneumatics + extras

# pid stuff
wheel_diameter = 2.75
ratio = 1
pid_tolerance = 0.1
pid_turn_tolerance = 0.1
pid_swing_tolerance = 0.1
wheel_base_width = 10  # Replace with your robot's actual drivetrain width in inches

# drive motors
left_drive_front = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
right_drive_front = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False) 
left_drive = MotorGroup(left_drive_front)
right_drive = MotorGroup(right_drive_front)

# EXAMPLE 6 Motor Drive
"""
left_drive_front = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
left_drive_back = Motor(Ports.PORT16, GearSetting.RATIO_6_1, True) 
left_drive_middle = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False)
left_drive = MotorGroup(left_drive_front, left_drive_back, left_drive_middle)
right_drive_front = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False) 
right_drive_back = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
right_drive_middle = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
right_drive = MotorGroup(right_drive_front, right_drive_back, right_drive_middle)
drivetrain = DriveTrain(left_drive, right_drive, 259.34, 295, 40, INCHES, ratio) # 319.19 if 4 inch wheel, 259.34 if 3.25, 219.44 if 2.75"""

# Miscellaneous motors or sensors
inertial = Inertial(Ports.PORT20)
intake = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)

# EXAMPLE
"""
intake = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
intake_boolean = True
arm_left = Motor(Ports.PORT18, GearSetting.RATIO_18_1, True)
arm_right = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
lady_brown = MotorGroup(arm_right, arm_left)
inertial = Inertial(Ports.PORT20)
distance_sensor = Distance(Ports.PORT1)
color_sort_distance_sensor = Distance(Ports.PORT2)
color_sort_switch = Limit(brain.three_wire_port.b)
color_sensor = Optical(Ports.PORT10)"""

# setting misc motors velo, stopping type, etc...


# global variables


# Auton Selector and stuff
def preauton():
    global auton_started, current_auton_selection, driver_started

    auton_started = False
    driver_started = False

    # Assuming a function to set font in Python, replace as needed
    brain.screen.set_font(FontType.MONO60)

    # resetting rotation sensor
    #rotation.set_position(0, DEGREES)

    # initialize inertial
    inertial.calibrate()

    while not auton_started and not driver_started:
        brain.screen.clear_screen()
        brain.screen.set_cursor(2, 1)
        brain.screen.print(autons[current_auton_selection])
        
        if brain.screen.pressing():
            while brain.screen.pressing():
                pass  # wait until finger lifted up from the screen
            current_auton_selection = (current_auton_selection + 1) % auton_num

        wait(50, MSEC)  # sleep for 50 ms

    brain.screen.set_font(FontType.MONO20)
    brain.screen.clear_screen()

# All motors are controlled from this function which is run as a separate thread
def drive_task():
    global intake_boolean
    global backClampPiston
    global setCornerClearer
    global driver_started

    # stop preauton screen
    driver_started = True

    wait(15, MSEC)

    while competition.is_driver_control() and competition.is_enabled():
        # Tank drive
        drive_right = controller.axis2.position()  # Right side is controlled by right axis
        drive_left = controller.axis3.position()  # Left side is controlled by left axis

        # Intake control
        if controller.buttonR1.pressing():
            intake.spin(FORWARD)
        elif controller.buttonR2.pressing():
            intake.spin(REVERSE)
        else:
            intake.stop()

        # Deadband logic for drive
        deadband = 15
        if abs(drive_left) < deadband:
            drive_left = 0
        if abs(drive_right) < deadband:
            drive_right = 0

        drive_left_voltage = PIDController.percent_to_voltage(drive_left)
        drive_right_voltage = PIDController.percent_to_voltage(drive_right)

        drive_left_voltages = drive_left_voltage
        drive_right_voltages =  drive_right_voltage

        left_drive_front.spin(FORWARD, drive_left_voltages, VOLT)
        left_drive_back.spin(FORWARD, drive_left_voltages, VOLT)
        left_drive_middle.spin(FORWARD, drive_left_voltages, VOLT)
        right_drive_front.spin(FORWARD, drive_right_voltages, VOLT)
        right_drive_back.spin(FORWARD, drive_right_voltages, VOLT)
        right_drive_middle.spin(FORWARD, drive_right_voltages, VOLT)

        wait(10, MSEC)

# PID STUFF
class PIDController:
    max_voltage = 12

    def __init__(self, p, i, d, start_i=0.0, reset_i_sgn=True):
        self.kP = p
        self.kI = i
        self.kD = d
        self.start_i = start_i  # start applying I only when error is small
        self.reset_i_sgn = reset_i_sgn

        self.target = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_measurement = 0.0
        self.output = 0.0

    def calculate(self, target, current_value):
        self.target = target
        self.error = target - current_value

        # Derivative on measurement to reduce "kick"
        self.derivative = current_value - self.prev_measurement

        # Integral accumulation with windup guard
        if abs(self.error) < self.start_i:
            self.integral += self.error
            self.integral = self.clamp(self.integral, -100, 100)  # anti-windup

        # Reset integral if error sign flips
        if (self.error > 0) != (self.prev_error > 0) and self.reset_i_sgn:
            self.integral = 0

        # PID output
        self.output = (self.kP * self.error +
                       self.kI * self.integral -
                       self.kD * self.derivative)

        # Update previous values
        self.prev_error = self.error
        self.prev_measurement = current_value

        return self.output

    @staticmethod
    def percent_to_voltage(percent):
        return percent / 100 * PIDController.max_voltage

    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min(value, max_val), min_val)

    @staticmethod
    def normalize_error(target_angle, current_angle):
        error = target_angle - current_angle
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error

    @staticmethod
    def normalize_heading(current_angle):
        if current_angle > 180:
            current_angle -= 360
        elif current_angle < -180:
            current_angle += 360
        return current_angle

def pid_turn_controller(degree, pid: PIDController, timeout_duration, min_volt=-12, max_volt=12):
    drivetrain.set_stopping(HOLD)
    brain.timer.reset()

    initial_rotation = inertial.rotation()
    target = initial_rotation + degree

    while True:
        current_degrees = inertial.rotation()
        pid_output = pid.calculate(target, current_degrees)
        pid_output = PIDController.clamp(pid_output, min_volt, max_volt)

        left_drive.spin(FORWARD, pid_output, VOLT)
        right_drive.spin(FORWARD, -pid_output, VOLT)

        if abs(pid.error) < pid_turn_tolerance:
            break

        if brain.timer.time(SECONDS) >= timeout_duration:
            break

        wait(10, MSEC)

    drivetrain.stop()

def turn_to_heading(degree, pid: PIDController, timeout_duration, min_volt=-12, max_volt=12):
    target_heading = degree % 360
    current_heading = inertial.heading()
    error = PIDController.normalize_error(target_heading, current_heading)

    pid_turn_controller(error, pid, timeout_duration, min_volt, max_volt)

def pid_controller(target_distance, pid: PIDController, timeout_duration, min_volt=-12, max_volt=12):
    # Reset motor positions
    left_drive_front.reset_position()
    left_drive_back.reset_position()
    left_drive_middle.reset_position()
    right_drive_front.reset_position()
    right_drive_back.reset_position()
    right_drive_middle.reset_position()

    drivetrain.set_stopping(HOLD)
    brain.timer.reset()

    while True:
        current_distance = get_current_distance()
        pid_output = pid.calculate(target_distance, current_distance)
        pid_output = PIDController.clamp(pid_output, min_volt, max_volt)

        left_drive.spin(FORWARD, pid_output, VOLT)
        right_drive.spin(FORWARD, pid_output, VOLT)

        if abs(pid.error) < pid_tolerance:
            break

        if brain.timer.time(SECONDS) >= timeout_duration:
            break

        wait(20, MSEC)

    drivetrain.stop()

def pid_swing_controller(deg, pid, timeout, direction=FORWARD, opposite_volt=0.0, min_volt=-12, max_volt=12):
    controller.screen.clear_row(1)
    # Initialize current degrees turned
    drivetrain.set_stopping(HOLD)
    brain.timer.reset()

    # current degrees
    initial_rotation = inertial.rotation()
    target = initial_rotation + deg

    while True:
        # Update current degrees
        current_degrees = inertial.rotation(DEGREES)

        # Calculate error using normalized error
        error = target - current_degrees

        controller.screen.set_cursor(1,1)
        controller.screen.print(error)
        
        # If the error is within tolerance, stop turning
        if abs(error) < pid_turn_tolerance:
            break

        # Calculate PID output based on the error
        if direction == FORWARD:
            pid_output = pid.calculate(target, current_degrees)
        else:
            pid_output = -pid.calculate(target, current_degrees)

        # Clamp output to motor limits
        pid_output = PIDController.clamp(pid_output, min_volt, max_volt)

        # Apply PID output to motors based on the direction and error sign
        if direction == FORWARD:
            if pid_output > 0:
                # Forward right swing
                left_drive.spin(FORWARD, pid_output, VOLT)
                right_drive.spin(FORWARD, opposite_volt, VOLT)
            elif pid_output < 0:
                # Forward left swing
                left_drive.spin(FORWARD, opposite_volt, VOLT)
                right_drive.spin(FORWARD, -pid_output, VOLT)
        else:  # direction == REVERSE
            if pid_output > 0:
                # Backward right swing
                left_drive.spin(REVERSE, pid_output, VOLT)
                right_drive.spin(REVERSE, opposite_volt, VOLT)
            elif pid_output < 0:
                # Backward left swing
                left_drive.spin(REVERSE, opposite_volt, VOLT)
                right_drive.spin(REVERSE, -pid_output, VOLT)
        controller.screen.set_cursor(2,1)
        controller.screen.print("Initial: " + str(initial_rotation % 360))

        # Timeout check
        if brain.timer.time(SECONDS) >= timeout:
            break

        wait(10, TimeUnits.MSEC)

    # Stop motors once target is reached or timeout occurs
    left_drive.stop()
    right_drive.stop()
    controller.screen.set_cursor(3,1)
    controller.screen.print("End: " + str(inertial.heading()))

def focused_pid_controller(target_distance, target_heading, linear_pid, angular_pid, timeout_duration, drive_max_voltage=12.0, heading_max_voltage=6.0, settle_error=0.5, settle_time=500):
    start_average_position = (get_current_distance_left() + get_current_distance_right()) / 2.0
    brain.timer.reset()
    settle_timer = 0

    while True:
        average_position = (get_current_distance_left() + get_current_distance_right()) / 2.0
        current_rotation = inertial.heading()

        drive_error = target_distance - (average_position - start_average_position)
        heading_error = PIDController.normalize_error(target_heading, current_rotation)

        drive_output = linear_pid.calculate(target_distance, average_position - start_average_position)
        heading_output = angular_pid.calculate(target_heading, current_rotation)

        drive_output = PIDController.clamp(drive_output, -drive_max_voltage, drive_max_voltage)
        heading_output = PIDController.clamp(heading_output, -heading_max_voltage, heading_max_voltage)

        left_drive.spin(FORWARD, drive_output + heading_output, VOLT)
        right_drive.spin(FORWARD, drive_output - heading_output, VOLT)

        if abs(drive_error) < settle_error and abs(heading_error) < settle_error:
            if settle_timer == 0:
                settle_timer = brain.timer.time(MSEC)
            elif brain.timer.time(MSEC) - settle_timer >= settle_time:
                break
        else:
            settle_timer = 0

        if brain.timer.time(SECONDS) >= timeout_duration:
            break

        wait(10, MSEC)

    drivetrain.stop()

def get_current_distance():
    # Get the average rotation from the motors
    average_rotation = (left_drive_middle.position(DEGREES) + left_drive_front.position(DEGREES) + left_drive_back.position(DEGREES) + right_drive_middle.position(DEGREES) + right_drive_front.position(DEGREES) + right_drive_back.position(DEGREES)) / 6
    distance_traveled = (average_rotation * (ratio) / 360) * wheel_diameter * math.pi # gets distance traveled in inches pi*d = distance
    return distance_traveled

def get_current_distance_left():
    # Average the positions of the left motors
    position_left_deg = (
        left_drive_front.position(DEGREES) +
        left_drive_back.position(DEGREES) +
        left_drive_middle.position(DEGREES)
    ) / 3
    # Convert degrees to inches
    return (position_left_deg * ratio / 360) * wheel_diameter * math.pi

def get_current_distance_right():
    # Average the positions of the right motors
    position_right_deg = (
        right_drive_front.position(DEGREES) +
        right_drive_back.position(DEGREES) +
        right_drive_middle.position(DEGREES)
    ) / 3
    # Convert degrees to inches
    return (position_right_deg * ratio / 360) * wheel_diameter * math.pi

pid = PIDController(p=0.78, i=0.0001, d=0.7)

def onboarding():
    pass

def autonomous():
    global current_auton_selection, auton_started
    auton_started = True
    controller.screen.clear_screen()
    # add as many autons as you like...
    # continue with elif statements and emumerate the number
    if current_auton_selection == 0:
        auton = Thread(onboarding)
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)

    auton.stop()

def draw_gif():
    while True:
        iterator = 0
        image_name = 0
        while iterator < 25: # replace this with the number of frames you have
            if image_name < 10:
                file_img = "0" + str(image_name) + ".png"
            else:
                file_img = str(image_name) + ".png"
            brain.screen.draw_image_from_file(file_img,0,0)
            iterator += 1
            image_name += 1
            wait(0.02, SECONDS)

# DEBUG PURPOSES
def debug_screen():
    brain.screen.clear_screen()
    while True:
        brain.screen.set_cursor(1, 1)
        brain.screen.print("IMU HEADING: " + str(inertial.heading()))

        brain.screen.set_cursor(2, 1)
        brain.screen.print("LEFT MOTORS: " + str(left_drive.position()))

        brain.screen.set_cursor(3, 1)
        brain.screen.print("RIGHT MOTORS: " + str(right_drive.position()))

# code initilization
competition = Competition(drive_task, autonomous)
preauton()
drawgif = Thread(debug_screen)

# Run the drive code
drive = Thread(drive_task)
