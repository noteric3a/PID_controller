#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
distance_21 = Distance(Ports.PORT21)


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

#endregion VEXcode Generated Robot Configuration
# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()

# The controller
controller = Controller()

# Pneumatics + extras
back_wings = DigitalOut(brain.three_wire_port.a) # back_WINGSSSS

# Drive motors
# YOU ARE IN FRONT OF THE INTAKE
# port 12 is the radio
left_drive_front = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
left_drive_back = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)  # cable needs to be switched
left_drive_middle = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
left_drive = MotorGroup(left_drive_front, left_drive_back, left_drive_middle)
right_drive_front = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False) # cable needs to be switched
right_drive_back = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
right_drive_middle = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
right_drive = MotorGroup(right_drive_front, right_drive_back, right_drive_middle)
drivetrain = DriveTrain(right_drive, left_drive, 319.19, 295, 40, INCHES, 0.75)

# Miscellaneous motors
slapper = Motor(Ports.PORT2, GearSetting.RATIO_36_1, False) # needs the cable
intake = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
inertial = Inertial(Ports.PORT15)
distance_sensor = Distance(Ports.PORT21)

# setting misc motors velo
intake.set_velocity(100, PERCENT)
slapper.set_velocity(100, PERCENT)

# All motors are controlled from this function which is run as a separate thread
def drive_task():
    drivetrain.set_stopping(COAST)

    drive_left = 0
    drive_right = 0

    global setPiston
    global setPiston2
    global slapperBool
    back_wings.set(False)
    setPiston = True
    slapperBool = True

    # System event handlers
    controller.buttonL2.pressed(back_wings_set) # toggle system for the back_wings
    controller.buttonA.pressed(slapper_toggle) # slapper

    wait(15, MSEC)

    while competition.is_driver_control() and competition.is_enabled():

        # tank drive
        drive_right = controller.axis2.position() # left side is controlled by left axis
        drive_left = controller.axis3.position() # right side is controlled by right axis

        if controller.buttonR1.pressing():
            intake.spin(FORWARD)
        elif controller.buttonR2.pressing():
            intake.spin(REVERSE)
        else:
            intake.stop()

        deadband = 15
        if abs(drive_left) < deadband:
            drive_left = 0
        if abs(drive_right) < deadband:
            drive_right = 0

        left_drive_front.spin(FORWARD, drive_left, PERCENT)
        left_drive_back.spin(FORWARD, drive_left, PERCENT)
        right_drive_front.spin(FORWARD, drive_right, PERCENT)
        right_drive_back.spin(FORWARD, drive_right, PERCENT)
        left_drive_middle.spin(FORWARD, drive_left, PERCENT)
        right_drive_middle.spin(FORWARD, drive_right, PERCENT)

        wait(10, MSEC)

def back_wings_set():
    global setPiston
    if setPiston:
        back_wings.set(True)
        setPiston = False
    else:
        back_wings.set(False)
        setPiston = True

def slapper_toggle():
    global slapperBool
    if slapperBool:
        slapper.spin(FORWARD)
        slapperBool = False
    else:
        slapper.stop()
        slapperBool = True

class PIDController: 
    def __init__(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d
        self.previous_error = 0
        self.integral = 0

    def calculate(self, distance_final, distance_traveled): 
        # distance_final is the distance we want to travel
        # distance_traveled is the current position
        # error is the difference between these so we can calculate the necessary velo percentage
        error = distance_final - distance_traveled
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return (self.kP * error) + (self.kI * self.integral) + (self.kD * derivative)

def pid_controller(target_distance, pid):
    global left_drive_front, left_drive_middle, left_drive_back, right_drive_front, right_drive_middle, right_drive_back

    # Reset the motors to 0 position
    left_drive_front.reset_position()
    left_drive_back.reset_position()
    left_drive_middle.reset_position()
    right_drive_front.reset_position()
    right_drive_back.reset_position()
    right_drive_middle.reset_position()

    # Initialize current distance traveled
    current_distance = 0

    drivetrain.set_stopping(HOLD)

    # Loop until the error (difference between target and current distance) is small enough
    while abs(target_distance - current_distance) > 0.1:
        current_distance = get_current_distance()
        pid_output = pid.calculate(target_distance, current_distance)

        # Apply PID output to drivetrain, handling both forward and backward directions
        if pid_output > 0:
            controller.screen.set_cursor(1,10)
            controller.screen.print("FORWARD")
        else:
            controller.screen.set_cursor(1,10)
            controller.screen.print("REVERSE")

        drivetrain.set_drive_velocity(pid_output, PERCENT) # Ensure the velocity is positive
        controller.screen.set_cursor(1,1)
        controller.screen.print(pid_output)
        drivetrain.drive(FORWARD)

    drivetrain.stop()


def get_current_distance():
    # Get the average rotation from the motors
    average_rotation = (left_drive_middle.position(DEGREES) + right_drive_middle.position(DEGREES)) / 2
    distance_traveled = (average_rotation * (3/4)  / 360) * 2.75 * 3.14159 # gets distance traveled in inches pi*d = distance
    return distance_traveled

def swing_right_forward(left, right, degrees):
    # Set initial heading to an absolute value
    inertial.set_heading(20, DEGREES)
    initial = inertial.heading(DEGREES)
    distance_turned = 0  # Initialize distance turned for tracking

    # Configure motor settings
    right_drive.set_stopping(HOLD)
    right_drive.set_velocity(right, PERCENT)
    left_drive.set_stopping(HOLD)
    left_drive.set_velocity(left, PERCENT)

    # Display initial heading on the screen
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)

    # Perform the swing turn until the desired angle is reached
    while distance_turned < degrees - 0.5:
        drivetrain.drive(FORWARD)
        distance_turned = abs(inertial.heading(DEGREES) - initial)
        brain.screen.print(inertial.heading(DEGREES))
        brain.screen.set_cursor(2, 1)

    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(3, 1)

def swing_right_backward(left, right, degrees):
    # Set initial heading to an absolute value
    inertial.set_heading(350, DEGREES)
    initial = inertial.heading(DEGREES)
    distance_turned = 0  # Initialize distance turned for tracking

    # Configure motor settings
    right_drive.set_stopping(HOLD)
    right_drive.set_velocity(right, PERCENT)
    left_drive.set_stopping(HOLD)
    left_drive.set_velocity(left, PERCENT)

    # Display initial heading on the screen
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)

    # Perform the swing turn until the desired angle is reached
    while distance_turned < degrees - 0.5:
        drivetrain.drive(REVERSE)
        distance_turned = abs(inertial.heading(DEGREES) - initial)
        brain.screen.print(inertial.heading(DEGREES))
        brain.screen.set_cursor(2, 1)

    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(3, 1)

def swing_left_forward(left, right, degrees):
    # Set initial heading to an absolute value
    inertial.set_heading(350, DEGREES)
    initial = inertial.heading(DEGREES)
    distance_turned = 0  # Initialize distance turned for tracking

    # Configure motor settings
    right_drive.set_stopping(HOLD)
    right_drive.set_velocity(right, PERCENT)
    left_drive.set_stopping(HOLD)
    left_drive.set_velocity(left, PERCENT)

    # Display initial heading on the screen
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)

    # Perform the swing turn until the desired angle is reached
    while distance_turned < degrees - 0.5:
        drivetrain.drive(FORWARD)
        distance_turned = abs(inertial.heading(DEGREES) - initial)
        brain.screen.print(inertial.heading(DEGREES))
        brain.screen.set_cursor(2, 1)

    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(3, 1)

def swing_left_backward(left, right, degrees):
    # Set initial heading to an absolute value
    inertial.set_heading(20, DEGREES)
    initial = inertial.heading(DEGREES)
    distance_turned = 0  # Initialize distance turned for tracking

    # Configure motor settings
    right_drive.set_stopping(HOLD)
    right_drive.set_velocity(right, PERCENT)
    left_drive.set_stopping(HOLD)
    left_drive.set_velocity(left, PERCENT)

    # Display initial heading on the screen
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)

    # Perform the swing turn until the desired angle is reached
    while distance_turned < degrees - 0.5:
        drivetrain.drive(REVERSE)
        distance_turned = abs(inertial.heading(DEGREES) - initial)
        brain.screen.print(inertial.heading(DEGREES))
        brain.screen.set_cursor(2, 1)

    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(3, 1)

def turn_right(degrees):
    inertial.set_heading(20,DEGREES)
    brain.screen.set_cursor(1, 1)
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)
    distance_turned = inertial.heading(DEGREES) - 20
    while distance_turned < degrees:
        drivetrain.turn(LEFT)
        distance_turned = inertial.heading(DEGREES) - 20
    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)

def turn_left(degrees):
    inertial.set_heading(350,DEGREES)
    brain.screen.set_cursor(1, 1)
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)
    distance_turned = inertial.heading(DEGREES) - 350
    while distance_turned < degrees:
        drivetrain.turn(RIGHT)
        distance_turned = 350 - inertial.heading(DEGREES)
    drivetrain.stop()
    brain.screen.print(inertial.heading(DEGREES))
    brain.screen.set_cursor(2, 1)


def auto():
    pid = PIDController(p=3, i=0.0006, d=1)
    inertial.set_heading(90, DEGREES)
    while competition.is_autonomous() and competition.is_enabled():   
        pid_controller(-20, pid)

        break

def autonomous():
    auton_task_0 = Thread(auto)

    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)

    auton_task_0.stop()

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

# code initilization
competition = Competition(drive_task, autonomous)

# Run the drive code
#drawgif = Thread(draw_gif)
drive = Thread(drive_task)
