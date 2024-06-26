#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
left_drive_smart = Motor(Ports.PORT1, 1.5, False)
right_drive_smart = Motor(Ports.PORT6, 1.5, True)

drivetrain = SmartDrive(left_drive_smart, right_drive_smart, brain_inertial, 200)
intake_motor_a = Motor(Ports.PORT5, True)
intake_motor_b = Motor(Ports.PORT11, False)
intake = MotorGroup(intake_motor_a, intake_motor_b)
lift_motor_a = Motor(Ports.PORT4, True)
lift_motor_b = Motor(Ports.PORT10, False)
lift = MotorGroup(lift_motor_a, lift_motor_b)
touchled_7 = Touchled(Ports.PORT7)
touchled_12 = Touchled(Ports.PORT12)



# Make random actually random
def setRandomSeedUsingAccel():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    urandom.seed(int(xaxis + yaxis + zaxis))
    
# Set random seed 
setRandomSeedUsingAccel()

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
# 	Author:       VEX
# 	Created:
# 	Description:  VEXcode IQ Python Project
# 
# ------------------------------------------


class KPID:
    def __init__(self, p, i, d, i_limit):
        # Takes in p, i, d, i_limit
        self.kp, self.ki, self.kd, self.ki_limit = p, i, d, i_limit

    def __repr__(self):
        return f"KPID({self.kp}, {self.ki}, {self.kd}, {self.ki_limit})"

    def __str__(self):
        return f"({self.kp}, {self.ki}, {self.kd}, {self.ki_limit})"

    def calc(self, p_mult, i_mult, d_mult):
        return p_mult * self.kp + i_mult * self.ki + d_mult * self.kd


class Robot:
    def __init__(self, forward_kpid, adjust_kpid, turn_kpid, gear_ratio, vel):
        self.forward_kpid = forward_kpid
        self.adjust_kpid = adjust_kpid
        self.turn_kpid = turn_kpid

        self.gear_ratio = gear_ratio
        self.form_factor = 548.64 * (1 / gear_ratio)
        self.default_vel = vel

        left_drive_smart.set_max_torque(100, PERCENT)
        right_drive_smart.set_max_torque(100, PERCENT)
        left_drive_smart.set_stopping(HOLD)
        right_drive_smart.set_stopping(HOLD)

        intake.set_max_torque(100, PERCENT)
        intake.set_stopping(COAST)

        lift.set_max_torque(100, PERCENT)
        lift.set_stopping(HOLD)

    def set_drivetrain_velocity(self, left, right):
        left_drive_smart.set_velocity(left, PERCENT)
        right_drive_smart.set_velocity(right, PERCENT)
    
    def drivetrain_drive(self, length, vel=None, time=None, straight=True):
        if vel is None:
            vel = self.default_vel

        start_rotation = brain_inertial.rotation()
        self.set_drivetrain_velocity(vel, vel)
        drivetrain.drive(FORWARD)

        while length * self.form_factor >= (left_drive_smart.position(DEGREES) + right_drive_smart.position(DEGREES)) / 2:
            current_rotation = brain_inertial.rotation()
            if current_rotation > start_rotation + 2:
                self.set_drivetrain_velocity(vel - 5, vel)
            elif current_rotation < start_rotation - 2:
                self.set_drivetrain_velocity(vel, vel - 5)
            else:
                self.set_drivetrain_velocity(vel, vel)
        
        drivetrain.stop()
        wait(10, MSEC)


GEAR_RATIO = 3 / 2
VELOCITY = 100
FORWARD_KPID = KPID()
ADJUST_KPID = KPID()
TURN_KPID = KPID()
robot = Robot(FORWARD_KPID, ADJUST_KPID, TURN_KPID, GEAR_RATIO, VELOCITY)
robot.drive_forward(10)
