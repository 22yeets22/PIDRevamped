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


class DictWrapper:
    def __init__(self, dictionary):
        self.__dict__ = dictionary


class Robot:
    def __init__(self, forward_kpid, adjust_kpid, turn_kpid, gear_ratio, vel, exit_tolerance_in=0.25, exit_tolerance_deg=5, exit_tolerance_count=2, stall_tolerance_vel=10, stall_tolerance_count=15):
        self.forward_kpid = forward_kpid
        self.adjust_kpid = adjust_kpid
        self.turn_kpid = turn_kpid

        self.gear_ratio = gear_ratio
        self.form_factor = 548.64 * (1 / gear_ratio)
        self.default_vel = vel

        self.wheel_circumference_mm = 200  # Wheel circumference in mm

        # Driving straight variables
        self.drive_variables = DictWrapper({
            'exit_tolerance': self.inches_to_degrees(exit_tolerance_in),
            'exit_tolerance_deg': exit_tolerance_deg,
            'exit_tolerance_count': exit_tolerance_count,
            'stall_tolerance_vel': stall_tolerance_vel,
            'stall_tolerance_count': stall_tolerance_count
        })

        '''self.exit_tolerance = self.inches_to_degrees(exit_tolerance_in)
        self.exit_tolerance_deg = exit_tolerance_deg
        self.stall_tolerance_vel = 10
        self.stall_tolerance_count = 15'''

        # Initalize motor
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
    
    def inches_to_degrees(self, inches):
        circumference_in_inches = self.wheel_circumference_mm / 25.4  # Convert mm to inches
        revolutions = inches / circumference_in_inches
        degrees = revolutions * 360 * self.gear_ratio
        return degrees
    
    def degrees_to_inches(self, degrees):
        circumference_in_inches = self.wheel_circumference_mm / 25.4  # Convert mm to inches
        revolutions = degrees / (360 * self.gear_ratio)
        inches = revolutions * circumference_in_inches
        return inches

    def drivetrain_drive(self, length, target_angle=None, vel=None, time=None, straight=True):
        if vel is None:
            vel = self.default_vel

        length_degrees = self.inches_to_degrees(length)

        stopped = 0  # Counter for how long it takes to stop
        stall = 0  # Counter for how long the robot is stalling

        fwd_p = None  # Positional value (the sensor value - desired value)
        fwd_prev_p = 0  # Error 20 MS ago
        fwd_i = 0  # Total error is all the error, integral
        fwd_d = None  # Speed, error - previousError

        adj_p = None  # Positional value, sensorvalue - desiredvalue
        adj_prev_p = 0  # Position 20 MS ago
        adj_i = 0  # Total error is all the error
        adj_d = None  # Speed, error - previousError

        if target_angle is None:
            target_angle = brain_inertial.rotation()

        left_drive_smart.set_position(0, DEGREES)
        right_drive_smart.set_position(0, DEGREES)

        # Drive forward
        left_drive_smart.spin(FORWARD)
        right_drive_smart.spin(FORWARD)
        while True:
            # Forward section PID            
            average_position = (left_drive_smart.position(DEGREES) + right_drive_smart.position(DEGREES)) / 2

            fwd_p = length_degrees - average_position  # Positional (error for lateral / driving forward)
            fwd_i += fwd_p  # Integral: velocity, position, absement
            if fwd_i > self.forward_kpid.ki_limit:
                fwd_i = self.forward_kpid.ki_limit
            elif fwd_i < -self.forward_kpid.ki_limit:
                fwd_i = -self.forward_kpid.ki_limit
            if fwd_prev_p is not None and ((fwd_p > 0 and fwd_prev_p < 0) or (fwd_p < 0 and fwd_prev_p > 0)):
                fwd_i = 0  # Reset integral when passes setpoint (let proportional cook!)
            fwd_d = fwd_p - fwd_prev_p  # Derivative: Similar to brakes and nitro
            forward_motor_power = self.forward_kpid.calc(fwd_p, fwd_i, fwd_d)

            # Adjust section PID
            adj_p = target_angle - brain_inertial.rotation()  # Positional (error for turning)
            adj_i += adj_p  # Intergral: velocity, position, absement
            if adj_i > self.forward_kpid.ki_limit:
                adj_i = self.forward_kpid.ki_limit
            elif adj_i < -self.forward_kpid.ki_limit:
                adj_i = -self.forward_kpid.ki_limit
            adj_d = adj_p - adj_prev_p  # Derivative: Similar to speed / velocity in turn error
            adjust_motor_power = self.adjust_kpid.calc(adj_p, adj_i, adj_d)

            # Spinning motors
            if forward_motor_power > 100 - adjust_motor_power:
                forward_motor_power = 100 - adjust_motor_power
            print("Motor Powers:", forward_motor_power * vel / 100, adjust_motor_power * vel / 100)
            left_drive_smart.set_velocity((forward_motor_power - adjust_motor_power) * vel / 100, PERCENT)
            right_drive_smart.set_velocity((forward_motor_power + adjust_motor_power) * vel / 100, PERCENT)

            fwd_prev_p = fwd_p
            adj_prev_p = adj_p

            # Check for stall and stop
            print()

            if drivetrain.current(CurrentUnits.AMP) > 1 and (abs(left_drive_smart.velocity(PERCENT)) + abs(right_drive_smart.velocity(PERCENT))) / 2 < self.drive_variables.stall_tolerance_vel:
                stall += 1
            else:
                stall = 0
            if stall > self.drive_variables.stall_tolerance_count:
                print("\nStalled")
                break

            # Is it at the goal
            if abs(fwd_p) < self.drive_variables.exit_movement and abs(adj_p) < self.drive_variables.exit_movement_deg: stopped += 1
            else: stopped = 0
            if self.drive_variables.exit_tolerance_count < stopped: break

            wait(20, MSEC)
        
        left_drive_smart.set_velocity(100, PERCENT)
        right_drive_smart.set_velocity(100, PERCENT)
        left_drive_smart.stop()
        right_drive_smart.stop()

        wait(30, MSEC)

        return fwd_p, adj_p


GEAR_RATIO = 3 / 2
VELOCITY = 100
FORWARD_KPID = KPID(1, 0, 0, 1000)
ADJUST_KPID = KPID(1, 0, 0, 1000)
TURN_KPID = KPID(1, 0, 0, 1000)
robot = Robot(FORWARD_KPID, ADJUST_KPID, TURN_KPID, GEAR_RATIO, VELOCITY)
robot.drivetrain_drive(12)
