"""simple_joint_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import time

PLOT = True

import matplotlib.pyplot as plt
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

control_structures = [["hipx_a", "hipy_a", "leg_a"],
                      ["hipx_b", "hipy_b", "leg_b"],
                      ["hipx_c", "hipy_c", "leg_c"],
                      ["hipx_d", "hipy_d", "leg_d"]]


def set_motor_speed(motorName, speed):
    motor = robot.getMotor(motorName)
    motor.setPosition(float("inf"))
    motor.setVelocity(speed)

def set_motor_position(motorName, pos):
    motor = robot.getMotor(motorName)
    motor.setPosition(pos)

# set motor speed
def set_all_motors(speed):
    for structure in control_structures:
        for motor_name in structure:
            set_motor_speed(motor_name, speed)

def leg_forward(leg, speed, sleep = None):
    set_motor_speed("hipx_" + leg, speed)
    if sleep is not None:
        robot.step(sleep)

def leg_raise(leg, speed, sleep = None):
    set_motor_speed("hipy_" + leg, -abs(speed))
    if sleep is not None:
        robot.step(sleep)

def leg_plant(leg, sleep = None):
    motor = robot.getMotor("hipy_" + leg)
    motor.setPosition(0.7)
    #set_motor_position("hipy_" + leg, 0.5)
    if sleep is not None:
        robot.step(sleep)

def leg_raise_type1(leg):
    set_motor_speed("hipy_" + leg, -0.2)
    set_motor_speed("leg_" + leg, -0.5)
    set_motor_speed("hipx_" + leg, 0.2)
    robot.step(2000)

def gait_1():
    set_motor_position("hipy_c", 0)
    set_motor_position("hipy_d", 0)
    set_motor_position("hipy_a", 0)
    set_motor_position("hipy_b", 0)
    set_motor_position("leg_c", 1.57)
    set_motor_position("leg_d", 1.57)
    set_motor_position("leg_a", 1.57)
    set_motor_position("leg_b", 1.57)
    robot.step(2000)
    while True:
        set_motor_position("hipx_a", 0.5)
        set_motor_position("hipx_b", -0.5)
        robot.step(2000)
        set_motor_position("hipy_c", -0.5)
        set_motor_position("leg_c", 0.5)
        robot.step(2000)
        set_motor_position("hipx_a", -0.5)
        set_motor_position("hipx_b", 0.5)
        set_motor_position("hipy_c", 0)
        set_motor_position("leg_c", 1.57)
        robot.step(2000)
        set_motor_position("hipx_a", 0.5)
        set_motor_position("leg_a", 0.5)
        robot.step(500)
        set_motor_position("leg_a", 1.57)
        robot.step(2000)
        set_motor_position("hipx_b", -0.5)
        set_motor_position("leg_b", 0.5)
        robot.step(500)
        set_motor_position("leg_b", 1.57)

def get_tilt():
    RPT = imu.getRollPitchYaw()
    return abs(RPT[0]) + abs(RPT[1])

def record_tilt():
    tilt_data.append(get_tilt())

def get_tilt_data():
    return tilt_data

def plot_tilt():
    plt.plot(get_tilt_data())
    plt.ylabel('tile = abs(roll) + abs(pitch)')
    plt.show()

tilt_data = []

def gait_2():
    set_motor_position("hipy_c", 0)
    set_motor_position("hipy_d", 0)
    set_motor_position("hipy_a", 0)
    set_motor_position("hipy_b", 0)
    set_motor_position("leg_c", 1.57)
    set_motor_position("leg_d", 1.57)
    set_motor_position("leg_a", 1.57)
    set_motor_position("leg_b", 1.57)
    robot.step(2000)
    record_tilt()
    set_motor_position("hipx_a", 0.2)
    set_motor_position("hipx_b", -0.2)
    set_motor_position("leg_c", 1.57)
    set_motor_position("leg_d", 1.57)
    set_motor_position("leg_a", 1.57)
    set_motor_position("leg_b", 1.57)
    robot.step(2000)
    record_tilt()

    while True:
        set_motor_position("hipy_c", -0.5)
        set_motor_position("leg_c", 0.5)
        robot.step(2000)
        record_tilt()
        set_motor_position("hipx_a", -0.2)
        set_motor_position("hipx_b", 0.2)
        set_motor_position("leg_c", 1.57)
        set_motor_position("leg_d", 1.57)
        set_motor_position("leg_a", 1.57)
        set_motor_position("leg_b", 1.57)

        robot.step(2000)
        record_tilt()
        set_motor_position("hipx_c", -0.5)
        set_motor_position("hipx_d", 0.5)
        robot.step(2000)
        record_tilt()
        set_motor_position("hipy_a", -0.5)
        robot.step(500)
        record_tilt()
        set_motor_position("hipx_a", 0.5)
        robot.step(500)
        record_tilt()
        set_motor_position("hipy_a", -0.2)
        robot.step(2000)
        record_tilt()
        set_motor_position("hipx_c", 0)
        set_motor_position("hipx_d", 0)


        robot.step(2000)
        record_tilt()
        set_motor_position("hipx_c", 0.5)
        set_motor_position("hipx_d", -0.5)
        robot.step(2000)
        record_tilt()
        set_motor_position("hipy_b", -0.5)
        robot.step(500)
        record_tilt()
        set_motor_position("hipx_b", -0.5)
        robot.step(500)
        record_tilt()
        set_motor_position("hipy_b", 0)
        set_motor_position("hipx_c", 0)
        set_motor_position("hipx_d", 0)
        if PLOT: plot_tilt()

imu = robot.getInertialUnit("inertial unit")
imu.enable(200)
robot.step(2000)
print(imu.getRollPitchYaw())

# 0. get data
# 1. graph
# 2. make dynamic

# Next steps
# 0) update motor characteristics 
# 1) reduce tilt, motor usage, etc
# 2) test on uneven terrain

gait_2()

#set_motor_position("hipy_a", 0)
#set_motor_position("leg_a", 1.57)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
