# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab1_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math

from controller import Robot

os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot


def getDistanceReadings():
    # sets readings when robot is in a straight orientation/ parallel to the wall

    # If robot is not parallel to wall, readings come from lidar, where left and right have two separate readings
    Front_Left = robot.get_lidar_range_image()[225]
    Rear_Left = robot.get_lidar_range_image()[175]
    Front_Right = robot.get_lidar_range_image()[575]
    Rear_Right = robot.get_lidar_range_image()[625]
    Average_Front_Distance = (robot.get_lidar_range_image()[400])
    Center_Robot_Distances = [round(Front_Left, 3), round(Rear_Left, 3), round(Average_Front_Distance, 3),
                              round(Front_Right, 3),
                              round(Rear_Right, 3)]
    return Center_Robot_Distances


def wallFollowingLeft(leftError, maxVelocity, Cmax, Cmin, Kps):
    Utl = abs(leftError) * Kps
    print(Utl)

    print("Saturation Left:", SaturationFunction(Utl, Cmax, Cmin))

    if leftError < 0:
        SetAngularVelocity(maxVelocity-SaturationFunction(Utl, Cmax, Cmin), maxVelocity)
    else:
        SetAngularVelocity(maxVelocity, maxVelocity-SaturationFunction(Utl, Cmax, Cmin))


def WallFollowingRight(rightError, maxVelocity, Cmax, Cmin, Kps):
    Utr = abs(rightError) * Kps
    print("Saturation Right:", SaturationFunction(Utr, Cmax, Cmin))
    if rightError < 0:
        SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utr, Cmax, Cmin))
    else:
        SetAngularVelocity(maxVelocity - SaturationFunction(Utr, Cmax, Cmin), maxVelocity)


def wallFollowing(targetDistances, Kpf, Kps, targetWall):
    maxVelocity = 0.9
    currentDistances = getDistanceReadings()
    Cmax = 0.9  # m/s
    Cmin = 0  # m/s

    leftDistance = round(((currentDistances[0] + currentDistances[1]) / 2), 3)
    rightDistance = round(((currentDistances[3] + currentDistances[4]) / 2), 3)

    frontError = targetDistances[1] - currentDistances[2]
    Utf = abs(frontError) * Kpf

    leftError = targetDistances[0] - leftDistance
    rightError = targetDistances[2] - rightDistance

    print(
        f'Distances from Walls: {[leftDistance, currentDistances[2], rightDistance]}')
    print(f"front Error: {frontError}")
    print(f"left Error: {leftError}")
    print(f"right Error: {rightError}")
    print("Saturation Front:", SaturationFunction(Utf, Cmax, Cmin))

    if frontError < 0:  # far away from front wall move foward
        if targetWall == 'l':
            wallFollowingLeft(leftError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)
        elif targetWall == 'r':
            WallFollowingRight(rightError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)
        else:
            SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))
    else:  # passed the wall Reverse

        if current_maze_file == maze_file[0]:
            SetAngularVelocity(-maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                               -maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
            if (frontError == 0):
                SetAngularVelocity(0, 0)
                return 0
        else:
            if targetWall == 'l':
                wallFollowingLeft(leftError, maxVelocity, Cmax, Cmin, Kps)
            elif targetWall == 'r':
                WallFollowingRight(rightError, maxVelocity, Cmax, Cmin, Kps)
            else:
                SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)
    print("Velocities: ", round(left_velocity,3),round(right_velocity,3))


def SaturationFunction(VelocityControl, Cmax, Cmin):
    if VelocityControl > Cmax:  # far away from object
        return Cmax
    elif Cmin <= VelocityControl <= Cmax:  # Somewhat close to object
        return VelocityControl
    else:  # Really close to object.
        return Cmin


# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = ['worlds/mazes/Labs/Lab2/Lab2_Task1.xml', 'worlds/mazes/Labs/Lab2/Lab2_Task2_1.xml',
             'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml', 'worlds/mazes/Labs/Lab2/Lab2_EC_1.xml',
             'worlds/mazes/Labs/Lab2/Lab2_EC_2.xml']

current_maze_file = maze_file[0]
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

while robot.experiment_supervisor.step(robot.timestep) != -1:
    # Distance Traveled by center of the robot. Average of the sum of all encoder readings*wheel radius
    Distance_Traveled = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
    # print("Distance_Traveled=", Distance_Traveled)

    # print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Robot's Encoder Readings
    # print("Motor Encoder Readings: ", robot.get_encoder_readings())
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    print(f"Orientation, {robot.get_compass_reading()}")

    # print(f"Center of Robot Distances Average={getDistanceReadings()}")
    if wallFollowing([0.4, 0.5, 0.4], 1, 1, 'r') == 0:
        break
