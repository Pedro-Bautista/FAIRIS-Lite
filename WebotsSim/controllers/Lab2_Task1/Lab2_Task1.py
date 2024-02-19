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


# This function is used to follow the left wall. Will adjust the distance from the left wall as well as the speed
# depending on the front wall
def wallFollowingLeft(leftError, maxVelocity, Cmax, Cmin, Kps):
    Utl = abs(leftError) * Kps
    # print(round(Utl,3))
    print("Saturation Left:", SaturationFunction(Utl, Cmax, Cmin))

    if leftError < 0:
        if maxVelocity != SaturationFunction(Utl, Cmax, Cmin):
            SetAngularVelocity(abs(maxVelocity - SaturationFunction(Utl, Cmax, Cmin)), maxVelocity)
        else:
            SetAngularVelocity(SaturationFunction(Utl, Cmax, Cmin), SaturationFunction(Utl, Cmax, Cmin))
    else:
        SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))


# This function is used to follow the right wall. Will adjust the distance from the right wall as well as the speed
# depending on the front wall
def wallFollowingRight(rightError, maxVelocity, Cmax, Cmin, Kps):
    Utr = abs(rightError) * Kps
    # print("Saturation Right:", SaturationFunction(Utr, Cmax, Cmin))
    if rightError < 0:
        SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utr, Cmax, Cmin))
    else:
        SetAngularVelocity(maxVelocity - SaturationFunction(Utr, Cmax, Cmin), maxVelocity)


# This function is used for PID for distance corresponding to the front wall and will tell the robot which wall to
# follow. Also tells the robot when and how to curve when there is an opening.
def wallFollowing(targetDistances, Kpf, Kps, targetWall):
    maxVelocity = 0.9
    currentDistances = getDistanceReadings()
    Cmax = 0.9  # m/s
    Cmin = 0  # m/s

    # Gets the average distance from left and right sensor. Reading are 50 Degrees apart from center robot.
    leftDistance = round(((currentDistances[0] + currentDistances[1]) / 2), 3)
    rightDistance = round(((currentDistances[3] + currentDistances[4]) / 2), 3)

    # Calculated the front error of the robot and calculates the velocity
    frontError = targetDistances[1] - currentDistances[2]
    Utf = abs(frontError) * Kpf

    # Calculates left and right error of the robot based on the walls.
    leftError = targetDistances[0] - leftDistance
    rightError = targetDistances[2] - rightDistance
    currentOrientation = robot.get_compass_reading()

    # Used for orientation calculation. Which tells the robot how far to rotate and when to stop
    tolerance = 5

    print(f'Distances from Walls: {[leftDistance, currentDistances[2], rightDistance]}')
    print(f"front Error: {frontError:.3}")
    print(f"left Error: {leftError:.3}")
    print(f"right Error: {rightError:.3}")
    print("FLAG Change Before:", flag_Change[0])
    print("Saturation Front:", SaturationFunction(Utf, Cmax, Cmin))
    originalWall = targetWall

    if rightDistance + leftDistance >= 1.1:  # Whenever there is an open space either left
        # or right side
        targetWall = 'c'  # change to c for curving
        # if frontError <= -0.5:  # Will happen when robot is 0.4 meters from the target distance of 0.5. So 0.9m away

        for target in target_orientations:
            if target == initial_orientation[0]:
                continue
            orientationDifference = abs(currentOrientation - target)
            # print("Target,Orientation:", currentOrientation, target)

            if orientationDifference > 360:
                orientationDifference = 360 - orientationDifference
            print("INITIAL orientation:", initial_orientation[0])
            print("ORIETNATION DIFFERENCE,", currentOrientation, "-", target)
            if orientationDifference <= tolerance:
                Target_reaches[0] = 1
                if flag_Change[0] == 1:
                    flag_Change[0] = 0
                    initial_orientation[0] = target
        if frontError >= -0.4:
            flag_Change[0] = 1
            print("LOLLLL Error")
            Target_reaches[0] = 0
    print("TARGET WALL", targetWall)
    print("ORIENTATION REACHED: ,", Target_reaches[0])

    if frontError < 0:  # far away from front wall move foward
        if targetWall == 'l':
            flag_Change[0] = 1
            wallFollowingLeft(leftError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)
        elif targetWall == 'r':
            flag_Change[0] = 1
            wallFollowingRight(rightError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)

        elif targetWall == 'c' and Target_reaches[0] == 0:
            if flag_Change[0] == 1:
                if originalWall == 'l':
                    if leftError <= rightError or leftDistance>=0.8:
                        print("HEY")
                        SetAngularVelocity(0.4 * Cmax, Cmax)

                    else:
                        print("LOLO")
                        SetAngularVelocity(Cmax, 0.58 * Cmax)

                elif originalWall == 'r':
                    if rightDistance >= 1:
                        SetAngularVelocity(Cmax, 0.58 * Cmax)
                    else:
                        SetAngularVelocity(Cmax, 0.58 * Cmax)
            else:
                print("BYW")
                SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))

        else:
            SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))

    else:  # passed the wall Reverse
        if current_maze_file == maze_file[0]:
            SetAngularVelocity(-maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                               -maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
            if frontError == 0:
                SetAngularVelocity(0, 0)
                return 0
        else:
            if targetWall == 'l':
                wallFollowingLeft(leftError, maxVelocity, Cmax, Cmin, Kps)
            elif targetWall == 'r':
                wallFollowingRight(rightError, maxVelocity, Cmax, Cmin, Kps)
            elif targetWall == 'c':
                if originalWall == 'l':
                    SetAngularVelocity(Cmax, 0.2 * Cmax)
                elif originalWall == 'r':
                    SetAngularVelocity(0.2 * Cmax, Cmax)

            else:
                SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)
    print("Velocities: ", round(left_velocity, 3), round(right_velocity, 3))


def SaturationFunction(VelocityControl, Cmax, Cmin):
    if VelocityControl > Cmax:  # far away from object
        return Cmax
    elif Cmin <= VelocityControl <= Cmax:  # Somewhat close to object
        return round(VelocityControl, 3)
    else:  # Really close to object.
        return Cmin


# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = ['worlds/mazes/Labs/Lab2/Lab2_Task1.xml', 'worlds/mazes/Labs/Lab2/Lab2_Task2_1.xml',
             'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml', 'worlds/mazes/Labs/Lab2/Lab2_EC_1.xml',
             'worlds/mazes/Labs/Lab2/Lab2_EC_2.xml']

current_maze_file = maze_file[1]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
target_orientations = [180, 90, 0, 270]  # Used to tell the robot when to stop turning.
Target_reaches = [0]
initial_orientation = [robot.get_compass_reading()]  # Grabs the initial reading of the orientation of the robot
flag_Change = [1]  # Used to limit the time it takes to change the target orientation.

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
    if wallFollowing([0.4, 0.5, 0.4], 1, 1, 'l') == 0:
        break
