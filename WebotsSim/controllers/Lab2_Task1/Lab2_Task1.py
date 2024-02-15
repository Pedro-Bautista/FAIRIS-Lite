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
    if (robot.get_compass_reading() == 0 or robot.get_compass_reading() == 90 or robot.get_compass_reading() == 180 or
            robot.get_compass_reading() == 270 or robot.get_compass_reading() == 360):

        # Left distances are from lidar where is the average from a wide range, where points are from the top and
        # bottom left corners and what is between.
        Average_Left_Distance = (sum(robot.get_lidar_range_image()[100:301]) / 200) - 0.06
        # similar to the left distance. The average will give you the distance horizontally to the robot.
        Average_Right_Distance = (sum(robot.get_lidar_range_image()[500:701]) / 200) - 0.06
        # Front distance the reading from the lidar
        Average_Front_Distance = (robot.get_lidar_range_image()[400] - 0.093) + 0.093
        # array for distances, [left,center,right]
        Center_Robot_Distances = [round(Average_Left_Distance, 3), round(Average_Front_Distance, 3),
                                  round(Average_Right_Distance, 3)]
    else:

        # If robot is not parallel to wall, readings come from lidar, where left and right have two separate readings
        Front_Left = robot.get_lidar_range_image()[225]
        Rear_Left = robot.get_lidar_range_image()[175]
        Front_Right = robot.get_lidar_range_image()[575]
        Rear_Right = robot.get_lidar_range_image()[625]
        Average_Front_Distance = (robot.get_lidar_range_image()[400] - 0.093) + 0.093
        Center_Robot_Distances = [round(Front_Left, 3), round(Rear_Left, 3), round(Average_Front_Distance, 3),
                                  round(Front_Right, 3),
                                  round(Rear_Right, 3)]
    return Center_Robot_Distances


def wallFollowing(targetWall, targetDistances, Kpf, Kps):
    maxVelocity = 0.8
    currentDistances = getDistanceReadings()
    Cmax = 0.8  # m/s
    Cmin = -0.8  # m/s
    if current_maze_file == maze_file[0]:
        targetDistances[1] = 0.5

    if targetWall == 'f':
        if len(currentDistances) == 3:
            print(f'Distances from Walls: {currentDistances}')
            leftDistance = currentDistances[0]
            rightDistance = currentDistances[2]
            frontError = targetDistances[1] - currentDistances[1]
            Utf = frontError * Kpf
            print(f"3 front {frontError}")

            if frontError < 0:
                wallFollowing('l', targetDistances, Kpf, Kps)
            elif -0.001 <= frontError <= 0.001:
                if current_maze_file == maze_file[0]:
                    if frontError == 0:
                        SetAngularVelocity(0, 0)
                    SetAngularVelocity(maxVelocity - (maxVelocity - abs(frontError) * Kpf),
                                       maxVelocity - (maxVelocity - abs(frontError) * Kpf))
                else:
                    if leftDistance + rightDistance <= 1.1:
                        SetAngularVelocity(maxVelocity, -(maxVelocity - abs(frontError) * Kpf))
            else:
                if leftDistance + rightDistance <= 1.1:
                    SetAngularVelocity(maxVelocity, -(maxVelocity - abs(frontError) * Kpf))
                SetAngularVelocity(-maxVelocity + (maxVelocity - abs(frontError) * Kpf),
                                   -maxVelocity + (maxVelocity - abs(frontError) * Kpf))
        elif len(currentDistances) == 5:
            leftDistance = (currentDistances[0] + currentDistances[1]) / 2
            RightDistance = (currentDistances[3] + currentDistances[4]) / 2
            print(
                f'Distances from Walls: {[leftDistance, currentDistances[2], RightDistance]}')

            frontError = targetDistances[1] - currentDistances[2]
            Utf = frontError * Kpf
            print(f"5 front {frontError}")
            print((maxVelocity - abs(frontError) * Kps))
            if frontError < 0:
                wallFollowing('l', targetDistances, Kpf, Kps)
            elif -0.001 <= frontError < 0.001:
                if current_maze_file == maze_file[0]:
                    if frontError == 0:
                        SetAngularVelocity(0, 0)
                    else:
                        SetAngularVelocity(maxVelocity - (maxVelocity - abs(frontError) * Kpf),
                                       maxVelocity - (maxVelocity - abs(frontError) * Kpf))
                else:
                    if leftDistance + RightDistance <= 1.1:
                        SetAngularVelocity(maxVelocity, -(maxVelocity - abs(frontError) * Kpf))
            else:
                if leftDistance + RightDistance <= 1.1:
                    SetAngularVelocity(maxVelocity, -(maxVelocity - abs(frontError) * Kpf))
                else:
                    if(RightDistance>leftDistance):
                        SetAngularVelocity(maxVelocity, -(maxVelocity - abs(frontError) * Kpf))
                    else:
                        SetAngularVelocity(-(maxVelocity - abs(frontError) * Kpf),maxVelocity)
                # else:
                #     SetAngularVelocity(-maxVelocity + (maxVelocity - abs(frontError) * Kpf),
                #                    -maxVelocity + (maxVelocity - abs(frontError) * Kpf))

    if targetWall == 'l':
        if len(currentDistances) == 3:
            leftError = targetDistances[0] - currentDistances[0]
            Utl = leftError * Kps
            print(f"3left {leftError}")
            if leftError < 0:
                SetAngularVelocity(maxVelocity - abs(leftError) * Kps, maxVelocity)
            else:
                if -0.001 <= leftError <= 0.001:
                    SetAngularVelocity(maxVelocity, maxVelocity)
                else:
                    SetAngularVelocity(maxVelocity, maxVelocity - abs(leftError) * Kps)
        elif len(currentDistances) == 5:
            leftDistance = (currentDistances[0] + currentDistances[1]) / 2
            rightDistance = (currentDistances[3] + currentDistances[4]) / 2
            leftError = targetDistances[0] - leftDistance
            print(f"5left {leftError}")
            if leftError < 0:
                if rightDistance > 1:
                    SetAngularVelocity(maxVelocity, maxVelocity - abs(leftError) * Kps)
                else:
                    SetAngularVelocity(maxVelocity - abs(leftError) * Kps, maxVelocity)
            else:
                SetAngularVelocity(maxVelocity, maxVelocity - abs(leftError) * Kps)

    if targetWall == 'r':
        if len(currentDistances) == 3:
            rightError = targetDistances[2] - currentDistances[2]

            if rightError < 0:
                SetAngularVelocity(maxVelocity, maxVelocity - abs(rightError) * Kps)
            else:
                SetAngularVelocity(maxVelocity - abs(rightError) * Kps, maxVelocity)

        elif len(currentDistances) == 5:
            rightError = targetDistances[2] - currentDistances[2]
            if rightError < 0:
                SetAngularVelocity(maxVelocity, maxVelocity - abs(rightError) * Kps)
            else:
                SetAngularVelocity(maxVelocity - abs(rightError) * Kps, maxVelocity)


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)


# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = ['worlds/mazes/Labs/Lab2/Lab2_Task1.xml', 'worlds/mazes/Labs/Lab2/Lab2_Task2_1.xml',
             'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml']
current_maze_file = maze_file[2]
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

Velocity_Changed = False
# Main Control Loop for Robot will break once the total distance is reached
# robot distance sensors for front or back. Lidar for sides.
while robot.experiment_supervisor.step(robot.timestep) != -1:
    # Distance Traveled by center of the robot. Average of the sum of all encoder readings*wheel radius
    Distance_Traveled = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
    # print("Distance_Traveled=", Distance_Traveled)

    # print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Robot's Encoder Readings
    # print("Motor Encoder Readings: ", robot.get_encoder_readings())
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    # print(f"Orientation, {robot.get_compass_reading()}")

    print(robot.get_compass_reading())

    # print(f"Center of Robot Distances Average={getDistanceReadings()}")
    wallFollowing('f', [0.35, 1, 0.35], 0.1, 0.1)
