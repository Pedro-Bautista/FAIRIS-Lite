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
    maxVelocity = 1
    currentDistances = getDistanceReadings()
    Cmax = 1  # m/s
    Cmin = 0.2  # m/s
    if current_maze_file == maze_file[0]:
        targetDistances[1] = 0.5

    if targetWall == 'f':
        if len(currentDistances) == 3:
            leftDistance = currentDistances[0]
            rightDistance = currentDistances[2]

            print(f'Distances from Walls: {currentDistances}')
            frontError = targetDistances[1] - currentDistances[1]

            Utf = abs(frontError) * Kpf
            print(f"3 front {frontError}")
            print("UT:", frontError * Kpf)

            if frontError < 0:  # far away from front wall move foward
                # SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))
                wallFollowing('l', targetDistances, Kpf, Kps)
            elif -0.001 <= frontError < 0.001:  # really close to wall Stop
                if current_maze_file == maze_file[0]:
                    if frontError == 0:
                        SetAngularVelocity(0, 0)
                    else:
                        SetAngularVelocity(maxVelocity - (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                                           maxVelocity - (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
            else:  # passed the wall Reverse
                if current_maze_file == maze_file[0]:
                    SetAngularVelocity(-maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                                       -maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
                else:
                    wallFollowing('l', targetDistances, Kpf, Kps)

        elif len(currentDistances) == 5:
            leftDistance = (currentDistances[0] + currentDistances[1]) / 2
            rightDistance = (currentDistances[3] + currentDistances[4]) / 2
            print(
                f'Distances from Walls: {[leftDistance, currentDistances[2], rightDistance]}')

            frontError = targetDistances[1] - currentDistances[2]
            Utf = abs(frontError) * Kpf
            print(f"5 front {frontError}")
            print("UTF:", frontError * Kpf)
            if frontError < 0:
                wallFollowing('l', targetDistances, Kpf, Kps)
            elif -0.001 <= frontError < 0.001:
                if current_maze_file == maze_file[0]:
                    if frontError == 0:
                        SetAngularVelocity(0, 0)
                    else:
                        SetAngularVelocity(maxVelocity - (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                                           maxVelocity - (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
            else:
                if current_maze_file == maze_file[0]:
                    SetAngularVelocity(-maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                                       -maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))
                else:
                    wallFollowing('l', targetDistances, Kpf, Kps)

    if targetWall == 'l':
        if len(currentDistances) == 3:
            leftError = targetDistances[0] - currentDistances[0]
            Utl = abs(leftError) * Kps
            print(f"3left {leftError}")
            print("UTL:", leftError * Kps)
            if currentDistances[0] + currentDistances[2] > 1.1 and currentDistances[1] <= 1.7:
                if (targetDistances[0] >= targetDistances[2]):
                    SetAngularVelocity(maxVelocity - (maxVelocity - SaturationFunction(Utl, Cmax, Cmin)), maxVelocity)
                else:
                    SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))
            else:
                if leftError < 0:  # far away from wall move towards wall.
                    SetAngularVelocity(maxVelocity - SaturationFunction(Utl, Cmax, Cmin), maxVelocity)

                else:  # close to wall, move away from wall.
                    SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))

        elif len(currentDistances) == 5:
            leftDistance = (currentDistances[0] + currentDistances[1]) / 2
            rightDistance = (currentDistances[3] + currentDistances[4]) / 2
            leftError = targetDistances[0] - leftDistance
            Utl = abs(leftError) * Kps
            print(f"5left {leftError}")
            print("Saturation:", SaturationFunction(Utl, Cmax, Cmin))
            if leftDistance + rightDistance > 1.1 and currentDistances[2] <= 1.7:
                if leftDistance >= rightDistance:
                    SetAngularVelocity(maxVelocity - (maxVelocity - SaturationFunction(Utl, Cmax, Cmin)), maxVelocity)
                else:
                    SetAngularVelocity(maxVelocity, maxVelocity - (maxVelocity - SaturationFunction(Utl, Cmax, Cmin)))
            else:
                if leftError < 0:
                    SetAngularVelocity(maxVelocity - SaturationFunction(Utl, Cmax, Cmin), maxVelocity)
                else:
                    SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))

    if targetWall == 'r':
        if len(currentDistances) == 3:
            rightError = targetDistances[2] - currentDistances[2]
            Utl = abs(rightError) * Kps
            print(f"3left {rightError}")
            print("UTL:", rightError * Kps)

            if rightError < 0:  # far away from wall move towards wall.
                SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))

            else:  # close to wall, move away from wall.
                SetAngularVelocity(maxVelocity - SaturationFunction(Utl, Cmax, Cmin), maxVelocity)

        elif len(currentDistances) == 5:
            leftDistance = (currentDistances[0] + currentDistances[1]) / 2
            rightDistance = (currentDistances[3] + currentDistances[4]) / 2
            rightError = targetDistances[2] - rightDistance
            Utl = abs(rightError) * Kps
            print(f"5left {rightError}")
            print("Saturation:", SaturationFunction(Utl, Cmax, Cmin))
            if rightError < 0:
                SetAngularVelocity(maxVelocity, maxVelocity - SaturationFunction(Utl, Cmax, Cmin))
            else:
                SetAngularVelocity(maxVelocity - SaturationFunction(Utl, Cmax, Cmin), maxVelocity)


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)


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
             'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml']
current_maze_file = maze_file[1]
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
    print(f"Orientation, {robot.get_compass_reading()}")

    print(robot.get_compass_reading())

    # print(f"Center of Robot Distances Average={getDistanceReadings()}")
    wallFollowing('f', [0.4, 0.5, 0.4], 1, 0.1)

    # SetAngularVelocity(0.8,-0.8)
