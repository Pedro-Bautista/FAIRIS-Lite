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


def findTarget(targetWall):
    objects = robot.rgb_camera.getRecognitionObjects()
    # Iterate over each object
    objectID = None

    for obj in objects:
        # Retrieve properties of the object
        objectID = obj.getId()
        # [0] how far away from camera, [1] how close to center of camera, [2] how close to edge of camera
        ObjectRelativePosition[0] = obj.getPosition()[0]
        ObjectRelativePosition[1] = obj.getPosition()[1]
        ObjectRelativePosition[2] = obj.getPosition()[2]

    # Object is not initially found
    if targetFound[0] == 0:
        # if object is found, utilizes PID to center object in camera
        if objectID is not None:
            CenterObjectPID(ObjectRelativePosition[1], 0.1, 0.5, 0)
            if -0.01 <= ObjectRelativePosition[1] <= 0.01:
                targetFound[0] = 1
                Target_reaches[0] = 0
            # for j, i in enumerate(ObjectRelativePosition):
            #     print("OBject Orientation at", j, i)
        else:
            # since there is no object, relative position is 1 and will keep turning.
            CenterObjectPID(1, 0.2, 0.5, 0)
    else:
        if targetFound[0] == 1:
            MotionToGoal()
            if objectID is None:
                targetFound[0] = 2
        if targetFound[0] == 2:
            NinetyDegreeTurns(0.8, targetWall)
        if targetFound[0] == 3:
            if objectID is not None:
                targetFound[0] = 0
            else:
                wallFollowing([0.4, 0.45, 0.4], 1, 1, targetWall)


def CenterObjectPID(ObjectRelativePosition, Kps, Cmax, Cmin):
    CenterTarget = 0
    ObjectCenterError = CenterTarget - ObjectRelativePosition
    UTOC = abs(ObjectCenterError) * Kps

    if ObjectCenterError < 0:
        SetAngularVelocity(-SaturationFunction(UTOC, Cmax, Cmin), SaturationFunction(UTOC, Cmax, Cmin))
    else:
        SetAngularVelocity(SaturationFunction(UTOC, Cmax, Cmin), -SaturationFunction(UTOC, Cmax, Cmin))


def MotionToGoal():
    TurnDirection[0] = 0
    NumberOfTurns[0] = 0
    return wallFollowing([0.5, 0.5, 0.5], 1, 1, 'f')


# This function is used to follow the left wall. Will adjust the distance from the left wall as well as the speed
# depending on the front wall
def wallFollowingLeft(leftError, maxVelocity, Cmax, Cmin, Kps):
    Utl = abs(leftError) * Kps

    if leftError < 0:
        if maxVelocity != SaturationFunction(Utl, Cmax, Cmin):
            SetAngularVelocity(abs(maxVelocity - SaturationFunction(Utl, Cmax, Cmin)), maxVelocity)
        else:
            SetAngularVelocity(SaturationFunction(Utl, Cmax, Cmin), SaturationFunction(Utl, Cmax, Cmin))
    else:
        SetAngularVelocity(maxVelocity, abs(maxVelocity - SaturationFunction(Utl, Cmax, Cmin)))


# This function is used to follow the right wall. Will adjust the distance from the right wall as well as the speed
# depending on the front wall
def wallFollowingRight(rightError, maxVelocity, Cmax, Cmin, Kps):
    Utr = abs(rightError) * Kps
    if rightError < 0:
        if maxVelocity != SaturationFunction(Utr, Cmax, Cmin):
            SetAngularVelocity(maxVelocity, abs(maxVelocity - SaturationFunction(Utr, Cmax, Cmin)))
        else:
            SetAngularVelocity(SaturationFunction(Utr, Cmax, Cmin), SaturationFunction(Utr, Cmax, Cmin))
    else:
        SetAngularVelocity(abs(maxVelocity - SaturationFunction(Utr, Cmax, Cmin)), maxVelocity)


def NinetyDegreeTurns(Cmax, targetWall):
    currentOrientation = robot.get_compass_reading()
    tolerance = 5
    currentDistances = getDistanceReadings()
    leftDistance = round(((currentDistances[0] + currentDistances[1]) / 2), 3)
    rightDistance = round(((currentDistances[3] + currentDistances[4]) / 2), 3)
    # if target is already reached return.
    # print("TARGET", Target_reaches[0])
    if Target_reaches[0] == 1:
        targetFound[0] = 3
        return

    for target in target_orientations:
        if target == initial_orientation[0] or (target - (initial_orientation[0] - 360)) == 0:
            continue
        orientationDifference = abs(currentOrientation - target)

        if orientationDifference > 360:
            orientationDifference = 360 - orientationDifference

        # print("DIFFERENCE", currentOrientation, target, orientationDifference)
        if orientationDifference <= tolerance:
            Target_reaches[0] = 1
            initial_orientation[0] = target
            previousWall[0] = leftDistance
            previousWall[1] = rightDistance
            NumberOfTurns[0] += 1

    if targetWall == 'r':
        if currentDistances[2] <= 0.8 or TurnDirection[0] == 1:
            SetAngularVelocity(Cmax, 0.3 * Cmax)
            TurnDirection[0] = 1
        else:
            if TurnDirection[0] == 2:
                SetAngularVelocity(0.1 * Cmax, -Cmax)
            else:
                SetAngularVelocity(0.4 * Cmax, Cmax)
                TurnDirection[0] = -1

    elif targetWall == 'l':
        if currentDistances[2] <= 0.8 or TurnDirection[0] == -1:
            SetAngularVelocity(0.4 * Cmax, Cmax)
            TurnDirection[0] = -1
        else:
            if TurnDirection[0] == -2:
                SetAngularVelocity(-Cmax, 0.1 * Cmax)
            else:
                SetAngularVelocity(Cmax, 0.4 * Cmax)
                TurnDirection[0] = 1


def UTurns(targetWall, Cmax):
    currentOrientation = robot.get_compass_reading()
    tolerance = 5
    for target in target_orientations:

        if target == initial_orientation[0]:
            continue

        if abs(target - initial_orientation[0]) == 90 or abs(target - (initial_orientation[0] + 360)) == 90:
            continue
        orientationDifference = abs(currentOrientation - target)

        if orientationDifference > 360:
            orientationDifference = 360 - orientationDifference

        if orientationDifference <= 2 * tolerance:
            SetAngularVelocity(0, 0)

    if targetWall == 'l':
        SetAngularVelocity(0.58 * Cmax, -0.45 * Cmax)

    elif targetWall == 'r':
        SetAngularVelocity(-0.2 * Cmax, 0.58 * Cmax)


# This function is used for PID for distance corresponding to the front wall and will tell the robot which wall to
# follow. Also tells the robot when and how to curve when there is an opening.
def wallFollowing(targetDistances, Kpf, Kps, targetWall):
    # print("TARGET WALL", targetWall)
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

    # Used for orientation calculation. Which tells the robot how far to rotate and when to stop
    tolerance = 5

    # print(f'Distances from Walls: {[leftDistance, currentDistances[2], rightDistance]}')
    # print(f"front Error: {frontError:.3}")

    if current_maze_file != maze_file[0] and targetWall != 'f':
        NinetyDegreeTurns(Cmax, targetWall)
        if Target_reaches[0] != 1:
            return
        else:
            if leftDistance - previousWall[0] >= 3 or rightDistance - previousWall[1] >= 2:
                # print("TIME", robot.experiment_supervisor.getTime() - previousTime[0])
                if robot.experiment_supervisor.getTime() - previousTime[0] <= 0.2:
                    SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))
                else:
                    if NumberOfTurns[0] == 2:
                        Target_reaches[0] = 0
                        targetFound[0] = 0
                    else:
                        Target_reaches[0] = 0
                        targetFound[0] = 2
            else:
                if previousWall[1] >= 2:
                    previousWall[1] -= 2
                if rightDistance >= 10 and leftDistance >= 10 and currentDistances[2] >= 10:
                    if (ObjectRelativePosition[2] * (180 / math.pi)) <= robot.get_compass_reading() - 180:
                        initial_orientation[0] += 90
                    targetFound[0] = 0
                previousTime[0] = robot.experiment_supervisor.getTime()
                # print(previousWall)
                # print(previousTime[0])

    if frontError < 0:  # far away from front wall move foward
        # Used to turn around 180
        if frontError >= -0.1 and leftDistance + rightDistance < 1.1:
            if targetWall == 'l':
                SetAngularVelocity(0, 0)
                UTurns('l', Cmax)
            elif targetWall == 'r':
                SetAngularVelocity(0, 0)
                UTurns('r', Cmax)

        # wall Follow Left
        elif targetWall == 'r':
            TurnDirection[0] = 0
            wallFollowingLeft(leftError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)

        # Wall Follow Right
        elif targetWall == 'l':
            TurnDirection[0] = 0
            wallFollowingRight(rightError, SaturationFunction(Utf, Cmax, Cmin), Cmax, Cmin, Kps)
            if rightDistance >= 10 and NumberOfTurns[0] == 2:
                targetFound[0] = 0

        else:
            # print(ObjectRelativePosition)
            if frontError >= -0.30 and ObjectRelativePosition[0] >= 1.5:
                targetFound[0] = 2
            else:
                if robot.get_front_left_distance_reading() <= 0.5 and currentDistances[2] >= 3:
                    targetFound[0] = 2
                    TurnDirection[0] = -2
                elif robot.get_front_right_distance_reading() <= 0.5 and currentDistances[2] >= 3:
                    targetFound[0] = 2
                    TurnDirection[0] = 2
                else:
                    previousWall[2] = currentDistances[2]
                    SetAngularVelocity(SaturationFunction(Utf, Cmax, Cmin), SaturationFunction(Utf, Cmax, Cmin))
    else:  # passed the wall Reverse
        if frontError == 0 and ObjectRelativePosition[0] <= 1.2:
            SetAngularVelocity(0, 0)
            return 0
        else:
            if targetWall == 'l':
                SetAngularVelocity(0, 0)
                UTurns('l', Cmax)
            elif targetWall == 'r':
                SetAngularVelocity(0, 0)
                UTurns('r', Cmax)
            else:
                SetAngularVelocity(-maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)),
                                   -maxVelocity + (maxVelocity - SaturationFunction(Utf, Cmax, Cmin)))


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)
    # print("Velocities: ", round(left_velocity, 3), round(right_velocity, 3))


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
maze_file = ['worlds/mazes/Labs/Lab3/Lab3_Task1.xml', 'worlds/mazes/Labs/Lab3/Lab3_Task2_1.xml',
             'worlds/mazes/Labs/Lab3/Lab3_Task2_2.xml']

current_maze_file = maze_file[2]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
# Used for turning
target_orientations = [180, 90, 0, 270]  # Used to tell the robot when to stop turning.
initial_orientation = [robot.get_compass_reading()]  # Grabs the initial reading of the orientation of the robot
Target_reaches = [0]
TurnDirection = [0]
NumberOfTurns = [0]
previousWall = [None, None, None]
previousTime = [0]

# Used for target finding.
# targetFound[0]=Object not found, spin until found.
# targetFound=1: Target found, and it centered in camera
# targetFound=2: Target was previously found, but was lost due to blockage. Turn and follow wall
# targetFound=3: After Turning Wall Follow
targetFound = [0]
ObjectRelativePosition = [None, None, None]

while robot.experiment_supervisor.step(robot.timestep) != -1:
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    # print(getDistanceReadings())
    # print(robot.get_compass_reading())
    findTarget('r')

    if targetFound[0] == 1:
        if MotionToGoal() == 0:
            print("Goal Reached!")
            print('Distance from Object',getDistanceReadings()[2])
            break
