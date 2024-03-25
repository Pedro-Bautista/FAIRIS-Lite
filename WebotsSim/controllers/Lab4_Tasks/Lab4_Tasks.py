# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab4_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math

from controller import Robot

os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

def printMaze(cells):
    for i in range(1, 17):
        if i in cells:
            print('.', end=' ')
        else:
            print('X', end=' ')
        if i % 4 == 0:
            print()


# checks to see if all goals are found
def is_targets_found(dictionary):
    for key, value in dictionary.items():
        if not value or len(value) != 4:
            return False
    return True


def get_color(RGB):
    if RGB[0] == 1 and RGB[1] == 0 and RGB[2] == 0:
        return "Red"
    elif RGB[0] == 0 and RGB[1] == 1 and RGB[2] == 0:
        return "Green"
    elif RGB[0] == 0 and RGB[1] == 0 and RGB[2] == 1:
        return "Blue"
    elif RGB[0] == 1 and RGB[1] == 1 and RGB[2] == 0:
        return "Yellow"


def get_localCoordinates(localCoordinates, GlobalCoordinates):
    X_coordinate = []
    Y_coordinate = []
    for key, value in localCoordinates.items():
        if value:
            X_coordinate.append((GlobalCoordinates[key][0] - value[2]))
            Y_coordinate.append((GlobalCoordinates[key][1] - value[3]))

    return round(sum(X_coordinate) / len(X_coordinate), 3), round(sum(Y_coordinate) / len(Y_coordinate), 3)


def findCurrentCell(RobotCoordinates):
    for key, value in GlobalCellCoordinates.items():
        if value[0][0] <= RobotCoordinates[0] <= value[0][1] and value[1][1] <= RobotCoordinates[1] <= value[1][0]:
            return key

    # not in any cell
    return -1


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


# Will populate the dictionary with the target's distance, orientation, and local X and Y coordinates for every goal
def findTarget(LocalTargets):
    objects = robot.rgb_camera.getRecognitionObjects()
    # Iterate over each object
    objectID = None
    GoalColor = [0, 0, 0]

    for obj in objects:
        # Retrieve properties of the object
        objectID = obj.getId()
        GoalColor = obj.getColors()
        # [0] how far away from camera, [1] how close to center of camera, [2] how close to edge of camera
        ObjectRelativePosition[0] = (obj.getPosition()[0])
        ObjectRelativePosition[1] = (obj.getPosition()[1])
        ObjectRelativePosition[2] = (obj.getPosition()[2])
    Key = get_color(GoalColor)

    # if object is found, utilizes PID to center object in camera
    if objectID is not None and not LocalTargets[Key]:
        CenterObjectPID(ObjectRelativePosition[1], 0.1, 0.5, 0)
        if -0.01 <= ObjectRelativePosition[1] <= 0.01:
            if len(LocalTargets[Key]) != 4:
                # gets distance from the robot to the object
                LocalTargets[Key].append(ObjectRelativePosition[0] - 0.1)
                # gets the orientation of the robot
                LocalTargets[Key].append(robot.get_compass_reading())
                # gets the local X coordinate of the robot
                LocalTargets[Key].append((ObjectRelativePosition[0] - 0.15) * math.cos(
                    math.radians(robot.get_compass_reading())))
                # gets the local Y coordinate of the robot
                LocalTargets[Key].append((ObjectRelativePosition[0] - 0.15) * math.sin(math.radians(
                    robot.get_compass_reading())))

        # for j, i in enumerate(ObjectRelativePosition):
        #     print("OBject Orientation at", j, i)
    else:
        # since there is no object, relative position is 1 and will keep turning.
        CenterObjectPID(1, 0.2, 0.5, 0)


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
    # works at 0.75-1
    # print("TARGET WALL", targetWall)
    maxVelocity = 1
    currentDistances = getDistanceReadings()
    Cmax = 1  # m/s
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
maze_file = ['worlds/mazes/Labs/Lab4/Lab4_Task1_1.xml', 'worlds/mazes/Labs/Lab4/Lab4_Task2_1.xml',
             'worlds/mazes/Labs/Lab4/Lab4_Task2_2.xml']
GlobalCellCoordinates = {1: [[-2, -1], [2, 1]], 2: [[-1, 0], [2, 1]], 3: [[0, 1], [2, 1]], 4: [[1, 2], [2, 1]],
                         5: [[-2, -1], [1, 0]], 6: [[-1, 0], [1, 0]], 7: [[0, 1], [1, 0]], 8: [[1, 2], [1, 0]],
                         9: [[-2, -1], [0, -1]], 10: [[-1, 0], [0, -1]], 11: [[0, 1], [0, -1]],
                         12: [[1, 2], [0, -1]], 13: [[-2, -1], [-1, -2]], 14: [[-1, 0], [-1, -2]],
                         15: [[0, 1], [-1, -2]],
                         16: [[1, 2], [-1, -2]]}
# Dictionary of the maze file
Task2_1 = {1: ['O', 'W', 'W', 'W'], 2: ['O', 'W', 'O', 'W'], 3: ['O', 'W', 'O', 'W'], 4: ['W', 'O', 'O', 'W'],
           5: ['W', 'O', 'W', 'O'], 6: ['W', 'O', 'W', 'O'], 7: ['W', 'W', 'O', 'O'], 8: ['O', 'W', 'O', 'W'],
           9: ['O', 'W', 'O', 'W'], 10: ['O', 'W', 'W', 'O'], 11: ['W', 'O', 'W', 'O'],
           12: ['O', 'O', 'W', 'W'], 13: ['O', 'O', 'O', 'W'], 14: ['W', 'O', 'O', 'W'],
           15: ['W', 'W', 'O', 'O'], 16: ['O', 'W', 'W', 'O']}

Task2_2 = {1: ['W', 'W', 'O', 'W'], 2: ['O', 'W', 'O', 'W'], 3: ['O', 'W', 'O', 'O'], 4: ['O', 'W', 'W', 'O'],
           5: ['W', 'W', 'O', 'O'], 6: ['O', 'W', 'W', 'O'], 7: ['W', 'O', 'W', 'O'], 8: ['W', 'O', 'W', 'O'],
           9: ['W', 'O', 'W', 'O'], 10: ['W', 'O', 'O', 'W'], 11: ['O', 'O', 'W', 'W'],
           12: ['W', 'O', 'W', 'O'], 13: ['W', 'O', 'O', 'W'], 14: ['O', 'W', 'O', 'W'],
           15: ['O', 'W', 'O', 'W'], 16: ['O', 'O', 'W', 'W']}

current_maze_file = maze_file[0]  # Will select the proper map to perform the task.
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

# Cooridinates of the targets in respect to the board
TargetLocationsGlobal = {'Yellow': [-2, 2], 'Red': [2, 2], 'Blue': [2, -2], 'Green': [-2, -2]}
# Cooridinates of the targets in respect to the robot; ie. [0]=Distance, [1]=Orientation, [2]=X, [3]=Y
TargetLocationsLocal = {'Yellow': [], 'Red': [], 'Blue': [], 'Green': []}

RobotCurrentCoordintes = [0, 0]

cellsVisited = []

PrintFlag = False

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

    if not is_targets_found(TargetLocationsLocal):
        findTarget(TargetLocationsLocal)
    else:
        # Finds the coordinates of the robot in respect to the board
        RobotCurrentCoordintes = get_localCoordinates(TargetLocationsLocal, TargetLocationsGlobal)
        # if robot is in a cell, print the maze and removes the cell from the maze
        currentCell = findCurrentCell(RobotCurrentCoordintes)
        if currentCell != -1:
            # deletes visited cell from maze
            GlobalCellCoordinates.pop(findCurrentCell(RobotCurrentCoordintes))

            # Prints necessary information once per cell
            print('Visited Cells:')
            printMaze(GlobalCellCoordinates)
            print(f'State Pose s=({RobotCurrentCoordintes[0]}, {RobotCurrentCoordintes[1]},{currentCell}'
                  f', {robot.get_compass_reading()})')
