# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab4_Task1.py
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


def get_color(RGB):
    if RGB[0] == 1 and RGB[1] == 0 and RGB[2] == 0:
        return "Red"
    elif RGB[0] == 0 and RGB[1] == 1 and RGB[2] == 0:
        return "Green"
    elif RGB[0] == 0 and RGB[1] == 0 and RGB[2] == 1:
        return "Blue"
    elif RGB[0] == 1 and RGB[1] == 1 and RGB[2] == 0:
        return "Yellow"


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
    else:
        # since there is no object, relative position is 1 and will keep turning.
        CenterObjectPID(1, 0.2, 0.5, 0)


def CenterObjectPID(RelativePosition, Kps, Cmax, Cmin):
    CenterTarget = 0
    ObjectCenterError = CenterTarget - RelativePosition
    UTOC = abs(ObjectCenterError) * Kps

    if ObjectCenterError < 0:
        SetAngularVelocity(-SaturationFunction(UTOC, Cmax, Cmin), SaturationFunction(UTOC, Cmax, Cmin))
    else:
        SetAngularVelocity(SaturationFunction(UTOC, Cmax, Cmin), -SaturationFunction(UTOC, Cmax, Cmin))


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


def get_localCoordinates(localCoordinates, GlobalCoordinates):
    X_coordinate = []
    Y_coordinate = []
    for key, value in localCoordinates.items():
        if value:
            X_coordinate.append((GlobalCoordinates[key][0] - value[2]))
            Y_coordinate.append((GlobalCoordinates[key][1] - value[3]))

    return round(sum(X_coordinate) / len(X_coordinate), 3), round(sum(Y_coordinate) / len(Y_coordinate), 3)


def findCurrentCell(RobotCoordinates):
    suroundingCells = [-1, -1, -1, -1]
    for key, value in GlobalCellCoordinates.items():
        if value[0][0] <= RobotCoordinates[0] <= value[0][1] and value[1][1] <= RobotCoordinates[1] <= value[1][0]:
            # Finds surrounding cells and checks if they are in the dict meaning not visited
            if ((key - 1) in GlobalCellCoordinates and value[1][1] == GlobalCellCoordinates[key - 1][1][1] and value[1][
                0]
                    == GlobalCellCoordinates[key - 1][1][0]):
                suroundingCells[0] = key - 1
            if ((key - 4) in GlobalCellCoordinates and value[0][0] == GlobalCellCoordinates[key - 4][0][0] and value[0][
                1]
                    == GlobalCellCoordinates[key - 4][0][1]):
                suroundingCells[1] = key - 4
            if ((key + 1) in GlobalCellCoordinates and value[1][1] == GlobalCellCoordinates[key + 1][1][1] and value[1][
                0]
                    == GlobalCellCoordinates[key + 1][1][0]):
                suroundingCells[2] = key + 1
            if ((key + 4) in GlobalCellCoordinates and value[0][0] == GlobalCellCoordinates[key + 4][0][0] and value[0][
                1]
                    == GlobalCellCoordinates[key + 4][0][1]):
                suroundingCells[3] = key + 4
            # returns the current cell and the surrounding cells
            return key, suroundingCells
    # not in any cell
    return -1, suroundingCells


def gotoCornerCell(currentPos, neighbor):
    CornerCells = [1, 4, 13, 16]
    TargetCorner=0
    if currentPos in CornerCells:
        print('In Corner')
        return
    else:
        for cell in neighbor:
            if cell in CornerCells and cell != -1:
                print('cell',cell)
                move_to_Neighbor(currentPos, cell)
                return

        for cell in neighbor:
            if (currentPos-5) in CornerCells and cell != -1 and (currentPos-1 == cell or currentPos-4 == cell):
                print('currentPOS-5',currentPos-5,cell)
                move_to_Neighbor(currentPos, cell)
                return
            elif (currentPos-3) in CornerCells and cell != -1 and (currentPos+1 == cell or currentPos-4 == cell):
                print('currentPOS-2',currentPos-3,cell)
                move_to_Neighbor(currentPos, cell)
                return
            elif (currentPos+5) in CornerCells and cell != -1 and (currentPos+1 == cell or currentPos+4 == cell):
                print('currentPOS+5',currentPos+5,cell)
                move_to_Neighbor(currentPos, cell)
                return
            elif (currentPos+3) in CornerCells and cell != -1 and (currentPos-1 == cell or currentPos+4 == cell):
                print('CurrentPOS+3',currentPos+3,cell)
                move_to_Neighbor(currentPos, cell)
                return


def MoveByAmountPID(DistanceTraveled, TargetDistance, Kps, Cmax, Cmin):
    CenterTarget = TargetDistance
    ObjectCenterError = CenterTarget - DistanceTraveled
    UTOC = abs(ObjectCenterError) * Kps

    # print("Distance Traveled", DistanceTraveled)
    # print("Distance Error", ObjectCenterError)

    if ObjectCenterError > 0:
        SetAngularVelocity(SaturationFunction(UTOC, Cmax, Cmin), SaturationFunction(UTOC, Cmax, Cmin))
        if -0.01 <= ObjectCenterError <= 0.01:
            robot.stop()
            return round(DistanceTraveled, 3)
    else:
        SetAngularVelocity(-SaturationFunction(UTOC, Cmax, Cmin), -SaturationFunction(UTOC, Cmax, Cmin))

    return 0


def TurnToOrientation(target, Kps, Cmax, Cmin):
    OrientationTarget = target
    OrientationError = OrientationTarget - robot.get_compass_reading()

    if target == 0 or target == 360:
        if OrientationError < -180:
            OrientationError += 360
        elif OrientationError > 180:
            OrientationError -= 360

    UTOC = abs(OrientationError) * Kps

    if OrientationError < 0:
        SetAngularVelocity(SaturationFunction(UTOC, Cmax, Cmin), -SaturationFunction(UTOC, Cmax, Cmin))
    else:
        SetAngularVelocity(-SaturationFunction(UTOC, Cmax, Cmin), SaturationFunction(UTOC, Cmax, Cmin))

    if abs(OrientationError) <= 3:
        robot.stop()


def move_to_Neighbor(currentState, targetCell):
    # Should get the distance that the robot has traveled before reaching the cell.
    Distance_Traveled = (sum(robot.get_encoder_readings()) * robot.wheel_radius / 4) - CurrentEncoderReading[0]

    if currentState + 1 == targetCell and targetCell in currentCellWithNeighbors[1]:
        print(robot.get_compass_reading())
        if 0 <= robot.get_compass_reading() <= 3 or 357 <= robot.get_compass_reading() <= 360:
            X_Moved = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
            if X_Moved != 0:
                RobotCurrentCoordintes[0] += X_Moved
                currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordintes)[0])
                currentCellWithNeighbors[1] = (findCurrentCell(RobotCurrentCoordintes)[1])

        else:
            TurnToOrientation(0, 1, 0.5, 0)

    elif currentState - 1 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 177 <= robot.get_compass_reading() <= 183:
            X_Moved = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
            #print(X_Moved)
            #print('Original X', RobotCurrentCoordintes[0])
            if X_Moved != 0:
                RobotCurrentCoordintes[0] -= X_Moved
                #print('New X', RobotCurrentCoordintes[0])
                currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordintes)[0])
                currentCellWithNeighbors[1] = (findCurrentCell(RobotCurrentCoordintes)[1])
        else:
            TurnToOrientation(180, 1, 0.5, 0)

    elif currentState + 4 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 267 <= robot.get_compass_reading() <= 273:
            Y_Moved = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
            if Y_Moved != 0:
                RobotCurrentCoordintes[1] -= Y_Moved
                currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordintes)[0])
                currentCellWithNeighbors[1] = (findCurrentCell(RobotCurrentCoordintes)[1])
        else:
            TurnToOrientation(270, 1, 0.5, 0)

    elif currentState - 4 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 87 <= robot.get_compass_reading() <= 93:
            Y_Moved = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
            if Y_Moved != 0:
                RobotCurrentCoordintes[1] += Y_Moved
                currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordintes)[0])
                currentCellWithNeighbors[1] = (findCurrentCell(RobotCurrentCoordintes)[1])
        else:
            TurnToOrientation(90, 1, 0.5, 0)


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

current_maze_file = maze_file[0]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

# Cooridinates of the targets in respect to the board
TargetLocationsGlobal = {'Yellow': [-2, 2], 'Red': [2, 2], 'Blue': [2, -2], 'Green': [-2, -2]}
# Cooridinates of the targets in respect to the robot; ie. [0]=Distance, [1]=Orientation, [2]=X, [3]=Y
TargetLocationsLocal = {'Yellow': [], 'Red': [], 'Blue': [], 'Green': []}

RobotCurrentCoordintes = [0, 0]
currentCellWithNeighbors = []

CurrentEncoderReading = [0]

# [0] = Distance, [1] = Center, [2] = Edge
ObjectRelativePosition = [-1, -1, -1]

InitialCordinates = False

while robot.experiment_supervisor.step(robot.timestep) != -1:
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    # print(getDistanceReadings())
    # print(robot.get_compass_reading())

    if not is_targets_found(TargetLocationsLocal):
        findTarget(TargetLocationsLocal)

    else:
        # Finds the coordinates of the robot in respect to the board
        if not InitialCordinates:
            RobotCurrentCoordintes = list(get_localCoordinates(TargetLocationsLocal, TargetLocationsGlobal))
            InitialCordinates = True

        # if robot is in a cell, print the maze and removes the cell from the maze
        if len(currentCellWithNeighbors) == 0:
            currentCellWithNeighbors.append(findCurrentCell(RobotCurrentCoordintes)[0])
            currentCellWithNeighbors.append(findCurrentCell(RobotCurrentCoordintes)[1])

        # deletes visited cell from maze and prints the maze once per cell
        if currentCellWithNeighbors[0] in GlobalCellCoordinates:
            GlobalCellCoordinates.pop(currentCellWithNeighbors[0])
            # Prints necessary information once per cell
            print('Visited Cells:')
            printMaze(GlobalCellCoordinates)
            print(
                f'State Pose s=({RobotCurrentCoordintes[0]}, {RobotCurrentCoordintes[1]},{currentCellWithNeighbors[0]}'
                f', {robot.get_compass_reading()})')

            print('Neighbors:', currentCellWithNeighbors[1])
            CurrentEncoderReading[0] = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
            print('------------------------------------------')
        #print(robot.get_compass_reading())
        #CHECK GO TO THE RIGHT
        gotoCornerCell(currentCellWithNeighbors[0], currentCellWithNeighbors[1])
        #move_to_Neighbor(currentCellWithNeighbors[0], currentCellWithNeighbors[1][0])
