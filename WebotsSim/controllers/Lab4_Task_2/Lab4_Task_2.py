# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab4_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import math
import os

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

os.chdir("../..")

# Functions not Used because of the lack of goal recognition
'''
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
        
def CenterObjectPID(RelativePosition, Kps, Cmax, C_min):
    CenterTarget = 0
    ObjectCenterError = CenterTarget - RelativePosition
    Control_signal = abs(ObjectCenterError) * Kps

    if ObjectCenterError < 0:
        SetAngularVelocity(-SaturationFunction(Control_signal, Cmax, C_min),
                           SaturationFunction(Control_signal, Cmax, C_min))
    else:
        SetAngularVelocity(SaturationFunction(Control_signal, Cmax, C_min),
                           -SaturationFunction(Control_signal, Cmax, C_min))
                           

# checks to see if all goals are found
def is_targets_found(dictionary):
    for key, value in dictionary.items():
        if not value or len(value) != 4:
            return False
    return True
    
'''


def getDistanceReadings():
    # sets readings when robot is in a straight orientation/ parallel to the wall

    Left_reading = robot.get_lidar_range_image()[200]
    front_reading = robot.get_lidar_range_image()[400]
    right_reading = robot.get_lidar_range_image()[600]
    back_reading = robot.get_lidar_range_image()[0]

    if Left_reading <= 1:
        Left_reading = 1
    else:
        Left_reading = 0

    if front_reading <= 1:
        front_reading = 1
    else:
        front_reading = 0

    if right_reading <= 1:
        right_reading = 1
    else:
        right_reading = 0

    if back_reading <= 1:
        back_reading = 1
    else:
        back_reading = 0

    return [Left_reading, front_reading, right_reading, back_reading]


def SetAngularVelocity(left_velocity, right_velocity):
    robot.set_left_motors_velocity(left_velocity / robot.wheel_radius)
    robot.set_right_motors_velocity(right_velocity / robot.wheel_radius)
    # print("Velocities: ", round(left_velocity, 3), round(right_velocity, 3))


def SaturationFunction(VelocityControl, Cmax, C_min):
    if VelocityControl > Cmax:  # far away from object
        return Cmax
    elif C_min <= VelocityControl <= Cmax:  # Somewhat close to object
        return round(VelocityControl, 3)
    else:  # Really close to object.
        return C_min


def printMaze(cells):
    print(' ' * 2 + '_' * 35)
    for i in range(1, 17):
        print('  ', end='|')
        if i in cells:
            print('{:^5}'.format('o'), end=' ')
        else:
            print('{:^5}'.format('X'), end=' ')
        if i % 4 == 0:
            print("|")
            if i != 16:
                print('  |' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 6 + '|')
    print('  ' + '_' * 35)


def outputPrinting():
    print('+' + '-' * 44 + '+')
    print('{:^36}'.format('Current State Position:'))
    print(' ')
    print('{:^36}\n'.format(f'S=({round(RobotCurrentCoordinates[0], 3)}, '
                            f'{round(RobotCurrentCoordinates[1], 3)}, '
                            f'{currentCellWithNeighbors[0]}, {robot.get_compass_reading()})'))
    print(' ')
    print('{:^36}'.format('Visited Cells:'))
    printMaze(GlobalCellCoordinates)
    print(' ')
    print('{:^36}'.format('State Probability:'))
    StateProbability()
    print('+' + '-' * 44 + '+')


def StateProbability():
    CurrentReadings = getDistanceReadings()
    # [z=0,s=0] = 0.7, [z=0,s=1] = 0.1, [z=1,s=0] = 0.3, [z=1,s=1] = 0.9; z=row, s=column
    SensorModel = [[0.7, 0.1], [0.3, 0.9]]
    currentOrientation = robot.get_compass_reading()
    cellProbabilities = []
    # [0] = West, [1] = North, [2] = East, [3] = South
    # [0] = Left, [1] = Front, [2] = Right, [3] = Back
    ReadingAdjusted = CurrentReadings
    # robot facing east
    if 0 <= currentOrientation <= 3 or 357 <= currentOrientation <= 360:
        # West == back sensor , North == left sensor, East == front sensor, South == right sensor
        ReadingAdjusted = [ReadingAdjusted[3], ReadingAdjusted[0], ReadingAdjusted[1], ReadingAdjusted[2]]
        # print("looking east")
    elif 87 <= currentOrientation <= 93:
        # West==left sensor, North==front sensor, East==right sensor, South==back sensor
        # print("looking north")
        ReadingAdjusted = [ReadingAdjusted[0], ReadingAdjusted[1], ReadingAdjusted[2], ReadingAdjusted[3]]
    elif 177 <= currentOrientation <= 183:
        # West==front sensor, North==right sensor, East==back sensor, South==left sensor
        ReadingAdjusted = [ReadingAdjusted[1], ReadingAdjusted[2], ReadingAdjusted[3], ReadingAdjusted[0]]
        # print("looking west")
    elif 267 <= currentOrientation <= 273:
        # West==right sensor, North==back sensor, East==left sensor, South==front sensor
        ReadingAdjusted = [ReadingAdjusted[2], ReadingAdjusted[3], ReadingAdjusted[0], ReadingAdjusted[1]]
        # print("looking south")
    # value = S
    for map, dictionary in enumerate(WorldConfiguration):
        if map == currentMap[0] - 1:
            for key, value in dictionary.items():
                # print('sValue:', value)
                # print('Adjusted Readings:', ReadingAdjusted)
                ReadingProb = 1
                # reading = Z
                for index, reading in enumerate(ReadingAdjusted):
                    ReadingProb *= SensorModel[int(reading)][int(value[index])]
                    # print('cell', key, 'Z-Value:', reading, 'S-Value:', value[index],'sensor index:',reading,value[index],
                    # 'prob:', SensorModel[int(reading)][int(value[index])], 'ReadingProb:', ReadingProb)
                cellProbabilities.append(ReadingProb)

    # print('Before Norm', cellProbabilities)

    Norm = 1 / sum(cellProbabilities)
    # print('Norm', Norm)
    for index, cell in enumerate(cellProbabilities):
        cellProbabilities[index] = cell * Norm

    # print('After Norm', cellProbabilities)
    print(' ' * 2 + '_' * 35)
    for i in range(1, 17):
        print('  ', end='|')
        print('{:^5}'.format(f'{round(cellProbabilities[i - 1], 3)}'), end=' ')
        if i % 4 == 0:
            print("|")
            if i != 16:
                print('  |' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 6 + '|')
    print('  ' + '_' * 35)


'''
def get_localCoordinates(localCoordinates, GlobalCoordinates):
    X_coordinate = []
    Y_coordinate = []
    for key, value in localCoordinates.items():
        if value:
            X_coordinate.append((GlobalCoordinates[key][0] - value[2]))
            Y_coordinate.append((GlobalCoordinates[key][1] - value[3]))

    return round(sum(X_coordinate) / len(X_coordinate), 3), round(sum(Y_coordinate) / len(Y_coordinate), 3)
'''


def findCurrentCell(RobotCoordinates):
    surroundingCells = [-1, -1, -1, -1]
    currentWorld = {}
    for index, world in enumerate(WorldConfiguration):
        if index == currentMap[0] - 1:
            currentWorld = world
    for Map, dictionary in enumerate(WorldConfiguration):
        if Map == currentMap[0] - 1:
            for key, value in GlobalCellCoordinates.items():
                if (value[0][0] <= RobotCoordinates[0] <= value[0][1] and
                        value[1][1] <= RobotCoordinates[1] <= value[1][0]):
                    # Finds surrounding cells and checks if they are in the dict meaning not visited
                    if ((key - 1) in GlobalCellCoordinates and value[1][1] == GlobalCellCoordinates[key - 1][1][1] and
                        value[1][0] == GlobalCellCoordinates[key - 1][1][0]) and currentWorld[key][0] != 1:
                        surroundingCells[0] = key - 1

                    if ((key - 4) in GlobalCellCoordinates and value[0][0] == GlobalCellCoordinates[key - 4][0][0] and
                        value[0][
                            1]
                        == GlobalCellCoordinates[key - 4][0][1]) and currentWorld[key][1] != 1:
                        surroundingCells[1] = key - 4
                    if ((key + 1) in GlobalCellCoordinates and value[1][1] == GlobalCellCoordinates[key + 1][1][1] and
                        value[1][
                            0]
                        == GlobalCellCoordinates[key + 1][1][0]) and currentWorld[key][2] != 1:
                        surroundingCells[2] = key + 1
                    if ((key + 4) in GlobalCellCoordinates and value[0][0] == GlobalCellCoordinates[key + 4][0][0] and
                        value[0][
                            1]
                        == GlobalCellCoordinates[key + 4][0][1]) and currentWorld[key][3] != 1:
                        surroundingCells[3] = key + 4
                    # returns the current cell and the surrounding cells
                    return key, surroundingCells
            # not in any cell
    return -1, surroundingCells


'''
def gotoCornerCell(currentPos, neighbor):
    CornerCells = [1, 4, 13, 16]
    if currentPos in CornerCells:
        # print('Corner Reached')
        return True
    else:
        for cell in neighbor:
            if cell in CornerCells and cell != -1:
                # print('cell',cell)
                move_to_Neighbor(currentPos, cell)
                return False

        for cell in neighbor:
            if (currentPos - 5) in CornerCells and cell != -1 and (currentPos - 1 == cell or currentPos - 4 == cell):
                # print('currentPOS-5',currentPos-5,cell)
                move_to_Neighbor(currentPos, cell)
                return False
            elif (currentPos - 3) in CornerCells and cell != -1 and (currentPos + 1 == cell or currentPos - 4 == cell):
                # print('currentPOS-2',currentPos-3,cell)
                move_to_Neighbor(currentPos, cell)
                return False
            elif (currentPos + 5) in CornerCells and cell != -1 and (currentPos + 1 == cell or currentPos + 4 == cell):
                # print('currentPOS+5',currentPos+5,cell)
                move_to_Neighbor(currentPos, cell)
                return False
            elif (currentPos + 3) in CornerCells and cell != -1 and (currentPos - 1 == cell or currentPos + 4 == cell):
                # print('CurrentPOS+3',currentPos+3,cell)
                move_to_Neighbor(currentPos, cell)
                return False
'''


def MoveByAmountPID(DistanceTraveled, TargetDistance, Kps, Cmax, C_min):
    CenterTarget = TargetDistance
    ObjectCenterError = CenterTarget - DistanceTraveled
    Control_signal = abs(ObjectCenterError) * Kps

    # print("Distance Traveled", DistanceTraveled)
    # print("Distance Error", ObjectCenterError)

    if ObjectCenterError > 0:
        SetAngularVelocity(SaturationFunction(Control_signal, Cmax, C_min),
                           SaturationFunction(Control_signal, Cmax, C_min))
        if -0.01 <= ObjectCenterError <= 0.01:
            robot.stop()
            return round(DistanceTraveled, 3)
    else:
        SetAngularVelocity(-SaturationFunction(Control_signal, Cmax, C_min),
                           -SaturationFunction(Control_signal, Cmax, C_min))

    return 0


def TurnToOrientation(target, Kps, Cmax, C_min):
    OrientationTarget = target
    OrientationError = OrientationTarget - robot.get_compass_reading()

    if target == 0 or target == 360:
        if OrientationError < -180:
            OrientationError += 360
        elif OrientationError > 180:
            OrientationError -= 360

    Control_Signal = abs(OrientationError) * Kps

    if OrientationError < 0:
        SetAngularVelocity(SaturationFunction(Control_Signal, Cmax, C_min),
                           -SaturationFunction(Control_Signal, Cmax, C_min))
    else:
        SetAngularVelocity(-SaturationFunction(Control_Signal, Cmax, C_min),
                           SaturationFunction(Control_Signal, Cmax, C_min))

    if abs(OrientationError) == 0:
        robot.stop()
        return 1


def move_to_Neighbor(currentState, targetCell):
    # Should get the distance that the robot has traveled before reaching the cell.
    Distance_Traveled = ((sum(robot.get_encoder_readings()) * robot.wheel_radius) / 4) - CurrentEncoderReading[0]
    DistanceReturned = 0

    if currentState + 1 == targetCell and targetCell in currentCellWithNeighbors[1]:
        # print(robot.get_compass_reading())
        if 0 <= robot.get_compass_reading() < 1 or 359 < robot.get_compass_reading() <= 360:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(0, 0.01, 0.5, 0)

    elif currentState - 1 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 179 < robot.get_compass_reading() < 181:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(180, 0.01, 0.5, 0)

    elif currentState + 4 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 269 < robot.get_compass_reading() < 271:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(270, 0.01, 0.5, 0)

    elif currentState - 4 == targetCell and targetCell in currentCellWithNeighbors[1]:
        if 89 < robot.get_compass_reading() < 91:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(90, 0.01, 0.5, 0)

    if DistanceReturned != 0:
        RobotCurrentCoordinates[0] += (DistanceReturned * math.cos(math.radians(robot.get_compass_reading())))
        RobotCurrentCoordinates[1] += (DistanceReturned * math.sin(math.radians(robot.get_compass_reading())))
        currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordinates)[0])
        currentCellWithNeighbors[1] = (findCurrentCell(RobotCurrentCoordinates)[1])
        VisitedCellOrder.append(currentCellWithNeighbors[0])


def backtracking(CurrentState, targetCell):
    surroundingCells = [-1, -1, -1, -1]
    Distance_Traveled = ((sum(robot.get_encoder_readings()) * robot.wheel_radius) / 4) - CurrentEncoderReading[0]
    DistanceReturned = 0
    currentWorld = {}
    for index, world in enumerate(WorldConfiguration):
        if index == currentMap[0] - 1:
            currentWorld = world

    if CurrentState + 1 == targetCell:
        # print(robot.get_compass_reading())
        if 0 <= robot.get_compass_reading() < 1 or 359 < robot.get_compass_reading() <= 360:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(0, 0.01, 0.5, 0)

    elif CurrentState - 1 == targetCell:
        if 179 < robot.get_compass_reading() < 181:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(180, 0.01, 0.5, 0)

    elif CurrentState + 4 == targetCell:
        if 269 < robot.get_compass_reading() < 271:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(270, 0.01, 0.5, 0)

    elif CurrentState - 4 == targetCell:
        if 89 < robot.get_compass_reading() < 91:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(90, 0.01, 0.5, 0)
    if CurrentState == targetCell:
        if VisitedCellOrder:
            VisitedCellOrder.pop()
            
    if DistanceReturned != 0:
        RobotCurrentCoordinates[0] += (DistanceReturned * math.cos(math.radians(robot.get_compass_reading())))
        RobotCurrentCoordinates[1] += (DistanceReturned * math.sin(math.radians(robot.get_compass_reading())))
        
        # Finds surrounding cells and checks if they are in the dict meaning not visited
        if (VisitedCellOrder[-1] - 1) in GlobalCellCoordinates and currentWorld[VisitedCellOrder[-1]][0] != 1:
            surroundingCells[0] = VisitedCellOrder[-1] - 1

        if ((VisitedCellOrder[-1] - 4) in GlobalCellCoordinates) and currentWorld[VisitedCellOrder[-1]][1] != 1:
            surroundingCells[1] = VisitedCellOrder[1] - 4
        if ((VisitedCellOrder[-1] + 1) in GlobalCellCoordinates) and currentWorld[VisitedCellOrder[-1]][2] != 1:
            surroundingCells[2] = VisitedCellOrder[-1] + 1
        if ((VisitedCellOrder[-1] + 4) in GlobalCellCoordinates) and currentWorld[VisitedCellOrder[-1]][3] != 1:
            surroundingCells[3] = VisitedCellOrder[-1] + 4

        currentCellWithNeighbors[0] = VisitedCellOrder[-1]
        currentCellWithNeighbors[1] = surroundingCells
        # print('Current Cell:', currentCellWithNeighbors[0], 'Neighbors:', currentCellWithNeighbors[1])
        # print(RobotCurrentCoordinates)
        CurrentEncoderReading[0] = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
        outputPrinting()


def ReachAllCells(currentState, SurroundingCells):
    count = 0
    for index, cell in enumerate(SurroundingCells):
        # print(index, cell, NeighborFollowing[0])
        if cell != -1 and index == NeighborFollowing[0]:
            move_to_Neighbor(currentState, cell)
            return False
        elif cell == -1:
            count += 1

    NeighborFollowing[0] += 1
    if NeighborFollowing[0] > 3:
        NeighborFollowing[0] = 0

    if not GlobalCellCoordinates:
        print('All Cells Visited')
        robot.stop()
        return True

    if count == 4:
        if VisitedCellOrder:
            backtracking(currentState, VisitedCellOrder[-1])
        return False


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
Task2_1 = {1: [1, 1, 0, 0], 2: [0, 1, 0, 1], 3: [0, 1, 0, 1], 4: [0, 1, 1, 0],
           5: [1, 0, 1, 0], 6: [1, 1, 0, 0], 7: [0, 1, 1, 0], 8: [1, 0, 1, 0],
           9: [1, 0, 1, 0], 10: [1, 0, 0, 1], 11: [0, 0, 0, 1], 12: [0, 0, 1, 1],
           13: [1, 0, 0, 1], 14: [0, 1, 0, 1], 15: [0, 1, 0, 1], 16: [0, 1, 1, 1]}

Task2_2 = {1: [1, 1, 0, 1], 2: [0, 1, 0, 1], 3: [0, 1, 0, 0], 4: [0, 1, 1, 0],
           5: [1, 1, 0, 0], 6: [0, 1, 1, 0], 7: [1, 0, 1, 0], 8: [1, 0, 1, 0],
           9: [1, 0, 1, 0], 10: [1, 0, 0, 1], 11: [0, 0, 1, 1], 12: [1, 0, 1, 0],
           13: [1, 0, 0, 1], 14: [0, 1, 0, 1], 15: [0, 1, 0, 1], 16: [0, 0, 1, 1]}

WorldConfiguration = [Task2_1, Task2_2]

currentMap = [2]
current_maze_file = maze_file[currentMap[0]]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

'''
# Coordinates of the targets in respect to the board
TargetLocationsGlobal = {'Yellow': [-2, 2], 'Red': [2, 2], 'Blue': [2, -2], 'Green': [-2, -2]}

# Coordinates of the targets in respect to the robot; .ie. [0]=Distance, [1]=Orientation, [2]=X, [3]=Y
TargetLocationsLocal = {'Yellow': [], 'Red': [], 'Blue': [], 'Green': []}
'''
majorOrientations = [0, 90, 180, 270, 360]
RobotCurrentCoordinates = [0, 0]
currentCellWithNeighbors = []

CurrentEncoderReading = [0]

# [0] = Distance, [1] = Center, [2] = Edge
ObjectRelativePosition = [-1, -1, -1]

CurrentWallReadings = [0, 0, 0]

InitialCoordinate = False

CornerReached = False

InitialOrientationNorth = False

NeighborFollowing = [0]

VisitedCellOrder = []

while robot.experiment_supervisor.step(robot.timestep) != -1:
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    # print(getDistanceReadings())
    # print(robot.get_compass_reading())

    '''
    if not is_targets_found(TargetLocationsLocal):
        findTarget(TargetLocationsLocal)

    else:
    '''
    if not InitialOrientationNorth:
        TurnToOrientation(90, 1, 0.5, 0)
        if 87 <= robot.get_compass_reading() <= 93:
            InitialOrientationNorth = True
        continue
    # Finds the coordinates of the robot in respect to the board
    if not InitialCoordinate:
        RobotCurrentCoordinates = [robot.starting_position.x, robot.starting_position.y]
        InitialCoordinate = True

    # if robot is in a cell, print the maze and removes the cell from the maze
    if len(currentCellWithNeighbors) == 0:
        currentCellWithNeighbors.append(findCurrentCell(RobotCurrentCoordinates)[0])
        currentCellWithNeighbors.append(findCurrentCell(RobotCurrentCoordinates)[1])
        VisitedCellOrder.append(currentCellWithNeighbors[0])

    # deletes visited cell from maze and prints the maze once per cell
    if currentCellWithNeighbors[0] in GlobalCellCoordinates:
        GlobalCellCoordinates.pop(currentCellWithNeighbors[0])
        # # Prints necessary information once per cell
        # print('------------------------------------------')
        # print('Visited Cells:')
        # printMaze(GlobalCellCoordinates)
        # print(
        #     f'State Pose s=({round(RobotCurrentCoordinates[0], 3)},'
        #     f' {round(RobotCurrentCoordinates[1], 3)},{currentCellWithNeighbors[0]}'
        #     f', {robot.get_compass_reading()})')
        #
        CurrentEncoderReading[0] = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
        # print('State Probability:')
        # StateProbability()
        outputPrinting()
    # print(robot.get_compass_reading())
    # CHECK GO TO THE RIGHT

    '''
    if not CornerReached:
        CornerReached = gotoCornerCell(currentCellWithNeighbors[0], currentCellWithNeighbors[1])
    else:
    '''
    if ReachAllCells(currentCellWithNeighbors[0], currentCellWithNeighbors[1]):
        break
