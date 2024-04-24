# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab4_Task2.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import math
import os
import matplotlib.pyplot as plt
import pickle as rick

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

os.chdir("../..")
GRID_SIZE = 4


def getDistanceReadings():
    # sets readings when robot is in a straight orientation/ parallel to the wall
    currentOrientation = robot.get_compass_reading()
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
    ReadingAdjusted = [Left_reading, front_reading, right_reading, back_reading]

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
    return ReadingAdjusted


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


def printWallConfiguration():
    for cell, walls in WorldConfiguration.items():
        row = math.floor((cell - 1) / GRID_SIZE)
        col = (cell - 1) % GRID_SIZE
        if walls[1] == 1:
            plt.plot([col, col + 1], [GRID_SIZE - row, GRID_SIZE - row], 'k')  # Top wall
        if walls[2] == 1:
            plt.plot([col + 1, col + 1], [GRID_SIZE - row, GRID_SIZE - 1 - row], 'k')  # Right wall
        if walls[3] == 1:
            plt.plot([col, col + 1], [GRID_SIZE - 1 - row, GRID_SIZE - 1 - row], 'k')  # Bottom wall
        if walls[0] == 1:
            plt.plot([col, col], [GRID_SIZE - row, GRID_SIZE - 1 - row], 'k')  # Left wall

    plt.xlim(0, GRID_SIZE)
    plt.ylim(0, GRID_SIZE)
    plt.gca()
    plt.gca().set_aspect('equal')
    plt.axis('off')
    plt.show()


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
    print('+' + '-' * 44 + '+')


def findCurrentCell(RobotCoordinates):
    # Initial neighbor cells depends on wall configuration
    # tst row=floor(cell#-1 /gridsize) col=cell#-1 % gridsize

    surroundingCells = [-1, -1, -1, -1]
    # finds column and row of the robot in respect to the board then finds the cell
    CurrentColumn = math.floor((GRID_SIZE / 2) + (RobotCoordinates[0]))
    CurrentRow = math.floor((GRID_SIZE / 2) - (RobotCoordinates[1]))
    CurrentCell = ((CurrentRow * GRID_SIZE) + CurrentColumn) + 1
    if CurrentCell not in GlobalCellCoordinates:
        return -1, surroundingCells

    # assigns wall configuration to the current cell
    WorldConfiguration[CurrentCell] = getDistanceReadings()
    # finds the surrounding cells of the current cell
    if WorldConfiguration[CurrentCell][0] != 1 and ((CurrentCell - 1) in GlobalCellCoordinates and
                                                    GlobalCellCoordinates[CurrentCell - 1][0] == CurrentRow):
        surroundingCells[0] = CurrentCell - 1

    if WorldConfiguration[CurrentCell][1] != 1 and ((CurrentCell - 4) in GlobalCellCoordinates and
                                                    GlobalCellCoordinates[CurrentCell - 4][1] == CurrentColumn):
        surroundingCells[1] = CurrentCell - 4

    if WorldConfiguration[CurrentCell][2] != 1 and ((CurrentCell + 1) in GlobalCellCoordinates and
                                                    GlobalCellCoordinates[CurrentCell + 1][0] == CurrentRow):
        surroundingCells[2] = CurrentCell + 1

    if WorldConfiguration[CurrentCell][3] != 1 and ((CurrentCell + 4) in GlobalCellCoordinates and
                                                    GlobalCellCoordinates[CurrentCell + 4][1] == CurrentColumn):
        surroundingCells[3] = CurrentCell + 4
    # returns the current cell and the surrounding cells
    # print('Current Cell:', CurrentCell, 'Neighbors:', surroundingCells)
    return CurrentCell, surroundingCells


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
    currentWorld = WorldConfiguration

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
maze_file = ['worlds/mazes/Labs/Lab5/Lab5_SmallMaze1.xml', 'worlds/mazes/Labs/Lab5/Lab5_SmallMaze2.xml',
             'worlds/mazes/Labs/lab5/Lab5_LargeMaze.xmll', 'worlds/mazes/MicroMouse/Maze2.xml',
             'worlds/mazes/MicroMouse/Maze3.xml', 'worlds/mazes/MicroMouse/Maze4.xml']
GlobalCellCoordinates = {}

# Dictionary of the maze file
WorldConfiguration = {}
for i in range(1, (GRID_SIZE * GRID_SIZE) + 1):
    row = math.floor((i - 1) / GRID_SIZE)
    col = (i - 1) % GRID_SIZE
    GlobalCellCoordinates[i] = [row, col]
current_maze_file = maze_file[2]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
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
        CurrentEncoderReading[0] = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
        outputPrinting()

    if ReachAllCells(currentCellWithNeighbors[0],
                     currentCellWithNeighbors[1]) or robot.experiment_supervisor.getTime() >= 180:
        printWallConfiguration()
        break

with open(f'../MapConfigurations/{current_maze_file}', 'wb') as file:
    rick.dump(WorldConfiguration, file)
