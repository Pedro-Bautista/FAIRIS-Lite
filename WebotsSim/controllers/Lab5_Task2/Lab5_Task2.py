# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab4_Task2.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import math
import os
import random

import matplotlib.pyplot as plt
import pickle as rick  # pickle rick

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

os.chdir("../..")

# *IF MAZE IS CHANGED, CHANGE THE GRID SIZE TO MATCH THE MAZE*
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
    for i in range(1, (GRID_SIZE ** 2) + 1):
        print('  ', end='|')
        if i in cells:
            print('{:^5}'.format('o'), end=' ')
        else:
            print('{:^5}'.format('X'), end=' ')
        if i % GRID_SIZE == 0:
            print("|")
            if i != 16:
                print('  |' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 8 + '|' + '_' * 6 + '|')
    print('  ' + '_' * 35)


def printWallConfiguration():
    color = 'w'
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

        if cell in ShortestPath:
            if cell == ShortestPath[0]:
                color = 'r'
            elif cell == ShortestPath[-1]:
                color = 'g'
            else:
                color = 'y'
            plt.fill_between([col, col + 1], GRID_SIZE - row, GRID_SIZE - 1 - row, color=color, alpha=0.5)

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
    # finds column and row of the robot in respect to the board then finds the cell
    CurrentColumn = math.floor((GRID_SIZE / 2) + (RobotCoordinates[0]))
    CurrentRow = math.floor((GRID_SIZE / 2) - (RobotCoordinates[1]))
    CurrentCell = ((CurrentRow * GRID_SIZE) + CurrentColumn) + 1
    if CurrentCell not in GlobalCellCoordinates:
        return -1
    # returns the current cell and the surrounding cells
    return CurrentCell


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

    if currentState + 1 == targetCell and targetCell in AllNeighbors[currentState]:
        # print(robot.get_compass_reading())
        if 0 <= robot.get_compass_reading() < 1 or 359 < robot.get_compass_reading() <= 360:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(0, 0.01, 0.5, 0)

    elif currentState - 1 == targetCell and targetCell in AllNeighbors[currentState]:
        if 179 < robot.get_compass_reading() < 181:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(180, 0.01, 0.5, 0)

    elif currentState + 4 == targetCell and targetCell in AllNeighbors[currentState]:
        if 269 < robot.get_compass_reading() < 271:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(270, 0.01, 0.5, 0)

    elif currentState - 4 == targetCell and targetCell in AllNeighbors[currentState]:
        if 89 < robot.get_compass_reading() < 91:
            DistanceReturned = MoveByAmountPID(Distance_Traveled, 1, 1, 1, 0)
        else:
            TurnToOrientation(90, 0.01, 0.5, 0)

    if DistanceReturned != 0:
        RobotCurrentCoordinates[0] += (DistanceReturned * math.cos(math.radians(robot.get_compass_reading())))
        RobotCurrentCoordinates[1] += (DistanceReturned * math.sin(math.radians(robot.get_compass_reading())))
        currentCellWithNeighbors[0] = (findCurrentCell(RobotCurrentCoordinates))
        VisitedCellOrder.append(currentCellWithNeighbors[0])
        ShortestPath.pop()


def FindShortestPath(CurrentState, Map, Goal):
    length = {j: float('inf') for j in range(1, len(Map))}
    length[CurrentState] = 0
    predecessor = {}
    visited = set()
    unvisited = sorted(list(AllNeighbors.keys()))

    while unvisited:
        current = min(unvisited, key=lambda x: length[x])
        unvisited.remove(current)
        visited.add(current)
        if current == Goal:
            shortestPath = []
            while current in predecessor:
                shortestPath.insert(0, current)
                current = predecessor[current]
            shortestPath.insert(0, CurrentState)
            return shortestPath[::-1]
        for direction, neighbor in enumerate(AllNeighbors[current]):
            if neighbor != -1:
                # Calculate the distance to the neighbor (assuming each step has a unit cost)
                distance_to_neighbor = length[current] + 1
                # Update the distance if it's shorter than the current distance
                if distance_to_neighbor < length[neighbor]:
                    length[neighbor] = distance_to_neighbor
                    predecessor[neighbor] = current
    print(length)
    return []


def FollowShortestPath(currentState, ShortestPlannedPath):
    if len(ShortestPlannedPath) != 0:
        if ShortestPlannedPath[-1] == currentState:
            ShortestPlannedPath.pop()
        move_to_Neighbor(currentState, ShortestPlannedPath[-1])
        return False

    robot.stop()
    print("GOAL")
    return True


# *IF YOU WANT TO CHANGE THE MAZE, CHANGE THE INDEX OF THE MAZE_FILE ARRAY. TO LOAD, CHANGE INDEX IN CURRENT_MAZE_FILE*
# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = ['worlds/mazes/Labs/Lab5/Lab5_SmallMaze1.xml', 'worlds/mazes/Labs/Lab5/Lab5_SmallMaze2.xml',
             'worlds/mazes/Labs/lab5/Lab5_LargeMaze.xml', 'worlds/mazes/Labs/Lab5/Lab5_SmallMazeTest.xml']

current_maze_file = maze_file[3]  # Will select the proper map to perform the task.
robot.load_environment(current_maze_file)

with open(f'MapConfigurations/{(current_maze_file.split("/")[-1]).split(".")[0]}', 'rb') as file:
    # Loaded dictionary with the neighbors of each cell
    AllNeighbors = rick.load(file)
    # Loaded dictionary with the maze configuration
    WorldConfiguration = rick.load(file)

# Grid Cell Numbering
GlobalCellCoordinates = {}
for i in range(1, (GRID_SIZE * GRID_SIZE) + 1):
    row = math.floor((i - 1) / GRID_SIZE)
    col = (i - 1) % GRID_SIZE
    GlobalCellCoordinates[i] = [row, col]

robot.move_to_start()

majorOrientations = [0, 90, 180, 270, 360]
RobotCurrentCoordinates = [0, 0]
currentCellWithNeighbors = []

CurrentEncoderReading = [0]
CurrentWallReadings = [0, 0, 0]

InitialCoordinate = False
InitialOrientationNorth = False

NeighborFollowing = [0]
VisitedCellOrder = []

ShortestPath = []
GoalCell = random.randint(1, GRID_SIZE ** 2)

while robot.experiment_supervisor.step(robot.timestep) != -1:

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
        currentCellWithNeighbors.append(findCurrentCell(RobotCurrentCoordinates))
        VisitedCellOrder.append(currentCellWithNeighbors[0])

        ShortestPath = FindShortestPath(currentCellWithNeighbors[0], AllNeighbors, GoalCell)
        printWallConfiguration()
    # deletes visited cell from maze and prints the maze once per cell
    if currentCellWithNeighbors[0] in GlobalCellCoordinates:
        GlobalCellCoordinates.pop(currentCellWithNeighbors[0])
        CurrentEncoderReading[0] = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
        outputPrinting()

    if FollowShortestPath(currentCellWithNeighbors[0], ShortestPath) or robot.experiment_supervisor.getTime() >= 180:
        break
