# Pedro Bautista U85594600
# WebotsSim/controllers/Lab1_Task1/Lab1_Task1.py
# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math

from controller import Robot

os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot


# Created a class to include all types of movement required by the robot.
class RobotMovement:
    # initialized the class where the maximum forward speed it to be passed. Also creates class lists for value history
    def __init__(self, maximum_forward_speed):
        self.maximum_forward_speed = maximum_forward_speed
        self.velocities_history = []
        self.distances_history = []
        self.angular_velocities_history = []

    # Move foward function requires distance which is calculated from the specific waypoints.
    def move_forward(self, distance):
        # velocity is calculated by the angular velocity given to the motor * the wheel radius.
        left_velocity = self.maximum_forward_speed * robot.wheel_radius
        # When moving straight, the right velocity is the same as the left velocity.
        right_velocity = left_velocity
        # center velocity is the average of both velocities
        center_velocity = (left_velocity + right_velocity) / 2
        # creates a temporary list that stores the given distance. Same with the velocities
        distances = [distance, distance, distance]
        current_velocities = [round(left_velocity, 3), round(center_velocity, 3), round(right_velocity, 3)]
        # Both distances and velocities are appended to the class list when called. Can grab this value by rembering
        # in what order the functions were called.
        self.velocities_history.append(current_velocities)
        self.distances_history.append(distances)

        # Time is calculated from the center distance divided by the center velocity.
        time = distances[1] / current_velocities[1]

        # Returns temporary lists as well with time for functions below.
        return current_velocities, distances, time

    # Move circular function requires radius of the circle, the portion traveled and the direction of the rotation to
    # properly calculate the velocities
    def move_circular(self, Radius, portion_of_circle_traveled, rotation):

        # Grabs the axel length from the robot instance divides by 2 to get axel_mid
        axel_length = robot.axel_length
        axel_mid = (axel_length / 2)
        # not the maximum angular velocity set by robot but due to error and under steering, velocity must slow down
        max_velocity_set = 4.99

        # center_Distance is the circumference/circle traveled
        center_Distance = (2 * Radius * math.pi) * portion_of_circle_traveled
        left_distance, right_distance = 0, 0

        # sets the left and right velocity to the max velocity possible from the max velocity set.
        # The proper value will update depending on which condition is met.
        left_velocity, right_velocity = max_velocity_set * robot.wheel_radius, max_velocity_set * robot.wheel_radius
        angular_velocity = 0

        # if Direction is clockwise, right velocity is properly calculated as well as the angular velocity and distances
        if rotation.lower() == "clockwise":
            # calculated based on the formula R=abs(axel_mid((left_velocity+right_Velocity)/(
            # left_velocity-right_Velocity)))
            right_velocity = (((Radius * left_velocity) - (axel_mid * left_velocity)) / (Radius + axel_mid))
            angular_velocity = (left_velocity - right_velocity) / axel_length
            left_distance = ((2 * math.pi) * (Radius + axel_mid)) * portion_of_circle_traveled
            right_distance = ((2 * math.pi) * (Radius - axel_mid)) * portion_of_circle_traveled
        # if Direction is counterclockwise,left velocity is properly calculated as well as the angular velocity and
        # distances
        elif rotation.lower() == "counterclockwise":
            # calculated based on the formula R=abs(axel_mid((left_velocity+right_Velocity)/(
            # right_velocity-left_Velocity)))
            left_velocity = (((Radius * right_velocity) - (axel_mid * right_velocity)) / (Radius + axel_mid))
            angular_velocity = (right_velocity - left_velocity) / axel_length
            left_distance = (2 * math.pi * (Radius - axel_mid)) * portion_of_circle_traveled
            right_distance = (2 * math.pi * (Radius + axel_mid)) * portion_of_circle_traveled

        # After the conditions are met, the center velocity is calculated using the radius and angular velocity
        # around ICC
        center_Velocity = Radius * angular_velocity
        # create temp lists that stores velocities, angular velocity for each wheel and the distances traveled and it
        # gets appended to the class list.
        current_velocities = [round(left_velocity, 3), round(center_Velocity, 3), round(right_velocity, 3)]
        current_angular_velocity = [left_velocity / robot.wheel_radius, right_velocity / robot.wheel_radius]
        self.angular_velocities_history.append(current_angular_velocity)
        self.velocities_history.append(current_velocities)
        distances = [round(left_distance, 3), round(center_Distance, 3), round(right_distance, 3)]
        self.distances_history.append(distances)

        time = center_Distance / center_Velocity

        return current_velocities, distances, time

    def get_velocity_history(self):
        return self.velocities_history

    def get_angular_velocity_history(self):
        return self.angular_velocities_history

    def get_distances_history(self):
        return self.distances_history

    def get_forward_velocity(self):
        return self.maximum_forward_speed

    def prints(self, movement, waypoint_X, waypoint_Y, Radius=0.0, circle_traveled=0.0, distance=0.0):
        velocities = []
        distances = []
        time = 0.0

        if movement == 'forward':
            velocities, distances, time = self.move_forward(distance)

        if movement == 'clockwise' or movement == 'counterclockwise':
            velocities, distances, time = self.move_circular(Radius, circle_traveled, movement)

        print(f"{waypoint_X}->{waypoint_Y}\nVelocities [Vl,Vc,Vr]: {velocities}m/s")
        print(f"Distances traveled by [Dl,Dc,Dr]: {distances}m")
        print(f"Time taken:{round(time, 3)} seconds\n")
        print(" \n")
        return velocities, distances


# Create the robot instance.
robot = MyRobot()
RobotMovement1 = RobotMovement(15)

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()
# The order of the movements that the robot will go through in this lab
# P0P1 Forward movement
P0P1 = RobotMovement1.prints('forward', "P0", "P1", distance=2.5)[1][1]

# P1P2 Circular Movement
P1P2 = RobotMovement1.prints("clockwise", "P1", "P2", 0.5, 0.25)[1][1] + P0P1

# P2P3 Forward Movement
P2P3 = RobotMovement1.prints('forward', "P0", "P1", distance=2)[1][1] + P1P2

# P3P4 Circular Movement
P3P4 = RobotMovement1.prints("clockwise", "P3", "P4", 0.5, 0.25)[1][1] + P2P3
# P4P5 Forward Movement
P4P5 = RobotMovement1.prints("forward", "P4", "P5", distance=1)[1][1] + P3P4
# P5P6 Circular Movement
P5P6 = RobotMovement1.prints("clockwise", "P5", "P6", 1.5, 0.25)[1][1] + P4P5
# P6P7 Circular Movement
P6P7 = RobotMovement1.prints("clockwise", "P6", "P7", 0.75, 0.5)[1][1] + P5P6
Total_Distance = P6P7
# print(Total_Distance)

Velocity_Changed = False
# Main Control Loop for Robot will break once the total distance is reached
while robot.experiment_supervisor.step(robot.timestep) != -1:

    # Distance Traveled by center of the robot. Average of the sum of all encoder readings*wheel radius
    Distance_Traveled = sum(robot.get_encoder_readings()) * robot.wheel_radius / 4
    # print("Distance_Traveled=", Distance_Traveled)

    # print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Robot's Encoder Readings
    # print("Motor Encoder Readings: ", robot.get_encoder_readings())
    # print("Simulation Time", robot.experiment_supervisor.getTime())
    # print(f"Orientation, {robot.get_compass_reading()}")

    # Stops the robot after the robot moves a distance of 2.5 meters
    if Distance_Traveled <= 2.5:
        robot.go_forward(RobotMovement1.get_forward_velocity())
        if not Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[0]}m/s")
            Velocity_Changed = True

    elif (Distance_Traveled >= 2.50) and (Distance_Traveled <= P1P2):
        if robot.get_compass_reading() >= 90:
            robot.set_left_motors_velocity(RobotMovement1.angular_velocities_history[0][0])
            robot.set_right_motors_velocity(RobotMovement1.angular_velocities_history[0][-1])
        else:
            robot.go_forward(RobotMovement1.get_forward_velocity())
        if Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[1]}m/s")
            Velocity_Changed = False

    elif (Distance_Traveled >= P1P2) and (Distance_Traveled <= P2P3):
        robot.go_forward(RobotMovement1.get_forward_velocity())
        if not Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[2]}m/s")
            Velocity_Changed = True

    elif (Distance_Traveled >= P2P3) and (Distance_Traveled <= P3P4):
        if 0 <= robot.get_compass_reading() <= 355:
            robot.set_left_motors_velocity(RobotMovement1.angular_velocities_history[1][0])
            robot.set_right_motors_velocity(RobotMovement1.angular_velocities_history[1][-1])
        else:
            robot.go_forward(RobotMovement1.get_forward_velocity())
        if Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[3]}m/s")
            Velocity_Changed = False

    elif (Distance_Traveled >= P3P4) and (Distance_Traveled <= P4P5 + 0.05):
        robot.go_forward(RobotMovement1.get_forward_velocity())
        if not Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[4]}m/s")
            Velocity_Changed = True

    elif (Distance_Traveled >= P4P5) and (Distance_Traveled <= P5P6):
        if 360 >= robot.get_compass_reading() >= 270:
            robot.set_left_motors_velocity(RobotMovement1.angular_velocities_history[2][0])
            robot.set_right_motors_velocity(RobotMovement1.angular_velocities_history[2][-1])
        else:
            robot.set_left_motors_velocity(RobotMovement1.angular_velocities_history[3][0])
            robot.set_right_motors_velocity(RobotMovement1.angular_velocities_history[3][-1])
        if Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[5]}m/s")
            Velocity_Changed = False

    elif (Distance_Traveled >= P5P6) and (Distance_Traveled <= P6P7):
        if robot.get_compass_reading() >= 95:
            robot.set_left_motors_velocity(RobotMovement1.angular_velocities_history[3][0])
            robot.set_right_motors_velocity(RobotMovement1.angular_velocities_history[3][-1])
        else:
            robot.stop()
            print("Robot has reached the end of the simulation!")
            # print(Total_Distance)
            # print(Distance_Traveled)
            break
        if not Velocity_Changed:
            print(f"Velocities have changed. New velocities [Vl,Vc,Vr] are {RobotMovement1.velocities_history[6]}m/s")
            Velocity_Changed = True

    else:
        robot.stop()
        print("Robot has reached the end of the simulation!")
        # print(Total_Distance)
        # print(Distance_Traveled)
        break
