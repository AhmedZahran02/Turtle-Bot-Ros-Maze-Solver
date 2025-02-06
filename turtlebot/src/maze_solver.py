#!/usr/bin/env python

import rospy, time
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import cv2
import actionlib
from math import atan2, pi
import math
import time
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped



current_linear_velocity = None
current_angular_velocity = None

explore_lite_process = None
move_base_process = None

initial_x = None
initial_y = None
total_distance = 0.0

visited_locations = []  
tolerance = 0.1  

map_origin = None
map_data = None
resolution = None
maze_image = None

robot_x = None
robot_y = None
current_pose = None

start_time = None
fail_cnt = 0

range_front = []
range_right = []
range_left  = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0
sensor_ranges = []


exit_point = None  
exit_found = False
out_maze = False
loop_detected = False
time_threshold = 20.0


LINEAR_VELOCITY = 0.2  
ANGULAR_VELOCITY = 0.5  


def print_status(event):
    global total_distance, start_time, current_angular_velocity, current_linear_velocity

    elapsed_time = rospy.get_time() - start_time

    # Print the robot's status
    print("----------------------------------------------------------------")
    print("Elapsed Time: ", elapsed_time/60.0, " minutes")
    print("Total Distance: ", total_distance, " meters")
    print("Current Speed: ", current_linear_velocity, " m/s")
    print("Angular Velocity: ", current_angular_velocity.z, " rad/s")
    robot_position = f"Robot Position: (x: {initial_x:.2f}, y: {initial_y:.2f})"

    print(robot_position)

def run_explore_lite():
    global explore_lite_process
    try:
        print("Starting explore_lite package...")
        explore_lite_process = subprocess.Popen(
            ["roslaunch", "explore_lite", "explore.launch"], 
            stdout=subprocess.DEVNULL,  
            stderr=subprocess.DEVNULL  
        )
        print("explore_lite started successfully.")
    except Exception as e:
        rospy.logerr("Failed to start explore_lite: %s", str(e))

def run_move_base():
    global move_base_process
    try:
        print("Starting move_base package...")
        move_base_process = subprocess.Popen(
            ["roslaunch", "turtlebot3_navigation", "move_base.launch"], 
            stdout=subprocess.DEVNULL,  
            stderr=subprocess.DEVNULL   
        )
        print("move_base started successfully.")
    except Exception as e:
        rospy.logerr("Failed to start move_base: %s", str(e))

def stop_explore_lite():
    global explore_lite_process
    try:
        if explore_lite_process:
            print("Shutting down explore_lite...")
            explore_lite_process.terminate() 
            explore_lite_process.wait()  
            print("explore_lite stopped successfully.")
    except Exception as e:
        rospy.logerr("Failed to stop explore_lite: %s", str(e))

def stop_move_base():
    global move_base_process
    try:
        if move_base_process:
            print("Shutting down move_base...")
            move_base_process.terminate()  
            move_base_process.wait()  
            print("move_base stopped successfully.")
    except Exception as e:
        rospy.logerr("Failed to stop move_base: %s", str(e))

def is_in_visited_locations(x, y, visited_locations, tolerance, time_threshold):
    current_time = time.time()
    for loc in visited_locations:
        stored_x, stored_y, stored_time = loc
        if abs(stored_x - x) < tolerance and abs(stored_y - y) < tolerance:
            if current_time - stored_time > time_threshold:
                return True
    return False

def update_visited_locations(x, y, visited_locations, tolerance, time_threshold):
    current_time = time.time()

    if is_in_visited_locations(x, y, visited_locations, tolerance, time_threshold):
        print("Loop detected! Revisiting location: (%f, %f)", x, y)
        loop_detected = True
        visited_locations.clear()  
    else:
        visited_locations.append((x, y, current_time))

def move_towards_exit():
    print("moving towards exit")

    twist = Twist()
    temp_start_time = rospy.get_time()
    while not rospy.is_shutdown() and not out_maze:
        robot_x = current_pose.position.x
        robot_y = current_pose.position.y

        orientation = current_pose.orientation
        _, _, current_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        dx = exit_point[0] - robot_x
        dy = exit_point[1] - robot_y
        target_angle = atan2(dy, dx)
        distance = (dx**2 + dy**2)**0.5
        angle_diff = target_angle - current_yaw
        angle_diff = (angle_diff + pi) % (2 * pi) - pi

        if distance < tolerance and abs(angle_diff) < tolerance:
            print("Goal reached!")
            break

        if rospy.get_time() - temp_start_time > 70:
            print("move towards exit time limit")
            break


        if min_front < 0.5:
            print("Obstacle detected, stopping or re-routing...")
            twist.linear.x = 0.0
            if min_left > min_right:
                twist.angular.z = 2*ANGULAR_VELOCITY  
            else:
                twist.angular.z = -2*ANGULAR_VELOCITY  
            cmd_vel_pub.publish(twist)
            rospy.sleep(1.0)  
            continue 

        twist.linear.x = min(LINEAR_VELOCITY, distance)  
        twist.angular.z = min(ANGULAR_VELOCITY, max(-ANGULAR_VELOCITY, min(ANGULAR_VELOCITY, angle_diff))) 

        cmd_vel_pub.publish(twist)

        rospy.sleep(0.1)  

    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    print("Robot stopped.")

def visualize_path(robot_x, robot_y):
    global maze_image

    if map_data is None:
        print("Map not available yet.")
        return

    if maze_image is None:
        maze_image = np.zeros((500, 500), dtype=np.uint8)  # Adjusted to 500x500 size

    def scale_coordinate(coord, min_input, max_input, min_output, max_output):
        return int((coord - min_input) / (max_input - min_input) * (max_output - min_output) + min_output)

    grid_x = scale_coordinate(robot_x, -10, 10, 0, 500)
    grid_y = scale_coordinate(robot_y, -10, 10, 0, 500)

    # Ensure the coordinates are within bounds
    if 0 <= grid_x < 500 and 0 <= grid_y < 500: 
        maze_image[grid_y, grid_x] = 255

def send_goal_to_move_base(x, y, frame="map"):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("Waiting for move_base action server...")
    client.wait_for_server()

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Facing forward

    print("Sending goal: (%f, %f)", x, y)
    client.send_goal(goal)

    while not rospy.is_shutdown() and not out_maze:
        state = client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            print("Successfully reached the goal!")
            break
        elif state == actionlib.GoalStatus.PREEMPTED:
            print("Goal was preempted.")
            break
        elif state == actionlib.GoalStatus.ABORTED:
            print("Goal was aborted.")
            break

        rate.sleep() 

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Successfully reached the goal!")
    else:
        print("Failed to reach the goal.")
    if out_maze:  
            print("Out Maze, aborting goal...")
            client.cancel_goal()  
            print("Goal aborted.")    
        
def scan_callback(msg):
    global range_front
    global range_right
    global range_left
    global sensor_ranges
    global min_front,i_front, min_right,i_right, min_left ,i_left
    
    
    sensor_ranges = msg.ranges

    range_front[:5] = msg.ranges[5:0:-1]  
    range_front[5:] = msg.ranges[-1:-5:-1]
    range_right = msg.ranges[300:345]
    range_left = msg.ranges[60:15:-1]


    min_front, i_front = min((value, index) for index, value in enumerate(range_front))
    min_right,i_right = min((value, index) for index, value in enumerate(range_right))
    min_left ,i_left  = min((value, index) for index, value in enumerate(range_left))
    
def map_callback(msg):
    global map_data, resolution, map_origin
    # print((msg.info.height, msg.info.width))
    # print(msg.info.resolution)

    map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    # detect_exit(map_data)
    resolution = msg.info.resolution
    map_origin = msg.info.origin.position

def odom_callback(msg):
    global initial_x, initial_y, total_distance
    global visited_locations
    global current_pose
    global exit_found, exit_point, out_maze
    global robot_x, robot_y, current_angular_velocity, current_linear_velocity
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    current_pose = msg.pose.pose

    if(robot_x > 10.2 or robot_x < -10.2 or robot_y > 10.2 or robot_y < -10.2):
        out_maze = True
    
    robot_orientation = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    update_visited_locations(robot_x, robot_y, visited_locations, tolerance, time_threshold)

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    if initial_x is None and initial_y is None:
        initial_x = current_x
        initial_y = current_y

    current_linear_velocity = msg.twist.twist.linear
    current_angular_velocity = msg.twist.twist.angular

    current_linear_velocity = (current_linear_velocity.x ** 2 + current_linear_velocity.y ** 2 + current_linear_velocity.z ** 2) ** 0.5

    distance_moved = math.sqrt((current_x - initial_x)**2 + (current_y - initial_y)**2)
    total_distance += distance_moved
    initial_x = current_x
    initial_y = current_y




    maze_min_x, maze_max_x = -13.0, 13.0
    maze_min_y, maze_max_y = -13.0, 13.0
    exit_points = []  

    for angle in range(len(sensor_ranges)):  
        distance = min(sensor_ranges[angle], 9.8)
        angle_rad = math.radians(angle) + robot_orientation
        point_x = robot_x + distance * math.cos(angle_rad)
        point_y = robot_y + distance * math.sin(angle_rad)
        if point_x > maze_max_x or point_x < maze_min_x or point_y > maze_max_y or point_y < maze_min_y:
            exit_points.append((point_x, point_y))
            exit_found = True

   

    if exit_points:
        reference_x = (maze_max_x + maze_min_x) / 2
        reference_y = (maze_max_y + maze_min_y) / 2
        exit_point = min(exit_points, key=lambda p: (p[0] - reference_x)**2 + (p[1] - reference_y)**2)
    

    visualize_path(robot_x, robot_y)

def abort_and_send_new_goal(new_x, new_y):
    print("Aborting current goal and sending new goal.")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("Waiting for move_base action server...")
    client.wait_for_server()
    client.cancel_goal()
    send_goal_to_move_base(new_x, new_y)


cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # to move the robot
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # to read the laser scanner
rospy.Subscriber("/map", OccupancyGrid, map_callback)
rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.init_node('maze_solver', log_level=rospy.WARN)
rospy.Timer(rospy.Duration(10), print_status)  # 10 seconds interval
command = Twist()
rate = rospy.Rate(5)
time.sleep(5)
       

def left_wall_following():
    loop_detected = False
    visited_locations.clear()
    print("left wall following")
    near_wall = 0 # start with 0, when we get to a wall, change to 1
    temp_start_time = rospy.get_time()
    while not rospy.is_shutdown() and not out_maze and not loop_detected:
        # The algorithm:
        # 1. Robot moves forward to be close to a wall
        # 2. Start following left wall.
        # 3. If too close to the left wall, reverse a bit to get away
        # 4. Otherwise, follow wall by zig-zagging along the wall
        # 5. If front is close to a wall, turn until clear
        # 6. Check for exit condition.

        # Check if the robot has found the exit
    
        if rospy.get_time() - temp_start_time > 300:
            print("left wall time limit")
            break

        while near_wall == 0 and not rospy.is_shutdown() and not out_maze:  # Step 1
            if min_front > 0.2 and min_right > 0.2 and min_left > 0.2:
                command.angular.z = -0.1  # If nothing near, go forward
                command.linear.x = 0.3
            elif min_left < 0.2:  # If wall on left, start tracking
                near_wall = 1
            else:
                command.angular.z = -0.25  # If not on left, turn right
                command.linear.x = 0.0

            cmd_vel_pub.publish(command)

        else:  # Left wall detected
            if min_front > 0.3:  # Step 2
                if min_left < 0.2:  # Step 3
                    command.angular.z = -1.2
                    command.linear.x = -0.1
                elif min_left > 0.3:  # Step 4
                    command.angular.z = 0.6
                    command.linear.x = 0.2
                else:
                    command.angular.z = -0.6
                    command.linear.x = 0.2

            else:  # Step 5
                command.angular.z = -1.0
                command.linear.x = 0.0
                cmd_vel_pub.publish(command)
                while min_front < 0.3 and not rospy.is_shutdown():
                    pass

            cmd_vel_pub.publish(command)


        rospy.sleep(0.1)  # Wait before sending the next command


try:
    start_time = rospy.get_time()
    explore_start_time = rospy.get_time()
    while not rospy.is_shutdown() and not out_maze:
        run_explore_lite()
        run_move_base()
        while not exit_found and not out_maze:
            if rospy.get_time() - explore_start_time > 1500:
                print("1500 seconds elapsed, stopping exploration and navigation.")
                break
        stop_move_base()
        stop_explore_lite()
        if exit_found and not out_maze:
            move_towards_exit()
        if exit_found and not out_maze:
            run_move_base()
            send_goal_to_move_base(exit_point[0], exit_point[1])
            stop_move_base()
        fail_cnt = fail_cnt + 1
        if fail_cnt > 3:
            left_wall_following()  
            explore_start_time = rospy.get_time()
            fail_cnt = 0  

        exit_found = False

    maze_image_rgb = cv2.cvtColor(maze_image, cv2.COLOR_GRAY2RGB)


    command.angular.z = 0.0
    command.linear.x = 0.0    
    cmd_vel_pub.publish(command)

    elapsed_time = (rospy.get_time() - start_time)/60.0
    print("Time taken in minutes = ", elapsed_time)
    print("Distance taken in meters = ", total_distance)
    maze_image = np.transpose(maze_image)
    cv2.imshow("Path", maze_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
except rospy.ROSInterruptException:
    print("ROS Interrupt Exception occurred. Shutting down...")
    command.angular.z = 0.0
    command.linear.x = 0.0
    cmd_vel_pub.publish(command)
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    command.angular.z = 0.0
    command.linear.x = 0.0
    cmd_vel_pub.publish(command)

