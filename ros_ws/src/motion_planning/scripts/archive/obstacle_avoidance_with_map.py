#! /usr/bin/env python3
import math
from turtle import update
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import LaserScan

# ==== INITIALIZATION VARIABLES ====
# Publisher (global because it is used in a subscriber callback)
pub = None

# Goal point (updated by an external node)
goal = None

# Current robot pose
current_robot_pose = None

# Current robot speed
current_robot_twist = None

# Dictionary mapping obstacle directions to angular and linear velocities
regions_to_cmd = {"left": (0, -1), "right": (0, 1), "front": (1, 0), "back": (0, 1)}

# Map info
map_info = None
offsets = None
data = None

# ==== FUNCTIONS ====
def update_odom(msg):
    """Update global robot odometry"""
    global current_robot_pose
    global current_robot_twist
    current_robot_pose = msg.pose.pose
    current_robot_twist = msg.twist.twist


def update_goal(msg):
    """Update goal coordinates"""
    global goal
    goal = msg


def coordinates_to_cell(x, y):
    """Convert XY coordinates to cell in Occupancy grid"""
    global map_info
    global data
    x = round(x / map_info.resolution)
    y = round(y / map_info.resolution)

    return data[offsets[1] + y][offsets[0] + x]


def is_occupied(cell):
    """Check if a cell is occupied by an obstacle"""
    if cell == 0:
        return False
    return True


def generate_beams():
    """Emulate a laser scan by generating beams in a cone"""
    ranges = []
    range_size = 60
    min_angle = -1 * math.pi / 2
    max_angle = math.pi / 2
    resolution = (max_angle - min_angle) / range_size
    max_distance = 8

    # Current robot pose
    current_x = round(current_robot_pose.position.x, 1)
    current_y = round(current_robot_pose.position.y, 1)
    orientation = current_robot_pose.orientation
    (_, _, yaw) = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )

    # For each angle... launch a beam (just like a laser)
    for i in range(range_size):
        angle = min_angle + (resolution * i) + yaw
        ranges.append(0)

        # Launch beam
        step = map_info.resolution / 2
        distance = step
        while distance <= max_distance:
            # Get the coordinate of the point ahead
            x = math.cos(angle) * distance + current_x
            y = math.sin(angle) * distance + current_y
            cell = coordinates_to_cell(x, y)

            # Record the current distance without an obstacle
            ranges[-1] = distance

            # If the beam hit an obstacle, break the loop
            if is_occupied(cell):
                break

            distance = distance + step

        # DEBUG PURPOSES
        # if i == range_size // 2:
        #    distance = ranges[range_size // 2]
        #    x = math.cos(angle) * distance + current_x
        #    y = math.sin(angle) * distance + current_y
        #    rospy.loginfo(f"x : {x} y : {y} range : {distance}")

    return ranges


def odom_callback(msg):
    """Get map data, create the beams and separate the ranges into zones, and navigate to goal while avoiding obstacles"""

    # Update odometry variable
    update_odom(msg)

    # If robot position is known, and there is a goal set, try to avigate to it
    if map_info != None and current_robot_pose != None and goal != None:
        ranges = generate_beams()
        navigate(ranges)


def callback_map(msg):
    global offsets
    global map_info
    global data

    # Set map info
    if map_info == None or map_info != msg.info:
        map_info = msg.info

        # Set map offsets
        if offsets == None:
            offset_x = msg.info.width // 2
            offset_y = msg.info.height // 2
            offsets = (offset_x, offset_y)

    # Create Numpy array to store occupancy grid
    data = np.array(msg.data)
    data = data.reshape(msg.info.height, msg.info.width)


def accelerate(speed, dest_speed, acceleration):
    """Control acceleration = Smoother speed control"""
    # Case acceleration
    if dest_speed > speed:
        if dest_speed - speed <= acceleration:
            return dest_speed
        return speed + acceleration

    # Case deceleration
    if speed - dest_speed <= acceleration:
        return dest_speed

    return speed - acceleration


def separate_ranges(ranges):
    """Separate beams readings into regions"""
    d = len(ranges) // 3

    regions = {
        "right": ranges[:d],
        "front": ranges[d : d * 2],
        "left": ranges[d * 2 :],
    }

    return regions


def is_region_occupied(ranges, num_slices, threshold_dist):
    slice_size = len(ranges) // num_slices
    min_values = [
        min(ranges[(slice_size * i) : (slice_size * (i + 1))])
        for i in range(num_slices - 1)
    ]
    return all([val < threshold_dist for val in min_values])


def navigate(ranges):
    # Initialize important constants
    threshold_dist = 2
    lookahead_dist = 1
    max_linear_speed = 1
    max_angular_speed = 0.6
    acceleration = 0.1
    goal_dist = 0.5

    # Instantiate twist message
    msg = Twist()
    linear_x = 0
    angular_z = 0

    # If the robot hasn't reached it's destination yet
    if (
        math.sqrt(
            (goal.x - current_robot_pose.position.x) ** 2
            + (goal.y - current_robot_pose.position.y) ** 2
        )
        > goal_dist
    ):
        # Variable for best region to go to
        best_region = "back"
        best_distance = math.inf

        # Regions that are occupied (for printing only)
        collision_regions = []

        # Separate laser scan into regions
        regions = separate_ranges(ranges)

        # Calculate a region's angle range (in radians)
        region_size = math.pi / len(regions)

        i = 0
        # For each region with its readings...
        for region, region_ranges in regions.items():
            num_slices = 2

            # If region is occupied
            # is_region_occupied(region_ranges, num_slices, threshold_dist)
            if min(region_ranges) < threshold_dist:
                collision_regions.append(region)
            else:
                # Get coordinates of a point located in the middle of the region
                point_theta_before = (region_size * i) + region_size / 2
                orientation = current_robot_pose.orientation
                (_, _, yaw) = euler_from_quaternion(
                    [orientation.x, orientation.y, orientation.z, orientation.w]
                )

                point_theta = point_theta_before + yaw - (math.pi / 2)

                point_x = (
                    lookahead_dist * math.cos(point_theta)
                    + current_robot_pose.position.x
                )
                point_y = (
                    lookahead_dist * math.sin(point_theta)
                    + current_robot_pose.position.y
                )

                distance_to_goal = math.sqrt(
                    (goal.x - point_x) ** 2 + (goal.y - point_y) ** 2
                )

                # rospy.loginfo(f"R :{region} x:{point_x} y:{point_y}")

                if distance_to_goal < best_distance:
                    best_distance = distance_to_goal
                    best_region = region
            i += 1

        # Set linear and angular speed
        speed_tuple = regions_to_cmd[best_region]
        linear_x = accelerate(
            current_robot_twist.linear.x,
            max_linear_speed * speed_tuple[0],
            acceleration,
        )
        angular_z = accelerate(
            -1 * current_robot_twist.angular.z,
            max_angular_speed * speed_tuple[1],
            acceleration,
        )

        # Fill twist message and publish it to control topic
        rospy.loginfo(
            f'best : {best_region} ---- collisions : {", ".join(collision_regions)}'
        )

    # Robot has reached destination
    else:
        linear_x = accelerate(current_robot_twist.linear.x, 0, acceleration)
        angular_z = accelerate(
            -1 * current_robot_twist.angular.z,
            0,
            acceleration,
        )
        rospy.loginfo(f"ROBOT HAS REACHED ITS DESTINATION !! HORRAY !!!")

    rospy.loginfo(f"linear : {linear_x} ---- angular : {angular_z}")
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub
    rospy.init_node("region_navigation")

    # Publish velocities
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Recieve laser scan
    sub = rospy.Subscriber("/map", OccupancyGrid, callback_map)

    # laser_sub = rospy.Subscriber("/laser_scan", LaserScan, callback_laser)

    # Recieve goal info
    goal_sub = rospy.Subscriber("/update_goal", Point, update_goal)

    # Recieve robot odometry
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    # Listen for messages
    rospy.spin()


if __name__ == "__main__":
    main()
