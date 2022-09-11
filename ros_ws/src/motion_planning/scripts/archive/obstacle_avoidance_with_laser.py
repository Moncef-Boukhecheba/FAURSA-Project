#! /usr/bin/env python3
import math
from turtle import update
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ==== INITIALIZATION VARIABLES ====
# Publisher (global because it is used in a subscriber callback)
pub = None

# Goal point (updated by an external node)
goal = None

# Current robot pose
current_robot_pose = None

# Current robot speed
current_robot_twist = None

# Laser scan
laser_scan = None

# Dictionary mapping obstacle directions to angular and linear velocities
regions_to_cmd = {"left": (1, -1), "right": (1, 1), "front": (1, 0), "back": (0, 1)}

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


def callback_laser(msg):
    """Get laser scan data and separate the ranges into zones, and navigate to goal while avoiding obstacles"""
    global laser_scan
    laser_scan = msg

    # If robot position is known, and there is a goal set, try to avigate to it
    if current_robot_pose != None and goal != None:
        navigate()


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
    """Separate laser scan readings into regions"""
    d = len(ranges) // 3

    regions = {
        "left": ranges[:d],
        "front": ranges[d : d * 2],
        "right": ranges[d * 2 :],
    }

    return regions


def is_region_occupied(ranges, num_slices, threshold_dist):
    slice_size = len(ranges) // num_slices
    min_values = [
        min(ranges[(slice_size * i) : (slice_size * (i + 1))])
        for i in range(num_slices - 1)
    ]
    return all([val < threshold_dist for val in min_values])


def navigate():
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
        regions = separate_ranges(laser_scan.ranges)

        # Calculate a region's angle range (in radians)
        region_size = (laser_scan.angle_max - laser_scan.angle_min) / len(regions)

        i = 1
        # For each region with its readings...
        for region, ranges in regions.items():
            num_slices = 2

            # If region is occupied
            if is_region_occupied(ranges, num_slices, threshold_dist):
                collision_regions.append(region)
            else:
                # Get coordinates of a point located in the middle of the region
                point_theta_before = (region_size * len(regions) - i) + region_size / 2
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
    sub = rospy.Subscriber("/laser_scan", LaserScan, callback_laser)

    # Recieve goal info
    goal_sub = rospy.Subscriber("/update_goal", Point, update_goal)

    # Recieve robot odometry
    odom_sub = rospy.Subscriber("/odom", Odometry, update_odom)

    # Listen for messages
    rospy.spin()


if __name__ == "__main__":
    main()
