/**
 * @file iss.cpp
 * @author moncef boukhecheba (moncef.boukhecheba@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#define PI 3.14159265
#define ROBOT_LENGTH 1 // Meters squared

#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <tf/tf.h>
#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <deque>
#include <chrono>

/**
 * @brief Short class used to store the 2D position of the robot
 */
class Pose2D
{
public:
    float x = 0;
    float y = 0;
    float theta = 0;
    float v = 0;
};

/**
 * @brief Class that encapsulates a marker for the points of the trajectory AND a list of markers for each of the trajectory lines.
 *
 * If we don't create a list for line strips and let one marker handle the job,
 * this will create a single line strip that will return to the origin at the end of each trajectory, creating a spider like structure that we don't want.
 *
 */
class TrajectoryMarkers
{
public:
    visualization_msgs::Marker points;
    std::vector<visualization_msgs::Marker> lines;
    TrajectoryMarkers() {}
    TrajectoryMarkers(visualization_msgs::Marker p, std::vector<visualization_msgs::Marker> l)
    {
        points = p;
        lines = l;
    }
};

struct TrajectoryParameters
{
    float v_max;
    float linear_accel;
    float dt;
    float time_horizon;
    float omega_max;
    float omega_min;
    float omega_range;
    float omega_increment;
};

struct TrajectoryList
{
    std::vector<float> omega_list;
    std::vector<std::vector<Pose2D>> traj_list;
};

struct BestTrajectory
{
    std::vector<Pose2D> trajectory;
    double start_time;
    TrajectoryMarkers markers;
    float omega;
};

// ============= ODOMETRY ===============
// Robot current position
Pose2D currentRobotPose;
geometry_msgs::Twist currentRobotSpeeds;
bool robot_pose_updated = false;

/**
 * @brief Update robot pose with odometry information
 *
 * @param msg
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Update robot current position
    currentRobotPose.x = msg->pose.pose.position.x;
    currentRobotPose.y = msg->pose.pose.position.y;

    // Transform quaternion to RPY (roll-pitch-yaw)
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Update Robot Orientation
    currentRobotPose.theta = yaw - PI / 2;
    robot_pose_updated = true;

    // Update robot speeds
    currentRobotPose.v = msg->twist.twist.linear.x;
}

// ================ LASER DATA ===============
// Number of total readings (distances) in a laser scan
int num_readings = -1;
std::vector<double> ranges_scan(1);

// Laser scan info
float resolution = 0;

/**
 * @brief Get laser data and store it in an array
 *
 * @param scan Laser scan
 */
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Update total number of readings in a scan
    if (num_readings == -1)
    {
        resolution = scan->angle_increment;
        num_readings = (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
        ranges_scan = std::vector<double>(num_readings);
    }

    // Copy scan values into the "range_scan" array
    for (int i = 0; i < num_readings; ++i)
    {
        ranges_scan[i] = scan->ranges[i];
    }
}

// ================ GOAL POINT DATA ===============
geometry_msgs::PoseStamped goal_point;
bool goal_point_updated;

/**
 * @brief Update the goal point
 *
 * @param goal
 */
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    goal_point.pose = goal->pose;
    goal_point_updated = true;
    ROS_INFO_STREAM("New Goal Point "
                    << "x " << goal_point.pose.position.x << " y " << goal_point.pose.position.y);
}

// ================ CLOCK DATA ===============
rosgraph_msgs::Clock current_time;

/**
 * @brief Update the clock
 *
 * @param current_clock
 */
void clockCallback(const rosgraph_msgs::Clock::ConstPtr &current_clock)
{
    current_time.clock = current_clock->clock;
}

// ================ MAP DATA ===============
// Store the map inside a 1D vector of size = num_cells
std::vector<int8_t, std::allocator<int8_t>> static_map;
nav_msgs::MapMetaData *map_info;
float offset_x;
float offset_y;

/**
 * @brief Get map data and store it in a global variable
 *
 * @param occupancy_grid Map generated from
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid)
{
    // Update map information if none
    if (map_info == nullptr)
    {
        map_info = new nav_msgs::MapMetaData(occupancy_grid->info);
        offset_x = round(map_info->width / 2);
        offset_y = round(map_info->height / 2);
    }

    static_map = occupancy_grid->data;
}

/**
 * @brief Get a map cell from a x, y euler coordinates
 *
 * @param x
 * @param y
 * @return int
 */
int coordinates_to_cell(float x, float y)
{
    x = offset_x + round(x / map_info->resolution);
    y = (offset_y + round(y / map_info->resolution)) * map_info->width;
    return static_map[x + y];
}

/**
 * @brief Given a 2D point, determine if the robot positioned at that point will hit an obstacle.
 *
 * @param x
 * @param y
 * @return true
 * @return false
 */
bool is_robot_in_collision(float x, float y)
{
    // Collision radius = Robot length + offset
    float robot_length_offset = .5;
    int robot_length_distance = ceil((ROBOT_LENGTH + robot_length_offset) / map_info->resolution) / 2.0;

    int y_cell = ceil(y / map_info->resolution) + offset_y;
    int y_start = (y_cell - robot_length_distance) * map_info->width;
    int y_end = (y_cell + robot_length_distance) * map_info->width;

    int x_cell = ceil(x / map_info->resolution) + offset_x;
    int x_start = x_cell - robot_length_distance;
    int x_end = x_cell + robot_length_distance;

    for (int y = y_start; y <= y_end; y = y + map_info->width)
    {
        for (int x = x_start; x <= x_end; x++)
        {
            // ROS_INFO_STREAM("x_start " << x_start << " x_end" << x_end << " y_start " << y_start << " y_end " << y_end);

            if (static_map[x + y] == 100)
                return true;
        }
    }

    return false;
}

/**
 * @brief Initialize a single marker in RVIZ with default scale and color
 *
 * @param frame_id
 * @param ns
 * @param type
 * @param action
 * @param id
 * @return visualization_msgs::Marker
 */
visualization_msgs::Marker initializeRvizMarker(const std::string &frame_id, const std::string &ns, u_short type, u_short action, u_short id)
{
    // General marker information
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;      // Frame
    marker.header.stamp = ros::Time::now(); // Stamp
    marker.ns = ns;                         // Namespace
    marker.action = action;                 // Marker action
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = type;

    return marker;
}

// ================ RVIZ ==================
/**
 * @brief Initialize the trajectory markers to add to RVIZ
 *
 * @param frame_id frame id of the markers
 * @param ns namespace of the markers
 * @return TrajectoryMarkers
 */
TrajectoryMarkers initializeTrajectoryMarkers(int num_trajectories, const std::string &frame_id, const std::string &ns)
{
    // ======= POINTS =========
    // Initialize sphere list
    visualization_msgs::Marker sphere_list = initializeRvizMarker(frame_id, ns, visualization_msgs::Marker::SPHERE_LIST,
                                                                  visualization_msgs::Marker::ADD, 0);
    sphere_list.scale.x, sphere_list.scale.y, sphere_list.scale.z = 0.1;

    // Color the points red
    sphere_list.color.r = 1.0f;
    sphere_list.color.a = 1.0;

    // ======= LINES ==========
    // Initialize line strip list
    std::vector<visualization_msgs::Marker> line_strip_list;

    // Fill list with line strips
    for (int i = 0; i < num_trajectories; i++)
    {
        visualization_msgs::Marker line_strip = initializeRvizMarker(frame_id, ns, visualization_msgs::Marker::LINE_STRIP,
                                                                     visualization_msgs::Marker::ADD, i + 1);

        // Color the lines blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.scale.x = 0.05;
        line_strip_list.push_back(line_strip);
    }

    // Return trajectory markers
    TrajectoryMarkers markers(sphere_list, line_strip_list);
    return markers;
}

// ========== TRAJECTORY GENERATION ==========
/**
 * @brief Create a Trajectory List object
 *
 * @param num_trajectories number of trajectories
 * @param num_points_per_trajectory number of points per trajectory
 * @param starting_pose the pose where the generated trajectories all start
 * @return std::vector<std::vector<Pose2D>> list of lists of Pose2D objects all initialized at starting pose
 */
std::vector<std::vector<Pose2D>> createTrajectoryList(int num_trajectories, int num_points_per_trajectory, Pose2D &starting_pose)
{
    std::vector<std::vector<Pose2D>> traj_list;
    for (int i = 0; i < num_trajectories; i++)
    {
        std::vector<Pose2D> point_list;

        // Fill point list with point objects
        for (int j = 0; j < num_points_per_trajectory; j++)
        {
            // Initialize point at current robot position
            Pose2D point;
            point.x = starting_pose.x;
            point.y = starting_pose.y;
            point.theta = starting_pose.theta;
            point.v = starting_pose.v;

            // Add point to list
            point_list.push_back(point);
        }

        // Add point list (trajectory) to list of trajectories
        traj_list.push_back(point_list);
    }

    return traj_list;
}

/**
 * @brief Using the kinematic model of the vehicle, generate a single trajectory given all the necessary variables.
 *
 * @param traj_parameters parameters of the trajectory
 * @param trajectory the actual list of points the trajectory will be stored into
 * @param num_points_per_trajectory number of points per trajectory
 * @param omega the angular speed set for the current trajectory
 * @param markers the markers object
 * @param p a point object used to update the markers
 * @param index index of the current trajectory in the list (used to update the lines marker)
 * @return true the trajectory is free
 * @return false the trajectory is obstructed by an obstacle
 */
bool generateTrajectory(TrajectoryParameters traj_parameters, std::vector<Pose2D> &trajectory,
                        int num_points_per_trajectory, float omega,
                        TrajectoryMarkers &markers, int index)
{
    // Prepare a point object to update RVIZ markers
    geometry_msgs::Point p;
    p.z = 0;

    // Foreach point...
    float dt = traj_parameters.dt;
    float linear_accel = traj_parameters.linear_accel;
    for (int j = 0; j < num_points_per_trajectory; j++)
    {
        float new_x, new_y, new_theta, new_v;

        if (j != num_points_per_trajectory - 1)
        {
            // Update the point's coordinates using the previous point and the equation of the robot's cinematic model
            new_v = trajectory[j].v + dt * linear_accel;
            new_x = trajectory[j].x - trajectory[j].v * dt * sin(trajectory[j].theta + (dt * omega) / 2);
            new_y = trajectory[j].y + trajectory[j].v * dt * cos(trajectory[j].theta + (dt * omega) / 2);
            new_theta = trajectory[j].theta + dt * omega;

            trajectory[j + 1].x = new_x;
            trajectory[j + 1].y = new_y;
            trajectory[j + 1].theta = new_theta;

            // Limit linear speed to v_max
            if (new_v >= traj_parameters.v_max)
            {
                trajectory[j + 1].v = traj_parameters.v_max;
            }
            else
            {
                trajectory[j + 1].v = new_v;
            }
        }

        // ==== Update RVIZ markers ====
        p.x = trajectory[j].x;
        p.y = trajectory[j].y;

        // Update points marker
        markers.points.points.push_back(p);

        // Draw lines between points
        markers.lines.at(index).points.push_back(p);

        // If robot is in collision, don't add the point and break the trajectory
        if (is_robot_in_collision(new_x, new_y))
        {
            trajectory.clear();
            return false;
        }
    }

    return true;
}

/**
 * @brief Samples a list of trajectories from the robots' possible future trajectories
 *
 * @param traj_parameters
 * @param num_trajectories
 * @param starting_pose
 * @param markers
 * @return TrajectoryList
 */
TrajectoryList generateTrajectoryListFromPoint(TrajectoryParameters traj_parameters, int num_trajectories, Pose2D &starting_pose, TrajectoryMarkers &markers)
{
    // Calculate the number of points to be generated per trajectory
    int num_points_per_trajectory = round(traj_parameters.time_horizon / traj_parameters.dt);

    // ==== TRAJECTORY LIST CREATION ====
    TrajectoryList trajectory_list;
    trajectory_list.traj_list = createTrajectoryList(num_trajectories, num_points_per_trajectory, starting_pose);

    // Foreach angular speed, create a trajectory
    float omega;
    for (int i = 0; i < num_trajectories; i++)
    {
        omega = (traj_parameters.omega_range / 2 - traj_parameters.omega_range) + traj_parameters.omega_increment * i;
        trajectory_list.omega_list.push_back(omega);

        bool trajectory_free = generateTrajectory(traj_parameters, trajectory_list.traj_list[i], num_points_per_trajectory,
                                                  omega, markers, i);

        // Color the trajectory red if it is obstructed
        // ==== TEST ONLY ====
        if (!trajectory_free)
        {
            markers.lines.at(i).color.r = 1;
            markers.lines.at(i).color.b = 0;
        }
    }

    return trajectory_list;
}

/**
 * @brief Calculate the euclidean distance between two 2D points.
 *
 * @param xa
 * @param ya
 * @param xb
 * @param yb
 * @return float
 */
float calculate_euclidean_distance(float xa, float ya, float xb, float yb)
{
    return sqrt(pow(xa - xb, 2) + pow(ya - yb, 2));
}

/**
 * @brief
 *
 * @param trajectory_list
 * @param markers
 * @return std::vector<Pose2D>
 */
BestTrajectory selectBestTrajectory(TrajectoryList &trajectory_list, TrajectoryMarkers &markers, double starting_time)
{
    float best_distance = INFINITY;
    float distance;
    int best_trajectory_index = 0;

    for (int i = 0; i < trajectory_list.traj_list.size(); i++)
    {
        int traj_size = trajectory_list.traj_list[i].size();

        // Ignore trajectory if empty (trajectory is obstructed)
        if (traj_size == 0)
        {
            continue;
        }

        // Calculate distance between trajectory's last point and goal
        distance = calculate_euclidean_distance(trajectory_list.traj_list[i][traj_size - 1].x, trajectory_list.traj_list[i][traj_size - 1].y,
                                                goal_point.pose.position.x, goal_point.pose.position.y);

        // If distance is best distance
        if (distance < best_distance)
        {
            best_distance = distance;
            best_trajectory_index = i;
        }
    }

    BestTrajectory best_trajectory;

    if (distance != INFINITY)
    {
        // Color the trajectory marker green
        markers.lines.at(best_trajectory_index).color.g = 1;
        markers.lines.at(best_trajectory_index).color.b = 0;

        best_trajectory.start_time = starting_time;
        best_trajectory.omega = trajectory_list.omega_list[best_trajectory_index];
        best_trajectory.trajectory = trajectory_list.traj_list[best_trajectory_index];
        best_trajectory.markers = markers;
    }

    return best_trajectory;
}

/**
 * @brief Compute the best controls so that the robot follows the trajectory
 *
 * @param trajectory_parameters
 * @param best_trajectory_deque
 * @return geometry_msgs::Twist
 */
geometry_msgs::Twist createControlsFromTrajectory(geometry_msgs::Twist &velocity_msg, TrajectoryParameters &trajectory_parameters,
                                                  std::deque<BestTrajectory> &best_trajectory_deque)
{
    BestTrajectory &best_trajectory = best_trajectory_deque.front();

    // Using the clock, calculate the point at which the robot should be at and use the velocity commands at that point
    int index = floor((current_time.clock.toSec() - best_trajectory.start_time) / trajectory_parameters.dt);
    // ROS_INFO_STREAM("index " << index << " time elapsed " << current_time.clock.toSec() - best_trajectory.start_time);

    // If no best trajectory is found, make the robot turn around
    if (best_trajectory.trajectory.size() == 0)
    {

        velocity_msg.angular.z = 1;
        velocity_msg.linear.x = 0;
        best_trajectory_deque.pop_front();
        return velocity_msg;
    }

    // If the robot finished the trajectory, remove the trajectory from the queue
    if (index >= best_trajectory.trajectory.size())
    {
        best_trajectory_deque.pop_front();
        return velocity_msg;
    }

    // Return the velocity at the correct point
    velocity_msg.angular.z = best_trajectory.omega;
    velocity_msg.linear.x = best_trajectory.trajectory.at(index).v;

    return velocity_msg;
}

/**
 * @brief Checks if the odometry information and the map are available before starting the navigation algorithm.
 *
 * @return true The requirements are met
 * @return false Some requirements are missing
 */
bool checkIfRequirementsAreMet()
{
    if (!robot_pose_updated)
    {
        ROS_INFO_STREAM("Waiting for odometry information...");
        return false;
    }

    if (static_map.size() == 0)
    {
        ROS_INFO_STREAM("Map information not recieved or is incorrect...");
        return false;
    }

    if (!goal_point_updated)
    {
        ROS_INFO_STREAM("Goal information not yet recieved ...");
        return false;
    }

    return true;
}

// ================ MAIN =====================
int main(int argc, char **argv)
{
    // Initialization
    ros::init(argc, argv, "spheres");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int frequency = 30;

    // Advertise the robot trajectory
    std::string marker_topic = "/robot_trajectory";
    nh.getParam("marker_topic", marker_topic);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, frequency);

    // Advertise the command velocity
    std::string cmd_vel_topic = "/cmd_vel";
    nh.getParam("cmd_vel_topic", cmd_vel_topic);
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, frequency * 2);

    // Subscribe to odometry
    std::string odom_topic = "/odom";
    nh.getParam("odom_topic", odom_topic);
    ros::Subscriber odom_sub = n.subscribe(odom_topic, 1, odomCallback);

    // Subscribe to laser data
    std::string scan_topic = "/laser_scan";
    nh.getParam("scan_topic", scan_topic);
    ros::Subscriber laser_sub = n.subscribe(scan_topic, 1, laserCallback);

    // Subscribe to goal point data
    std::string goal_topic = "/move_base_simple/goal";
    nh.getParam("goal_topic", goal_topic);
    ros::Subscriber goal_sub = n.subscribe(goal_topic, frequency, goalCallback);

    // Subscribe to goal point data
    std::string clock_topic = "/clock";
    nh.getParam("clock_topic", clock_topic);
    ros::Subscriber clock_sub = n.subscribe(clock_topic, frequency, clockCallback);

    // Map data -- Option 1 : Subscribe to map topic
    std::string use_map_topic = "true";
    nh.getParam("use_map_topic", use_map_topic);

    ros::Subscriber map_sub;
    if (use_map_topic == "true")
    {
        std::string map_topic = "/map";
        nh.getParam("map_topic", map_topic);
        map_sub = n.subscribe(map_topic, 1, mapCallback);
    }

    // Publish at rate in Hz
    ros::Rate loop_rate(frequency);

    // ===== TRAJECTORY PARAMETERS =====
    TrajectoryParameters traj_parameters;
    traj_parameters.v_max = 1;        // Max linear speed (m / s)
    traj_parameters.linear_accel = 1; // Constant linear acceleration / deceleration (m / s²)
    traj_parameters.dt = 0.2;         // Time of the step between each point in the trajectory (s)
    traj_parameters.time_horizon = 2; // Total time of the planification (s)

    int num_trajectories = 10; // Total number of trajectories

    // Omega (angular speed) configuration
    // Generate omegas for each trajectory : divide space into n = num_trajectories omega speeds
    traj_parameters.omega_max = 1;
    traj_parameters.omega_min = -1 * traj_parameters.omega_max;
    traj_parameters.omega_range = traj_parameters.omega_max - traj_parameters.omega_min;
    traj_parameters.omega_increment = traj_parameters.omega_range / (num_trajectories - 1);

    // Threshold distance
    float distance_threshold = 1; // in meters

    // ====== PARTIAL MOTION PLANNER PARAMETERS ======
    int max_stack_size = 1;
    std::deque<BestTrajectory> best_trajectory_deque;

    // ====== RVIZ MARKERS INITIALIZATION =========
    std::string markers_frame_id = "odom";
    std::string markers_namespace = "traj";

    // Create velocity message
    geometry_msgs::Twist velocity_msg;

    // ====== MAIN LOOP =======
    while (ros::ok())
    {
        // Map data -- Option 2 : Use service call (static map service)
        if (use_map_topic == "false" && static_map.size() == 0)
        {
            ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("static_map");
            nav_msgs::GetMap srv;
            if (map_client.call(srv))
            {
                boost::shared_ptr<nav_msgs::OccupancyGrid> map(new nav_msgs::OccupancyGrid(srv.response.map));
                mapCallback(map);
            }
            else
            {
                ROS_ERROR("Failed to call service static_map");
            }
        }

        // If some necessary information is not yet recieved, do not continue
        if (!checkIfRequirementsAreMet())
        {
            ros::spinOnce();
            continue;
        }

        // Check if robot arrived to the destination
        float distance_from_goal = calculate_euclidean_distance(currentRobotPose.x, currentRobotPose.y,
                                                                goal_point.pose.position.x, goal_point.pose.position.y);
        if (distance_from_goal < distance_threshold)
        {
            ROS_INFO_STREAM("Robot arrived to destination");

            // Tell the robot to stop
            velocity_msg.linear.x = 0;
            velocity_msg.angular.z = 0;
            velocity_pub.publish(velocity_msg);
            ros::spinOnce();

            continue;
        }

        // ===== TRAJECTORY GENERATION =======
        if (best_trajectory_deque.size() < max_stack_size)
        {
            // Starting pose is current robot pose
            Pose2D starting_pose;
            double starting_time;

            if (best_trajectory_deque.size() == 0)
            {
                starting_pose.x = currentRobotPose.x;
                starting_pose.y = currentRobotPose.y;
                starting_pose.theta = currentRobotPose.theta;
                starting_pose.v = currentRobotPose.v;
                starting_time = current_time.clock.toSec();
            }
            else
            {
                starting_pose = best_trajectory_deque.back().trajectory.back();
                starting_time = current_time.clock.toSec() + (traj_parameters.time_horizon * best_trajectory_deque.size());
            }

            TrajectoryMarkers markers = initializeTrajectoryMarkers(num_trajectories, markers_frame_id, markers_namespace);

            // Generate the trajectories
            auto start = std::chrono::high_resolution_clock::now();
            TrajectoryList trajectory_list = generateTrajectoryListFromPoint(traj_parameters, num_trajectories, starting_pose, markers);

            // Select best control from trajectory list
            best_trajectory_deque.push_back(selectBestTrajectory(trajectory_list, markers, starting_time));
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
            ROS_INFO_STREAM("Time Elapsed : " << duration.count());
        }

        // Make the robot move using command velocity
        if (best_trajectory_deque.size() != 0)
        {
            if (best_trajectory_deque.front().trajectory.size() == 0)
            {
                velocity_msg.angular.z = 1;
                velocity_msg.linear.x = 0;
                best_trajectory_deque.pop_front();
                ;
            }
            else
            {
                // Publish marker information into RVIZ
                marker_pub.publish(best_trajectory_deque.front().markers.points);
                for (visualization_msgs::Marker line_strip : best_trajectory_deque.front().markers.lines)
                {
                    marker_pub.publish(line_strip);
                }

                // Compute the best controls to follow the best trajectory
                velocity_msg = createControlsFromTrajectory(velocity_msg, traj_parameters, best_trajectory_deque);
            }
        }

        ROS_INFO_STREAM("Current Velocity : "
                        << "Linear : " << velocity_msg.linear.x << " Angular : " << velocity_msg.angular.z);

        ROS_INFO_STREAM("Distance from goal : " << distance_from_goal);

        // Publish the velocity command to the robot
        velocity_pub.publish(velocity_msg);

        // Sleep to update with the frequency rate
        loop_rate.sleep();

        // Listen to upcoming messages
        ros::spinOnce();
    }
}
