/**
 * @file optimisation_local.cpp
 * @author moncef boukhecheba (moncef.boukhecheba@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-08-25
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
#include <random>
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
    float v_min;
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
    float linear_speed;
};

/**
 * @brief Calculate the euclidean distance between two 2D points.
 *
 * @param xa
 * @param ya
 * @param xb
 * @param yb
 * @return float
 */
float calculateEuclideanDistance(float xa, float ya, float xb, float yb)
{
    return sqrt(pow(xa - xb, 2) + pow(ya - yb, 2));
}

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

    // ROS_INFO_STREAM("GOAL INFORMATION : "
    //                << "x " << goal_point.pose.position.x << " y " << goal_point.pose.position.y);
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
    int robot_length_distance = ceil((ROBOT_LENGTH + robot_length_offset) / map_info->resolution) / 2;

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

// ========== TRAJECTORY FUNCTIONS ==========

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
 * @return
 */
bool generateTrajectory(TrajectoryParameters traj_parameters, std::vector<Pose2D> &trajectory,
                        int num_points_per_trajectory, float omega, float linear_speed)
{
    // Minimum distance so that the trajectory is valid
    float min_distance = 1.5;

    // Prepare a point object to update RVIZ markers
    geometry_msgs::Point p;
    p.z = 0;

    // Foreach point...
    float dt = traj_parameters.dt;
    for (int j = 0; j < num_points_per_trajectory; j++)
    {
        float new_x, new_y, new_theta;

        if (j != num_points_per_trajectory - 1)
        {
            // Update the point's coordinates using the previous point and the equation of the robot's cinematic model
            new_x = trajectory[j].x - linear_speed * dt * sin(trajectory[j].theta + (dt * omega) / 2.0);
            new_y = trajectory[j].y + linear_speed * dt * cos(trajectory[j].theta + (dt * omega) / 2.0);
            new_theta = trajectory[j].theta + dt * omega;

            trajectory[j + 1].x = new_x;
            trajectory[j + 1].y = new_y;
            trajectory[j + 1].theta = new_theta;
        }

        // If robot is in collision, don't add the point and break the trajectory
        if (is_robot_in_collision(new_x, new_y))
        {
            trajectory.clear();
            return true;
        }
    }

    // Prevent trajectory from looping
    while (calculateEuclideanDistance(trajectory.back().x, trajectory.back().y,
                                      currentRobotPose.x, currentRobotPose.y) < min_distance)
    {
        trajectory.pop_back();
    }

    if (trajectory.size() == 0)
    {
        return true;
    }

    return false;
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
        velocity_msg.angular.z = 0;
        velocity_msg.linear.x = 0;
        return velocity_msg;
    }

    // Return the velocity at the correct point
    velocity_msg.angular.z = best_trajectory.omega;
    velocity_msg.linear.x = best_trajectory.linear_speed;

    return velocity_msg;
}

class Particle
{
public:
    float linear_speed;
    float angluar_speed;
};

/**
 * @brief
 *
 * @param traj_parameters
 * @param particle
 * @param starting_pose
 * @return std::vector<Pose2D>
 */
std::vector<Pose2D> generateTrajectoryFromParticle(TrajectoryParameters traj_parameters, Particle particle, Pose2D starting_pose)
{
    // Calculate the number of points to be generated per trajectory
    int num_points_per_trajectory = round(traj_parameters.time_horizon / traj_parameters.dt);

    // Create the trajectory object
    std::vector<Pose2D> trajectory;

    // Fill point list with point objects
    for (int j = 0; j < num_points_per_trajectory; j++)
    {
        // Initialize point at current robot position
        Pose2D point;
        point.x = starting_pose.x;
        point.y = starting_pose.y;
        point.theta = starting_pose.theta;

        // Add point to list
        trajectory.push_back(point);
    }

    // Generate the trajectory from the starting point
    generateTrajectory(traj_parameters, trajectory, num_points_per_trajectory,
                       particle.angluar_speed, particle.linear_speed);

    return trajectory;
}

/**
 * @brief Calculate the particle's fitness
 *
 * @param traj_parameters
 * @param particle
 * @param starting_pose
 * @return float
 */
float calculateParticleFitness(TrajectoryParameters traj_parameters, Particle particle, Pose2D starting_pose)
{
    // ====== TRAJECTORY GENERATION ========
    std::vector<Pose2D> trajectory = generateTrajectoryFromParticle(traj_parameters, particle, starting_pose);

    // ======= FITNESS CALCULATION =========
    // If trajectory is obstructed, return infinity
    if (trajectory.size() == 0)
    {
        // ROS_INFO_STREAM("Trajectory Obstructed");
        return INFINITY;
    }

    // G1 = Calculate distance between trajectory's last point and goal
    float goal_distance = calculateEuclideanDistance(trajectory.back().x, trajectory.back().y,
                                                     goal_point.pose.position.x, goal_point.pose.position.y);

    // G2 = Calculate distance between trajectory's last point and robot (priviledge straight paths)
    float robot_distance = calculateEuclideanDistance(trajectory.back().x, trajectory.back().y,
                                                      currentRobotPose.x, currentRobotPose.y);

    // Initialize the weights and calculate the fitness
    float w_1 = 1;
    float w_2 = .5;
    float fitness = w_1 * goal_distance + w_2 * robot_distance;
    return fitness;
}

/**
 * @brief Apply the WOA alogithm to generate the best trajectory
 *
 * @param traj_parameters
 * @param num_iterations
 * @param num_particles
 * @param starting_pose
 * @return Particle
 */
BestTrajectory applyWOAAlgorithm(TrajectoryParameters &traj_parameters, int num_iterations, int num_particles,
                                 Pose2D &starting_pose, TrajectoryMarkers &markers, double starting_time)
{
    ROS_INFO_STREAM("Applying WOA...");

    BestTrajectory best_trajectory;

    // Initialize Random Number Generator
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());

    // Initialize best particle and best fitness
    Particle best_particle;
    std::uniform_real_distribution<float> linear_distr(traj_parameters.v_min, traj_parameters.v_max);
    std::uniform_real_distribution<float> angular_distr(traj_parameters.omega_min, traj_parameters.omega_max);
    best_particle.linear_speed = linear_distr(generator);
    best_particle.angluar_speed = angular_distr(generator);
    float best_fitness = INFINITY;

    // Initialize the particles (whales)
    std::vector<Particle> particles(num_particles);
    for (Particle particle : particles)
    {
        // Linear Speed
        particle.linear_speed = linear_distr(generator);

        // Angular Speed
        particle.angluar_speed = angular_distr(generator);

        // Calculate the fitness for each particle
        float fitness = calculateParticleFitness(traj_parameters, particle, starting_pose);

        // If fitness is best fitness
        if (fitness < best_fitness)
        {
            best_fitness = fitness;
            best_particle = particle;
        }
    }

    // Until the number of iterations is reached
    for (int i = 0; i < num_iterations; i++)
    {
        // Initialize temp variables
        Particle best_particle_temp = best_particle;
        float best_fitness_temp = best_fitness;

        // Initialize best vector
        std::vector<float> best_position_vector;
        best_position_vector.push_back(best_particle.linear_speed);
        best_position_vector.push_back(best_particle.angluar_speed);

        // Foreach particle
        for (int p_i = 0; p_i < num_particles; p_i++)
        {
            // Initialize particle vector
            std::vector<float> position_vector;
            position_vector.push_back(particles.at(p_i).linear_speed);
            position_vector.push_back(particles.at(p_i).angluar_speed);

            // Spiraling movement probability
            std::uniform_real_distribution<float> p_distr(0, 1);
            float p = p_distr(generator);

            // Spiraling movement
            if (p >= 0.5)
            {
                std::uniform_real_distribution<float> l_distr(-1, 1);
                float l = l_distr(generator);

                for (int j = 0; j < position_vector.size(); j++)
                {
                    float D_prime = abs(best_position_vector.at(j) - position_vector.at(j));
                    float new_value = D_prime * exp(l * PI / 2) * cos(2 * PI * l) * best_position_vector.at(j);
                    position_vector.at(j) = new_value;
                }
            }

            // Circling prey
            else
            {
                // Calculate a
                float a = 2 - i * 2 / (float)num_iterations;
                std::uniform_real_distribution<float> r_1_distr(0, 1);

                // Calculate A vector
                std::vector<float> A(2);
                for (int j = 0; j < A.size(); j++)
                {
                    float A_Value = 2 * a * r_1_distr(generator) - a;
                    A.at(j) = A_Value;
                }

                // Calculate the vector norm of A
                float A_norm = pow(A.at(0), 2) + pow(A.at(1), 2);
                std::vector<float> target_vector;

                // Exploration
                if (A_norm > 1)
                {
                    std::uniform_int_distribution<int> rand_distr(0, num_particles - 1);
                    int random_index = rand_distr(generator);

                    // Choose random particle to go through
                    target_vector.push_back(particles.at(random_index).linear_speed);
                    target_vector.push_back(particles.at(random_index).angluar_speed);
                }

                // Intensification
                else
                {
                    target_vector.push_back(best_particle.linear_speed);
                    target_vector.push_back(best_particle.angluar_speed);
                }

                // Update the position vector
                for (int j = 0; j < position_vector.size(); j++)
                {
                    std::uniform_real_distribution<float> r_2_distr(0, 1);
                    float C = 2 * r_2_distr(generator);
                    float D = abs(target_vector.at(j) * C - position_vector.at(j));
                    float new_value = target_vector.at(j) - A.at(j) * D;
                    position_vector.at(j) = new_value;
                }
            }

            // Correct linear speed
            if (position_vector.at(0) < traj_parameters.v_min)
            {
                position_vector.at(0) = traj_parameters.v_min;
            }
            else if (position_vector.at(0) > traj_parameters.v_max)
            {
                position_vector.at(0) = traj_parameters.v_max;
            }

            // Correct angular speed
            if (position_vector.at(1) < traj_parameters.omega_min)
            {
                position_vector.at(1) = traj_parameters.omega_min;
            }
            else if (position_vector.at(1) > traj_parameters.omega_max)
            {
                position_vector.at(1) = traj_parameters.omega_max;
            }

            particles.at(p_i).linear_speed = position_vector.at(0);
            particles.at(p_i).angluar_speed = position_vector.at(1);

            // Calculate the fitness for each particle
            float fitness = calculateParticleFitness(traj_parameters, particles.at(p_i), starting_pose);

            // If fitness is best fitness
            if (fitness < best_fitness)
            {
                best_fitness = fitness;
                best_particle = particles.at(p_i);
            }
        }
    }

    // Color the trajectory marker green
    markers.lines.at(0).color.g = 1;
    markers.lines.at(0).color.b = 0;
    std::vector<Pose2D> trajectory = generateTrajectoryFromParticle(traj_parameters, best_particle, starting_pose);
    geometry_msgs::Point p;
    p.z = 0;
    for (Pose2D point : trajectory)
    {
        p.x = point.x;
        p.y = point.y;
        markers.lines.at(0).points.push_back(p);
        markers.points.points.push_back(p);
    }

    best_trajectory.start_time = starting_time;
    best_trajectory.omega = best_particle.angluar_speed;
    best_trajectory.trajectory = trajectory;
    best_trajectory.linear_speed = best_particle.linear_speed;
    best_trajectory.markers = markers;
    return best_trajectory;
}

class ParticlePSO
{
public:
    Particle X;
    Particle X_pbest;
    float pbest_fitness;
    Particle velocity;
};

BestTrajectory applyPSOAlgorithm(TrajectoryParameters &traj_parameters, int num_iterations, int num_particles,
                                 Pose2D &starting_pose, TrajectoryMarkers &markers, double starting_time)
{

    // ======= SWARM INTELLIGENCE COMMON INITIALIZATION =======
    BestTrajectory best_trajectory;

    // Initialize Random Number Generator
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());

    // Initialize best particle and best fitness
    Particle best_particle;
    std::uniform_real_distribution<float> linear_distr(traj_parameters.v_min, traj_parameters.v_max);
    std::uniform_real_distribution<float> angular_distr(traj_parameters.omega_min, traj_parameters.omega_max);
    best_particle.linear_speed = linear_distr(generator);
    best_particle.angluar_speed = angular_distr(generator);
    float best_fitness = INFINITY;

    // Initialize the particles
    std::vector<ParticlePSO> particles(num_particles);
    for (ParticlePSO particle : particles)
    {
        // Linear Speed
        particle.X.linear_speed = linear_distr(generator);

        // Angular Speed
        particle.X.angluar_speed = angular_distr(generator);

        // Pbest init
        particle.X_pbest.angluar_speed = particle.X.angluar_speed;
        particle.X_pbest.linear_speed = particle.X.linear_speed;

        // Calculate the fitness for each particle
        float fitness = calculateParticleFitness(traj_parameters, particle.X, starting_pose);
        particle.pbest_fitness = fitness;

        // If fitness is best fitness
        if (fitness < best_fitness)
        {
            best_fitness = fitness;
            best_particle = particle.X;
        }
    }

    // ====== PSO ALGORITHM =======
    ROS_INFO_STREAM("Applying PSO...");
    float c_1 = 0.5; // Cognitive parameter
    float c_2 = 0.5; // Social parameter

    // Until the number of iterations is reached
    for (int i = 0; i < num_iterations; i++)
    {
        // Initialize temp variables
        Particle best_particle_temp = best_particle;
        float best_fitness_temp = best_fitness;

        // Initialize best vector
        std::vector<float> gbest_position_vector;
        gbest_position_vector.push_back(best_particle.linear_speed);
        gbest_position_vector.push_back(best_particle.angluar_speed);

        // Foreach particle
        for (int p_i = 0; p_i < num_particles; p_i++)
        {
            // Initialize particle vector
            std::vector<float> position_vector;
            position_vector.push_back(particles.at(p_i).X.linear_speed);
            position_vector.push_back(particles.at(p_i).X.angluar_speed);

            // Initialize pbest position_vector
            std::vector<float> pbest_position_vector;
            pbest_position_vector.push_back(particles.at(p_i).X_pbest.linear_speed);
            pbest_position_vector.push_back(particles.at(p_i).X_pbest.angluar_speed);

            // Initialize velocity vector
            std::vector<float> velocity_vector;
            velocity_vector.push_back(particles.at(p_i).velocity.linear_speed);
            velocity_vector.push_back(particles.at(p_i).velocity.angluar_speed);

            // Rand 1
            std::uniform_real_distribution<float> r_1_distr(0, 1);
            float r_1 = r_1_distr(generator);

            // Rand 2
            std::uniform_real_distribution<float> r_2_distr(0, 1);
            float r_2 = r_2_distr(generator);

            for (int j = 0; j < position_vector.size(); j++)
            {
                // Calculate speed of particle
                float particle_velocity = velocity_vector.at(j) + (c_1 * r_1 * pbest_position_vector.at(j)) +
                                          (c_2 * r_2 * gbest_position_vector.at(j));

                // Update position of particle
                position_vector.at(j) = position_vector.at(j) + particle_velocity;
            }

            // Correct linear speed
            if (position_vector.at(0) < traj_parameters.v_min)
            {
                position_vector.at(0) = traj_parameters.v_min;
            }
            else if (position_vector.at(0) > traj_parameters.v_max)
            {
                position_vector.at(0) = traj_parameters.v_max;
            }

            // Correct angular speed
            if (position_vector.at(1) < traj_parameters.omega_min)
            {
                position_vector.at(1) = traj_parameters.omega_min;
            }
            else if (position_vector.at(1) > traj_parameters.omega_max)
            {
                position_vector.at(1) = traj_parameters.omega_max;
            }

            particles.at(p_i).X.linear_speed = position_vector.at(0);
            particles.at(p_i).X.angluar_speed = position_vector.at(1);

            // Calculate the fitness for particle
            float fitness = calculateParticleFitness(traj_parameters, particles.at(p_i).X, starting_pose);

            // If fitness is best fitness, update gbest
            if (fitness < best_fitness)
            {
                best_fitness = fitness;
                best_particle = particles.at(p_i).X;
            }

            // If fitness is personal best fitness, update pbest
            if (fitness < particles.at(p_i).pbest_fitness)
            {
                particles.at(p_i).pbest_fitness = fitness;
                particles.at(p_i).X_pbest = particles.at(p_i).X;
            }
        }
    }

    // Color the trajectory marker green
    markers.lines.at(0).color.g = 1;
    markers.lines.at(0).color.b = 0;
    std::vector<Pose2D> trajectory = generateTrajectoryFromParticle(traj_parameters, best_particle, starting_pose);
    geometry_msgs::Point p;
    p.z = 0;
    for (Pose2D point : trajectory)
    {
        p.x = point.x;
        p.y = point.y;
        markers.lines.at(0).points.push_back(p);
        markers.points.points.push_back(p);
    }

    best_trajectory.start_time = starting_time;
    best_trajectory.omega = best_particle.angluar_speed;
    best_trajectory.trajectory = trajectory;
    best_trajectory.linear_speed = best_particle.linear_speed;
    best_trajectory.markers = markers;
    return best_trajectory;
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
    ros::init(argc, argv, "optimization_navigation");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Choose optimisation algorithm
    std::string algorithm = "WOA";
    nh.getParam("algorithm", algorithm);

    // Advertise the robot trajectory
    std::string marker_topic = "/robot_trajectory";
    nh.getParam("marker_topic", marker_topic);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);

    // Advertise the command velocity
    std::string cmd_vel_topic = "/cmd_vel";
    nh.getParam("cmd_vel_topic", cmd_vel_topic);
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

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
    ros::Subscriber goal_sub = n.subscribe(goal_topic, 1, goalCallback);

    // Subscribe to goal point data
    std::string clock_topic = "/clock";
    nh.getParam("clock_topic", clock_topic);
    ros::Subscriber clock_sub = n.subscribe(clock_topic, 1, clockCallback);

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
    ros::Rate loop_rate(30);

    // ===== TRAJECTORY PARAMETERS =====
    int max_stack_size = 1;
    std::deque<BestTrajectory> best_trajectory_deque;
    TrajectoryParameters traj_parameters;
    traj_parameters.v_max = 1;          // Max linear speed (m / s)
    traj_parameters.v_min = 0.25;       // Min linear speed (m / s)
    traj_parameters.linear_accel = 1;   // Constant linear acceleration / deceleration (m / s²)
    traj_parameters.dt = 0.2;           // Time of the step between each point in the trajectory (s)
    traj_parameters.time_horizon = 3.5; // Total time of the planification (s)

    int num_trajectories = 1; // Total number of trajectories

    // Omega (angular speed) configuration
    // Generate omegas for each trajectory : divide space into n = num_trajectories omega speeds
    traj_parameters.omega_max = 1;
    traj_parameters.omega_min = -1 * traj_parameters.omega_max;
    traj_parameters.omega_range = traj_parameters.omega_max - traj_parameters.omega_min;
    traj_parameters.omega_increment = traj_parameters.omega_range / (num_trajectories - 1);

    // Threshold distance from goal to consider that the robot arrived to destination
    float distance_threshold = .75;

    // ====== OPTIMIZATION PARAMETERS =======
    int num_iterations = 100;
    int num_particles = 15;

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
                ROS_ERROR("Retrieved static_map !");
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
        float distance_from_goal = calculateEuclideanDistance(currentRobotPose.x, currentRobotPose.y,
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
            ROS_INFO_STREAM("Calculating new trajectory");

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

            /*
            else
            {
                starting_pose = best_trajectory_deque.back().trajectory.back();
                starting_time = current_time.clock.toSec() + (traj_parameters.time_horizon * best_trajectory_deque.size());
                // ROS_INFO_STREAM("start pose : x : " << starting_pose.x
                //                                    << " y " << starting_pose.y << " size " << best_trajectory_deque.size());
            }*/

            TrajectoryMarkers markers = initializeTrajectoryMarkers(num_trajectories, markers_frame_id, markers_namespace);

            // Start timer
            auto start = std::chrono::high_resolution_clock::now();

            if (algorithm == "WOA")
            {
                best_trajectory_deque.push_back(applyWOAAlgorithm(traj_parameters, num_iterations, num_particles,
                                                                  starting_pose, markers, starting_time));
            }
            else if (algorithm == "PSO")
            {
                best_trajectory_deque.push_back(applyPSOAlgorithm(traj_parameters, num_iterations, num_particles,
                                                                  starting_pose, markers, starting_time));
            }
            else
            {
                ROS_INFO_STREAM("ALGORITHM NOT VALID");
                break;
            }

            // Stop timer
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            ROS_INFO_STREAM("Time Elapsed : " << duration.count());

            // Publish marker information into RVIZ
            marker_pub.publish(markers.points);
            for (visualization_msgs::Marker line_strip : markers.lines)
            {
                marker_pub.publish(line_strip);
            }
        }

        // Make the robot move using command velocity
        if (best_trajectory_deque.size() != 0)
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

        // Publish the velocity command to the robot
        velocity_pub.publish(velocity_msg);

        // Sleep to update with the frequency rate
        loop_rate.sleep();

        // Listen to upcoming messages
        ros::spinOnce();
    }
}
