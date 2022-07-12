#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include <vector>
#define PI 3.14159265
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <string>

/**
 * @brief Short class used to store the 2D position of the robot
 */
class Pose2D
{
public:
    float x = 0;
    float y = 0;
    float theta = 0;
};

/**
 * @brief Class that encapsulates a marker for the points of the trajectory AND a list of markers for each of the trajectory lines.
 *
 * If we don't create a list for line strips and let one marker handle the job,
 * this will create a single line strip that will return to the origin at the end of each trajectory, creating a spider like structure we don't want.
 *
 */
class TrajectoryMarkers
{
public:
    visualization_msgs::Marker points;
    std::vector<visualization_msgs::Marker> lines;
    TrajectoryMarkers(visualization_msgs::Marker p, std::vector<visualization_msgs::Marker> l)
    {
        points = p;
        lines = l;
    }
};

// Robot current position
Pose2D currentRobotPose;
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
    currentRobotPose.theta = yaw;
    robot_pose_updated = true;
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

int main(int argc, char **argv)
{
    // Initialization
    ros::init(argc, argv, "spheres");
    ros::NodeHandle n;
    ros::NodeHandle n_sub;

    // Advertise the robot trajectory
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("robot_trajectory", 1);

    // Subscribe to odometry
    std::string odom_topic = "/odom";
    ros::Subscriber sub = n_sub.subscribe(odom_topic, 1, odomCallback);

    // Publish at rate in Hz
    ros::Rate loop_rate(30);

    // ====== MAIN LOOP =======
    while (ros::ok())
    {

        // If odometry information is not yet recieved, do not continue
        /* if (!robot_pose_updated)
        {
            ROS_INFO_STREAM("Waiting for odometry information on odom topic : " << odom_topic);
            ros::spinOnce();
            continue;
        }*/

        // ==== TRAJECTORY LIST CREATION ====
        int num_trajectories = 10;
        int num_points_per_trajectory = 10;
        std::vector<std::vector<Pose2D>> traj_list;
        for (int i = 0; i < num_trajectories; i++)
        {
            std::vector<Pose2D> point_list;

            // Fill point list with point objects
            for (int j = 0; j < num_trajectories; j++)
            {
                // Initialize point at 0, 0, 0
                Pose2D point;
                point.x = 0;
                point.y = 0;
                point.theta = 0;

                // Add point to list
                point_list.push_back(point);
            }

            // Add point list (trajectory) to list of trajectories
            traj_list.push_back(point_list);
        }

        // ====== RVIZ MARKERS INITIALIZATION =========
        std::string markers_frame_id = "map";
        std::string markers_namespace = "traj";
        TrajectoryMarkers markers = initializeTrajectoryMarkers(num_trajectories, markers_frame_id, markers_namespace);
        visualization_msgs::Marker spheres_list = markers.points;
        std::vector<visualization_msgs::Marker> line_strip_list = markers.lines;

        // ===== TRAJECTORY PARAMETERS =====
        int max_distance = 1;
        float omega;
        float v = 1;
        float dt = 0.5;

        // Omega (angular speed) configuration
        // Generate omegas for each trajectory : divide omega space into n = num_trajectory parts
        float omega_max = 1;
        float omega_min = -1 * omega_max;
        float omega_range = omega_max - omega_min;
        float omega_increment = omega_range / (num_trajectories - 1);

        // ===== TRAJECTORY GENERATION =======

        // Prepare a point object to update RVIZ markers
        geometry_msgs::Point p;
        p.z = 0;

        ROS_INFO_STREAM("----------- BEGIN MASSACRE ------------- ");
        // Foreach trajectory...
        for (int i = 0; i < num_trajectories; i++)
        {
            omega = (omega_range / 2 - omega_range) + omega_increment * i;
            ROS_INFO_STREAM("Omega : " << omega);

            // Foreach point...
            for (int j = 0; j < num_points_per_trajectory; j++)
            {
                if (j != num_points_per_trajectory - 1)
                {
                    // Update the point's coordinates using the previous point and the equation of the robot's cinematic model
                    traj_list[i][j + 1].x = traj_list[i][j].x - v * dt * sin(traj_list[i][j].theta + currentRobotPose.theta + (dt * omega) / 2);
                    traj_list[i][j + 1].y = traj_list[i][j].y + v * dt * cos(traj_list[i][j].theta + currentRobotPose.theta + (dt * omega) / 2);
                    traj_list[i][j + 1].theta = traj_list[i][j].theta + dt * omega;
                }

                // ==== Update RVIZ markers ====
                p.x = traj_list[i][j].x;
                p.y = traj_list[i][j].y;

                // Update points marker
                spheres_list.points.push_back(p);

                // Draw lines between points
                line_strip_list.at(i).points.push_back(p);

                ROS_INFO_STREAM("x : " << traj_list[i][j].x << " y : " << traj_list[i][j].y << " theta : " << traj_list[i][j].theta);
            }
        }
        // Publish marker information into RVIZ
        marker_pub.publish(spheres_list);
        for (visualization_msgs::Marker line_strip : line_strip_list)
        {
            marker_pub.publish(line_strip);
        }

        // Sleep to update with the frequency rate
        loop_rate.sleep();

        // Listen to upcoming messages
        ros::spinOnce();
    }
}
