//----------------------------------------------------------------
//             program example : read laser data, publich cmd
//-----------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <vector>
#include <math.h>
#include <iostream>

#define PI 3.14159265

using namespace std;

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
        resolution = scan->angle_increment * -1;
        num_readings = (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
        ranges_scan = std::vector<double>(num_readings);
    }

    // Copy scan values into the "range_scan" array
    for (int i = 0; i < num_readings; ++i)
    {
        ranges_scan[i] = scan->ranges[i];
    }

    ROS_INFO("laser Range:  [%ld]", ranges_scan.size());
}

int main(int argc, char **argv)
{
    // Initialize variables
    double begin_timer, end_timer;
    float Lc = 0.35, Ly = 0.7;
    float X, Y, linear_vel, angular_vel, TIME_LOOP;
    string scan_topic = "/scan";

    ros::init(argc, argv, "LaserRead");
    ros::NodeHandle n;

    // Handle for parameters
    ros::NodeHandle nh("~");
    nh.getParam("scan_topic", scan_topic);
    ROS_INFO_STREAM("Waiting on scan data on topic : " << scan_topic);

    // Publisher of command velocity
    ros::Publisher Velocity_pub = n.advertise<geometry_msgs::Twist>("/ARDUINO1/Velocity", 1);

    // Subscribe for laser data and handle everything in the laserCallback function
    ros::Subscriber sub_laser = n.subscribe(scan_topic, 1, laserCallback);

    // ros::Rate loop_rate_1(80);

    // Frequency of publishing = 40Hz
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        // Start a timer (this is only to measure the performance of the code, it is optional)
        begin_timer = ros::Time::now().toSec();

        // Create an empty geometry twist message (it tells the robot to move with a certain linear speed and angular speed)
        geometry_msgs::Twist msg;

        /* ==== UNCOMMENT THIS PART IF YOU WANT THE ROBOT TO MOVE (forward, forever) =====
            // Configure linear and angular speed so that the robot moves forward
            linear_vel = 1;
            angular_vel = 0;

            // Update twist message
            msg.linear.x = linear_vel;
            msg.angular.z = angular_vel;

            // Publish twist message
            Velocity_pub.publish(msg);
            ROS_INFO_STREAM("velocity command:"
                            << " linear=" << msg.linear.x << " angular=" << msg.angular.z);
        */

        // For each reading of the scan ranges...
        for (int ii = 0; ii < num_readings; ++ii)
        {
            /* ===== UNCOMMENT TO PRINT COORDINATES =====
            // Calculate the x and y point
            X = cos(ii * resolution) * ranges_scan[ii] + Lc;
            Y = sin(ii * resolution) * ranges_scan[ii] + Ly;

            // Print the coordinates to the terminal
            ROS_INFO_STREAM("Point coordinates:"
                            << "x = " << X << " y = " << Y);

            // End the timer and print the time of the current loop
            end_timer = ros::Time::now().toSec();
            TIME_LOOP = end_timer - begin_timer;
            // --- Uncomment to print ----
            // cout << "diff: " << TIME_LOOP << endl;
            */

            ROS_INFO_STREAM("Angle : " << ii * resolution << " distance = " << ranges_scan[ii]);
        }

        // ==== ROS Boiler plate code =====
        // Necessary for subscribers to keep listening
        ros::spinOnce();

        // Sleep at the frequency rate
        loop_rate.sleep();
    }

    ros::spin();
}
