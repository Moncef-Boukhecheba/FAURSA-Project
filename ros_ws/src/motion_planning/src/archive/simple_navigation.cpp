#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h> 
#include <tf/tf.h>
# define M_PI           3.14159265358979323846

/**
 * @brief Short class used to store the 2D position of the robot
 */
class Pose{
  public :
    float x;
    float y;
    float theta;

    Pose(float x, float y, float theta){
      this->x = x;
      this->y = y;
      this->theta = theta;
    }
};

// Robot current position
Pose currentRobotPose(0, 0, 0);

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
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
    
  currentRobotPose.theta = yaw;
}

/**
 * @brief 
 * 
 * @param axis_theta the angle to which the robot should correct to
 * @param currentRobotPose current pose of the robot
 * @param angular_speed angular speeed
 * @param msg twist message
 */
void correctSteering(float axis_theta, Pose currentRobotPose, geometry_msgs::Twist& msg, 
                      float angular_speed, float linear_speed){
  short direction = 1; 
  float diff_rad = currentRobotPose.theta - axis_theta;
  
  // If both the current theta and the axis theta are both negative or positive, 
  // the direction of turning is equal to the sign of the difference between the two values
  if (abs(currentRobotPose.theta) + abs(axis_theta) == currentRobotPose.theta + axis_theta){
    direction = diff_rad / abs(diff_rad);
  } 
  // If one value is positive and the other is negative, 
  // compare the length counterclockwise and clockwise and choose the best direction
  else {
    float direct_length = diff_rad;
    float alternative_path = M_PI - abs(currentRobotPose.theta) + M_PI - abs(axis_theta);

    if (direct_length < alternative_path){
      direction = direct_length / abs(direct_length);
    } else {
      direction = (direct_length / abs(direct_length)) * -1;
    }
  }

  msg.angular.z = direction * angular_speed;

  /* 
  // Adjust linear velocity relative to angle sharpness
  float new_linear_speed = ((diff_rad * -1 * linear_speed) / M_PI) + linear_speed;
  if (new_linear_speed > 0){
    // msg.linear.x = new_linear_speed;
    msg.linear.x = 0;
  } else {
    msg.linear.x = 0;
  } */
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  // Initialization
  ros::init(argc, argv, "simple_navigation");
  ros::NodeHandle n_pub;
  ros::NodeHandle n_sub;
  ros::Rate loop_rate(50);
  float min_distance_sides = 0.5;
  float min_distance = 0.5;
  float orientation_margin = 0.0002;

  // Get the goal position and put it in a Pose object
  float x = 0, y = 0;
  ros::NodeHandle nparams("~");
  nparams.getParam("x", x);
  nparams.getParam("y", y);
  ROS_INFO("%f %f", x, y);
  Pose goal(x, y, 0);

  // Prepare to publish cmd velocities
  ros::Publisher pub = n_pub.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Listen for current position through odometry
  ros::Subscriber sub = n_sub.subscribe("/odom", 1000, odomCallback);

  // Primary loop
  while(ros::ok()){
    float diff_x = goal.x - currentRobotPose.x;
    float diff_y = goal.y - currentRobotPose.y;
    float distance_from_goal = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    float angular_speed = 2;
    float linear_speed = 2;

    // Instantiate Twist message
    geometry_msgs::Twist msg;    

    // If we didn't reach the end (min eucledean distance not reached)
    if (distance_from_goal > min_distance){
      // We start moving on the X axis 
      if (abs(diff_x) > min_distance_sides){
        // Align the car to the axis 
        float orientation_cos = diff_x / abs(diff_x);
        float axis_theta = acos(orientation_cos);

        // TODO : REMOVE
        ROS_INFO("%s %f %f %f %f", "MOVING ON X", diff_x, orientation_cos, cos(currentRobotPose.theta), currentRobotPose.theta);

        // If car is not properly oriented on the axis, correct the rotation to line up with the axis          
        if (abs(cos(currentRobotPose.theta) - orientation_cos) > orientation_margin){
          correctSteering(axis_theta, currentRobotPose, msg, angular_speed, linear_speed);
        } else {
            // Go forward
            msg.linear.x = linear_speed;
        }      
      }

      // We move on the Y axis
      else if (abs(diff_y) > min_distance_sides){
        // Align the car to the axis 
        float orientation_sin = diff_y / abs(diff_y);
        float axis_theta = asin(orientation_sin);

        ROS_INFO("%s %f %f %f %f", "MOVING ON Y", diff_y, orientation_sin, sin(currentRobotPose.theta), currentRobotPose.theta);

        // If car is not properly oriented on the axis
        if (abs(sin(currentRobotPose.theta) - orientation_sin) > orientation_margin){
          // Correct the rotation to line up with the axis          
          correctSteering(axis_theta, currentRobotPose, msg, angular_speed, linear_speed);
        } else {
            // Go forward
            msg.linear.x = linear_speed;
        }
      }

      else {
        min_distance_sides = min_distance_sides / 2;
      }

      pub.publish(msg);
      ROS_INFO("%f %f", msg.linear.x, msg.angular.z);
    } 
    // Arrived to destination
    else {
      ROS_INFO("%s %f", "ARRIVED TO DESTINATION", distance_from_goal);
    }
    
    // Sleep and spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}