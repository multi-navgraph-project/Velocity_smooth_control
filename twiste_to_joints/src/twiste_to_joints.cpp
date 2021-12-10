#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <cmath>

ros::Subscriber cmd_sub;
ros::Subscriber joints_sub;
ros::Publisher joint_steer_pub;
ros::Publisher joint_drive_pub;
ros::Publisher odom_pub;
nav_msgs::Odometry odom_msg;
std_msgs::Float64 joints_drive_msg;
std_msgs::Float64 joints_steer_msg;
double omega_fb, v_fb;
const double wheel_radius = 0.12;
const double wheel_base = 1.7;
const double min_turning_radius = 0.3;

double computeWheelMotion(double v, double omega)
{
  if (omega == 0 || v == 0)
  {
    return 0;
  }
  else 
  {
    
    double radius = v / omega;
    //  if (fabs(radius) < min_turning_radius)
    //  {
    //    radius = std::copysign(min_turning_radius,radius );
    //  }
   return std::atan(wheel_base/radius);
  }
  
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  joints_steer_msg.data = computeWheelMotion(msg->linear.x, msg->angular.z);
  //ROS_INFO("PHI : %f",joints_steer_msg.data*(180/M_PI));
  //steer direct

  //joints_steer_msg.data = msg->angular.z;

  //joints_drive_msg.data = std::copysign((std::sqrt(pow(msg->linear.x,2) + pow(msg->angular.z*wheel_base,2))) / wheel_radius, msg->linear.x);
  //drive 
   joints_drive_msg.data = msg->linear.x/0.12;

  // if (fabs(joints_steer_msg.data) > 1.3)
  //  {
  //    joints_drive_msg.data = std::copysign(0.3/0.12, msg->linear.x);
  //  }
  // else
  // {
  //   joints_drive_msg.data = msg->linear.x/0.12;
  // }

  //joints_drive_msg.data = msg->linear.x/wheel_radius;

  joint_drive_pub.publish(joints_drive_msg);
  joint_steer_pub.publish(joints_steer_msg);
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
   omega_fb = ((msg->velocity[1]*wheel_radius)/ wheel_base)* std::sin(msg->position[0]);
   v_fb = (msg->velocity[1]* wheel_radius)* std::cos(msg->position[0]);


  odom_msg.twist.twist.linear.x = v_fb;
  odom_msg.twist.twist.angular.z = omega_fb;

  odom_pub.publish(odom_msg);

}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "twiste_to_joints");
  ros::NodeHandle nh;
  std::string vel_topic;
  nh.param<std::string>("twist_topic", vel_topic, "/cmd_vel");
  cmd_sub = nh.subscribe(vel_topic, 10, cmdVelCallback);
  joints_sub = nh.subscribe("forklift/joint_states", 10, jointStateCallback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_feedback", 10);
  joint_drive_pub = nh.advertise<std_msgs::Float64>("/forklift/drive_velocity_controller/command", 10);
  joint_steer_pub = nh.advertise<std_msgs::Float64>("/forklift/steering_position_controller/command", 10);

  ros::spin();

  return 0;
}
