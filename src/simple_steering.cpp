#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_steering");
  ros::NodeHandle nh;

  pub_vel = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1); //This is the advertiser object that will publish the desired forward and angular velocity of the robot.

  tf::TransformListener(nh) tf;

  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;
