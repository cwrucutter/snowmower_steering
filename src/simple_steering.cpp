#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_steering");
  ros::NodeHandle nh;

  pub_vel = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1); //This is the advertiser object that will publish the desired forward and angular velocity of the robot.

  tf::TransformListener listener;

  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  ros::Rate rate(10.0);

  while (nh.ok(){
      tf::StampedTransform transform;
      try{
	listener.lookupTransform("/robot0","/map",ros::Time(0),transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
      }

      vel.angular.z = 4.0 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
      vel.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));

      pub_vel.publish(vel);
      rate.sleep();
    }
    return 0;
};
