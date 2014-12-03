#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "simple_steering");
  ros::NodeHandle nh;

  //This is the advertiser object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);

  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  //Create a TransformListener Object
  tf::TransformListener listener;

  //Create a Stamped Transform Object. This will be the transformation between the current robot pose and the goal pose.
  tf::StampedTransform transform;

  ros::Rate rate(10.0);

  while (nh.ok()){
      try{
        //Listen for the most recent transform (should use listener.waitForTransform)
	listener.lookupTransform("/robot0","/goalPose",ros::Time(0),transform);
      }
      catch (tf::TransformException ex){
	//If there is an exception, print error and sleep a second. This is dangerous. The velocity command should reset to zero.
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
      }

      //Simple algorithm to steer robot to a point (not orientation).
      vel.angular.z = 4.0 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
      vel.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));

      //Publish velocity command.
      vel_pub.publish(vel);
      //And sleep for the rest of the loop cycle.
      rate.sleep();
    }
  return 0;
}
