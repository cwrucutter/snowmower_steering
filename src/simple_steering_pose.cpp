#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <math.h>

geometry_msgs::Pose g_goalPose;
geometry_msgs::Pose g_robotPose;
int g_path_iteration = 0;
bool g_isPath = false;

void goalPose_CB(const nav_msgs::Path& msg){
  //Write published PoseStamped to the global variable
  g_goalPose = msg.poses[g_path_iteration].pose;
  g_isPath = true;
}

void robotPose_CB(const nav_msgs::Odometry& msg){
  //Write published PoseStamped to the global variable
  g_robotPose = msg.pose.pose;
}

int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "simple_steering_pose");
  ros::NodeHandle nh;

  //This is the advertiser object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);

  //Create a subscriber object that gets the global path message
  ros::Subscriber global_path_sub = nh.subscribe("/path",1,goalPose_CB);

  //Create a subscriber object that gets the global path message
  ros::Subscriber robot_odometry_sub = nh.subscribe("/robot0/odom",1,robotPose_CB);

  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  ros::Rate rate(10.0);

  //x and y position of g_goalPose in robot frame
  std_msgs::Float64 x;
  std_msgs::Float64 y;

  //theta_r is the angle of the robot in the map frame
  std_msgs::Float64 theta_r;

  double d_thres = .1;

  while (nh.ok()){
    //Check for published message (update g_robotPose and g_goalPose)
    ros::spinOnce();
    theta_r.data = g_robotPose.orientation.w; //Not sure if this is correct

    x.data = (g_goalPose.position.x-g_robotPose.position.x)*cos(theta_r.data) + (g_goalPose.position.y-g_robotPose.position.y)*sin(theta_r.data);

    y.data = -(g_goalPose.position.x-g_robotPose.position.x)*sin(theta_r.data) + (g_goalPose.position.y-g_robotPose.position.y)*cos(theta_r.data);

    //Simple algorithm to steer robot to a point (not orientation).
    vel.angular.z = 4.0 * atan2(y.data,x.data);
    vel.linear.x = 0.5 * sqrt(pow(x.data,2)+pow(y.data,2));

    //Publish velocity command.
    vel_pub.publish(vel);
    //If we are close enough to the goal, pick the next goal the next time around.
    if ((sqrt(pow(x.data,2)+pow(y.data,2)) < d_thres)||(g_isPath)) {
      g_path_iteration++;
    }
    //And sleep for the rest of the loop cycle.
    rate.sleep();
  }
  return 0;
}
