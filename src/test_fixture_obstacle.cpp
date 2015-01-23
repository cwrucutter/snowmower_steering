#include <ros/ros.h>
#include <math.h>
#include <snowmower_steering/Obstacle.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
/*
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <snowmower_steering/CmdPose.h>
*/
double obs_x;
double obs_y;
snowmower_steering::Obstacle msg;
geometry_msgs::Pose robot_pose;
double robot_heading;

void synthetic_obstacle(double x, double y) {
	obs_x = x;
	obs_y = y;
	msg.name = "Synthetic obstacle 00";
	msg.type = "add";
	msg.size = .80;
	msg.r = 777.777;
	msg.theta = 0.0;
}
void robotPoseCB(const geometry_msgs::Pose& msg) {
	ROS_INFO("Recieved new robot pose");
	robot_pose.position = msg.position;
	robot_pose.orientation = msg.orientation;
	robot_heading = 2.0*acos(msg.orientation.w);
	if (msg.orientation.x < 0.0 || msg.orientation.y < 0.0 || msg.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
}
void robotSimPoseCB(const nav_msgs::Odometry& msg) {
	robot_heading = 2.0*acos(msg.pose.pose.orientation.w);
	if (msg.pose.pose.orientation.x < 0.0 || msg.pose.pose.orientation.y < 0.0 || msg.pose.pose.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
	robot_pose.position = msg.pose.pose.position;
	robot_pose.orientation = msg.pose.pose.orientation;
}
void robotRealPoseCB(const geometry_msgs::PoseWithCovariance& msg) {
	robot_heading = 2.0*acos(msg.pose.orientation.w);
	if (msg.pose.orientation.x < 0.0 || msg.pose.orientation.y < 0.0 || msg.pose.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
	robot_pose.position = msg.pose.position;
	robot_pose.orientation = msg.pose.orientation;
}




/*
	MAIN
 */

int main(int argc, char** argv) {
	ros::init(argc,argv,"test_fixture_obstacle");
	if(argc > 2) {
		synthetic_obstacle(atof(*(argv+1)), atof(*(argv+2)));
	}
	else {
		return 0;
	}
	ros::NodeHandle n;
	ros::Rate timer(20);
	ROS_INFO("ROS init for obstacle testing");
	
	//TODO
	ros::Publisher obstacle_pub = n.advertise< snowmower_steering::Obstacle >("/steering/obstacle", 1);
	ros::Subscriber sub_robot = n.subscribe ("/steering/robot_pose", 1, robotPoseCB);
	ros::Subscriber sub_robot_odom = n.subscribe ("/robot0/odom", 1, robotSimPoseCB);
	ros::Subscriber sub_robot_est = n.subscribe ("/pose_est" , 1, robotRealPoseCB);
	
	while(ros::ok()) {
		ros::spinOnce();
	}
}
