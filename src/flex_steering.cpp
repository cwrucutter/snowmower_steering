#include <ros/ros.h>
//input
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
//output
#include <geometry_msgs/Twist.h>

/*

Author : William Baskin
Created: 1430-12-04-14
Edited : 1730-12-04-14

TODO:
	+ generate_header()
	+ default_path()
	+ [non-default] get_path()
	+ current_robot_pose()
	+ x_deriv()
	+ y_deriv()
	+ [alternate drive scheme[s]?]

*/

double max_vel = 10.0;
double max_ang_vel = 2.0;

ros::NodeHandle setup(String name) {
	ros::init(argc, argv, name);
	ros::NodeHandle nh;
	return nh;
}
/*
//automatic generation functions
*/
std_msgs::Header generate_header() {
	std_msgs::Header h;
	//TODO
	return h;
}
geometry_msgs::Point fill_point(double x, double y, double z) {
	geometry_msgs::Point to_return;
	to_return.x = x;
	to_return.y = y;
	to_return.z = z;

	return to_return;
}
geometry_msgs::Quaternion fill_quaternion(double w, double x, double y, double z) {
	geometry_msgs::Quaternion to_return;
	to_return.x = x;
	to_return.y = y;
	to_return.z = z;
	to_return.w = w;

	return to_return;
}
std::vector< geometry_msgs::PoseStamped > default_path() {
	//TODO
	std::vector< geometry_msgs::PoseStamped > to_return;
	//assign a path
	geometry_msgs::PoseStamped pose0;
	pose0.header = generate_header();
	pose0.point = fill_point(0.0 , 0.0 , 0.0);
	pose0.orientation = fill_quaternion(0.0 , 0.0 , 0.0 , 0.0);

	geometry_msgs::PoseStamped pose1;
	pose1.header = generate_header();
	pose1.point = fill_point(0.0 , 0.0 , 0.0);
	pose1.orientation = fill_quaternion(0.0 , 0.0 , 0.0 , 0.0);

	geometry_msgs::PoseStamped pose2;
	pose2.header = generate_header();
	pose2.point = fill_point(0.0 , 0.0 , 0.0);
	pose2.orientation = fill_quaternion(0.0 , 0.0 , 0.0 , 0.0);

	return to_return();
}

std::vector< geometry_msgs::PoseStamped > get_path() {
	//currently generates a default path. Can be replaced by loading from a yaml, getting info from node, etc.
	std::vector< geometry_msgs::PoseStamped > to_return = default_path();
	//TODO
	return to_return();

}

geometry_msgs::Twist setup_command() {
	geometry_msgs::Twist vel;
	vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;
	return vel;
}
geometry_msgs::PoseStamped current_robot_pose() {
	//TODO
	return geometry_msgs::PoseStamped crp();
}

/*
 motion selection functions
*/
double x_deriv(geometry_msgs::PoseStamped last_goal, geometry_msgs::PoseStamped next_goal, geometry_msgs::PoseStamped next_next_goal, geometry_msgs::PoseStamped robot_current) {
	//TODO
	return 0.0;
}
double y_deriv(geometry_msgs::PoseStamped last_goal, geometry_msgs::PoseStamped next_goal, geometry_msgs::PoseStamped next_next_goal, geometry_msgs::PoseStamped robot_current) {
	//TODO
	return 0.0;
}

double dx;
double dy;
double dTheta;
double dr;
//these are messages of pose all in the same frame.
geometry_msgs::Twist update_command(geometry_msgs::Twist old_command, geometry_msgs::PoseStamped last_goal, geometry_msgs::PoseStamped next_goal, geometry_msgs::PoseStamped next_next_goal, geometry_msgs::PoseStamped robot_current) {	
	//desired change in position for next move
	dx = x_deriv(last_goal, next_goal, next_next_goal, robot_current);
	dy = y_deriv(last_goal, next_goal, next_next_goal, robot_current);
	dTheta = atan2(dx,dy);
	dr = sqrt(pow(dx,2)+pow(dy,2));

	old_command.angular.z = 4 * dTheta;

	if(dTheta > 1.5) { //if the necessary turn is huge, then stop and turn for a bit to realign and resume
	old_command.linear.x = 0; 
	}
	else {
	old_command.linear.x = 0.5 * dr;
	}
}

//***** BEGIN MAIN METHOD *****\\

int main(int argc, char** argv) {
	//generic setup
	ros::NodeHandle n = setup();
	ros::Rate rate(10.0);

	//setup path
	std::vector< geometry_msgs::PoseStamped > path = get_path();

	//setup outgoing message
	geometry_msgs::Twist twist_command = setup_command();

	//useful variables
	geometry_msgs::PoseStamped g0;
	geometry_msgs::PoseStamped g1;
	geometry_msgs::PoseStamped g2;
	geometry_msgs::PoseStamped robot_pose;

	int i = 0;
	while(ros::ok()) {
		for(i = 0; i+2 < path.size() && ros::ok() ; i++) {
		//loops while ros::ok(), there are path points to process
		g0 = path[i];
		g1 = path[i+1];
		g2 = path[i+2];
		
		twist_command = update_command(twist_command, g0,g1,g2,robot_pose);
		
		}
	}
	return 0;
}
