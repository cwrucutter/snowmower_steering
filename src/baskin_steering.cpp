#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <baskin_steering/CmdPose.h>
#include <baskin_steering/VelCmd.h>
#include <baskin_steering/Sensor.h>

//steering shell

baskin_steering::CmdPose start_pose;
baskin_steering::CmdPose end_pose;
geometry_msgs::Pose robot_pose;
bool new_route = false;
bool new_obstacles = false;

double d_distance;
double d_angle;
double k1; //linear adjustment coefficient
double k2; //angular adjustment coefficient
double max_v = 5;
double max_w = 2;

geometry_msgs::Twist control_output; // linear.x = forward, angular.z = turn
std_msgs::String feedback;


	
//pre-declare functions
void startPoseCB(const baskin_steering::CmdPose& cmd);
void endPoseCB(const baskin_steering::CmdPose& cmd);
void robotPoseCB(const geometry_msgs::Pose& message_pose);
/* Calculation Structure
update(): call this function
 + generate_potential-map: update [component] functions
    + update_obstacles: update any obstacle locations
    + update_waypoints: revise start/end pose influence if needed
 + evaluate_point || evaluate_map: calculate potentials for one (point) or more (map)
    + get angles from 4 functions (depart, arrive, motive, obstacle) for a point (x,y,theta)
    + add them with linear weights, and then sum vectors
    + extract vector direction
    + outputs the vector direction as a delta from the robot's current direction
 + calculate distance to / return speed
 + convert_to_twist: convert the output of the algorithm to Twist-speak
*/
void update();
	void generate_potential_map();
		void update_obstacles();
		void update_waypoints();
	double evaluate_point(double x, double y, double theta);
		double depart_component(double x, double y, double theta); 
		double arrive_component(double x, double y, double theta); 
		double motive_component(double x, double y, double theta);
		double obstacle_component(double x, double y, double theta);
	double dist_to_goal(double x, double y, double theta);
	void convert_to_twist();
//end pre-declare functions

void startPoseCB(const baskin_steering::CmdPose& cmd) { 
	start_pose.pose = cmd.pose;
	start_pose.v_arrive = cmd.v_arrive;
	start_pose.v_depart = cmd.v_depart;
	new_route = true;
}

void endPoseCB(const baskin_steering::CmdPose& cmd) {
	end_pose.pose = cmd.pose;
	end_pose.v_arrive = cmd.v_arrive;
	end_pose.v_depart = cmd.v_depart; 
	new_route = true;
}

void robotPoseCB(const geometry_msgs::Pose& message_pose) {
	robot_pose.position = message_pose.position;
	robot_pose.orientation = message_pose.orientation;

}

void update() {
	//main work function for the steering
	d_distance = 3-d_distance;
	d_angle = 3-d_angle;
	generate_potential_map();
	d_angle = evaluate_point(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.z);
	d_distance = dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.z);
	//converts work function output to message format
	convert_to_twist();

}
void generate_potential_map() {
	update_obstacles();
	
}
void update_obstacles() {
	//TODO 1
	//need to add an obstacle message, callback, etc.
}
void update_waypoints() {
	//TODO 1
	// this and update obstacles maybe unnecessary or shifted (maybe moved to callbacks)
}
double evaluate_point(double x, double y, double theta) {
	double dep = depart_component(x,y,theta);
	double arr = depart_component(x,y,theta);
	double mov = depart_component(x,y,theta);
	double obs = depart_component(x,y,theta);

	double dx = 3*cos(obs) + 1*cos(dep) + 1*cos(arr) + 1*cos(mov);
	double dy = 3*sin(obs) + 1*sin(dep) + 1*sin(arr) + 1*sin(mov);
	double dth = atan2(dy, dx);
	return dth - robot_pose.orientation.z;
}
double depart_component(double x, double y, double theta) {
	//TODO 0
	//dist from line source
	return 0.0;
}
double arrive_component(double x, double y, double theta) {
	//TODO 0
	//dist from line sink
	return 0.0;
} 
double motive_component(double x, double y, double theta) {
	//TODO 0
	//radial sink
	return 0.0;
}
double obstacle_component(double x, double y, double theta) {
	//TODO 0
	//multiple radial source
	return 0.0;
}
double dist_to_goal(double x, double y, double theta) {
	//TODO 0
	// distance based
	return 0.0;
}
void convert_to_twist() {
	//takes controller input and makes it into a Twist

	//2d non-holonomic constraints
	control_output.linear.y = 0;
	control_output.linear.z = 0;
	control_output.angular.x = 0;
	control_output.angular.y = 0;

	//commands
	control_output.linear.x = std::max(-max_v, std::min(max_v,k1*d_distance));
	control_output.angular.z = std::max(-max_w, std::min(max_w, k2*d_angle));
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"baskin_steering");
	ros::NodeHandle n;
	ros::Rate timer(10);
	///*

	ros::Publisher output_pub = n.advertise<geometry_msgs::Twist>("/steering/out",1);
	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("/steering/feedback",1);

	ros::Subscriber sub_start = n.subscribe ("/steering/start_pose", 1, startPoseCB);
	ros::Subscriber sub_end = n.subscribe ("/steering/end_pose", 1, endPoseCB);
	ros::Subscriber sub_robot = n.subscribe ("/steering/robot_pose", 1, robotPoseCB);

	
	d_distance = 1.0;
	d_angle = 2.0;
	feedback.data = "repeat";
	
	while(ros::ok())
	{
		//publish
		//do something with control_output
		update();
		output_pub.publish(control_output);
		// request refresh from planner
		feedback_pub.publish(feedback);

		//subscribe
		ros::spinOnce();
		timer.sleep();
	}
	//*/
	return 0;
}
