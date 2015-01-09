#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <snowmower_steering/CmdPose.h>
#include <snowmower_steering/Obstacle.h>

//steering shell

snowmower_steering::CmdPose start_pose;
snowmower_steering::CmdPose end_pose;
geometry_msgs::Pose robot_pose;
std::vector< snowmower_steering::Obstacle > obstacles;
bool new_route = false;
bool new_obstacles = false;
bool send_feedback = true;

bool debug_mode = false;
bool debug_speed = true;

double d_distance;
double d_angle;
double k1; //linear adjustment coefficient
double k2; //angular adjustment coefficient

double k_stable; //depart/arrive strength

double max_v = 5;
double max_w = 2;

geometry_msgs::Twist control_output; // linear.x = forward, angular.z = turn
std_msgs::String feedback;

//helper functions
double dist(double x1, double y1, double x2, double y2) { return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)); }
double dist_from_line(double x, double y, double xt0, double yt0, double xt, double yt) {
	/*
		dist of point (x,y)
		from line : x(t) = t*xt + xt0
					y(t) = t*yt + yt0
		**Note: negative to left of line, positive to right
	*/
	double x1 = x-xt0;
	double y1 = y-yt0;
	return (x1*yt+y1*xt)/sqrt(xt*xt+yt*yt);
}
// end helper functions

	
//pre-declare functions
void startPoseCB(const snowmower_steering::CmdPose& cmd);
void endPoseCB(const snowmower_steering::CmdPose& cmd);
void robotPoseCB(const geometry_msgs::Pose& message_pose);
void obstacleCB(const snowmower_steering::Obstacle& cmd);
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
// end predeclared functions

void startPoseCB(const snowmower_steering::CmdPose& cmd) {
	std::stringstream ss;
	ss << "[" << cmd.pose.position.x << "," << cmd.pose.position.y << "," << cmd.pose.orientation.w << "]";
	ROS_INFO("Recieved new start pose %s", ss.str().c_str());
	start_pose.pose = cmd.pose;
	start_pose.v_arrive = cmd.v_arrive;
	start_pose.v_depart = cmd.v_depart;
	new_route = true;
}

void endPoseCB(const snowmower_steering::CmdPose& cmd) {
	std::stringstream ss;
	ss << "[" << cmd.pose.position.x << "," << cmd.pose.position.y << "," << cmd.pose.orientation.w << "]";
	ROS_INFO("Recieved new end pose %s", ss.str().c_str());
	end_pose.pose = cmd.pose;
	end_pose.v_arrive = cmd.v_arrive;
	end_pose.v_depart = cmd.v_depart; 
	new_route = true;
}

void robotPoseCB(const geometry_msgs::Pose& message_pose) {
	ROS_INFO("Recieved new robot pose");
	robot_pose.position = message_pose.position;
	robot_pose.orientation = message_pose.orientation;

}
///*
void obstacleCB(const snowmower_steering::Obstacle& cmd) {
	if (cmd.type == "add") {
		obstacles.push_back(cmd);
	}
	else if (cmd.type == "remove") { //removes all obstacles of a given name
		for (int i = 0; i < obstacles.size(); i++) {
			if (cmd.name == obstacles[i].name) { 
				obstacles.erase(obstacles.begin()+i); 
			}
		}
	}
	else //updates all obstacles of a given name
	{
		for (int i = 0; i < obstacles.size(); i++) {
			if (cmd.name == obstacles[i].name) { 
				obstacles[i].type = cmd.type;
				obstacles[i].radius = cmd.radius;
				obstacles[i].x = cmd.x;
				obstacles[i].y = cmd.y;
			}
		}
	}
}
//*/
void update() {
	if (debug_mode) { ROS_INFO("updating"); }
	if (dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w < .05)) {
		//at goal point w/in 5 cm
		ROS_INFO("near goal location...");
		if (abs(robot_pose.orientation.w - end_pose.pose.orientation.w) < .05) {
			//at goal orientation w/in 3 degrees
			ROS_INFO("...near goal pose");
			send_feedback = true;
			feedback.data = "next";
			d_angle = 0.0;
			d_distance = 0.0;
		}
		else {
			ROS_INFO(".rotating to goal pose");
			send_feedback = true;
			feedback.data = "repeat";
			d_angle = end_pose.pose.orientation.w - robot_pose.orientation.w;
			d_distance = dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w);
		}
	}
	else {	
	//main work function for the steering
	if (debug_mode) { ROS_INFO("following potential function.."); }
	generate_potential_map();
	d_angle = evaluate_point(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w);
	d_distance = dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w);

	}

	//converts work function output to message format
	convert_to_twist();
}
void generate_potential_map() {
	update_obstacles();
	update_waypoints();
	
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
	double arr = arrive_component(x,y,theta);
	double mov = motive_component(x,y,theta);
	double obs = obstacle_component(x,y,theta);
	double dx = 0.0;
	double dy = 0.0;	
	//TODO **Note: soften obstacle edge
	if (obs == -77.7) {
		dx = 1*cos(dep) + 1*cos(arr) + 1*cos(mov);
		dy = 1*sin(dep) + 1*sin(arr) + 1*sin(mov);
	}
	else {
		dx = 3*cos(obs) + 1*cos(dep) + 1*cos(arr) + 1*cos(mov);
		dy = 3*sin(obs) + 1*sin(dep) + 1*sin(arr) + 1*sin(mov);
	}
	double dth = atan2(dy, dx); //desired theta angle
	return dth - robot_pose.orientation.w;
}
double depart_component(double x, double y, double theta) {
	//dist from line source
	double angle_offset = atan(k_stable*dist_from_line(x,y, start_pose.pose.position.x, start_pose.pose.position.y, cos(start_pose.pose.orientation.w), sin(start_pose.pose.orientation.w)));

	return start_pose.pose.orientation.w - angle_offset;
}
double arrive_component(double x, double y, double theta) {
	//dist from line sink
	double angle_offset = atan(k_stable*dist_from_line(x,y, 
		end_pose.pose.position.x, end_pose.pose.position.y, 
		cos(end_pose.pose.orientation.w), sin(end_pose.pose.orientation.w)));

	return end_pose.pose.orientation.w + angle_offset;
} 
double motive_component(double x, double y, double theta) {
	//single radial sink
	double dy = end_pose.pose.position.y - end_pose.pose.position.y;
	double dx = end_pose.pose.position.x - end_pose.pose.position.x;
	return atan2(dy,dx);
}
double obstacle_component(double x, double y, double theta) {
	//multiple radial source
	/*
		sums the vectors for nearby objects to establish the obstacle avoidance vector
	*/
	double dx = 0.0;
	double dy = 0.0;
	
	for (int i = 0; i < obstacles.size(); i++) {
		// if it is within the obstacle radius, "push" away
		if (dist(x,y,obstacles[i].x,obstacles[i].y) < obstacles[i].radius) {
			double pth = atan2(y-obstacles[i].y, x-obstacles[i].x); //push theta
			dx = dx + cos(pth);
			dy = dy + sin(pth);
		}
	}
	// error or no obstacle effect
	if (dx == 0.0 && dy == 0.0) {
		if (debug_mode) { ROS_INFO("No obstacles."); }
		return -77.7;
	}
	else {
		if (debug_mode) { ROS_INFO("Obstacles nearby"); }
		return atan2(dy, dx);
	}
}
double dist_to_goal(double x, double y, double theta) {
	return dist(x, y, end_pose.pose.position.x, end_pose.pose.position.y);
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
	std::stringstream ss;
	ss << "--> Twist Out: v: "<< control_output.linear.x <<" w: " << control_output.angular.z << " ";
	ROS_INFO("updating to new commands %s", ss.str().c_str());
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"baskin_steering");
	ros::NodeHandle n;
	ros::Rate timer(10);
	if (debug_speed) {timer = ros::Rate(1); }
	ROS_INFO("ROS init for baskin_steering");

	///*

	ros::Publisher output_pub = n.advertise<geometry_msgs::Twist>("/steering/out",1);
	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("/steering/feedback",1);

	ros::Subscriber sub_start = n.subscribe ("/steering/start_pose", 1, startPoseCB);
	ros::Subscriber sub_end = n.subscribe ("/steering/end_pose", 1, endPoseCB);
	ros::Subscriber sub_robot = n.subscribe ("/steering/robot_pose", 1, robotPoseCB);
	ros::Subscriber sub_obstacle = n.subscribe ("/steering/obstacle", 1, obstacleCB);

	ros::spinOnce();
	d_distance = 1.0;
	d_angle = 2.0;
	send_feedback = true;
	feedback.data = "restart";
	feedback_pub.publish(feedback);
	feedback_pub.publish(feedback);
	feedback_pub.publish(feedback);
	
	while(ros::ok())
	{
		ros::spinOnce();
		//publish
		//do something with control_output
		send_feedback = false;
		update();
		if (debug_mode) { ROS_INFO("Publishing control commands for robot"); }
		output_pub.publish(control_output);
		// request refresh from planner
		if(send_feedback || true) { feedback_pub.publish(feedback); }

		//subscribe
		ros::spinOnce();
		timer.sleep();
	}
	//*/
	return 0;
}
