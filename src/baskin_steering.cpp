/* William Baskin */
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <snowmower_steering/CmdPose.h>
#include <snowmower_steering/Obstacle.h>
/* NOTE TO SELF: Check Quaternion implmentation */
//steering shell

snowmower_steering::CmdPose start_pose;
snowmower_steering::CmdPose end_pose;
geometry_msgs::Pose robot_pose;
double robot_heading = 0.0;
ros::Publisher plow_angle_pub;
std_msgs::UInt8 jackie;
std::vector< snowmower_steering::Obstacle > obstacles;

//std::vector< visualization_msgs::Marker > markers;

bool new_route = false;
bool new_obstacles = false;
bool send_feedback = true;

bool start_flag = false;
bool end_flag = false;
bool robot_flag = false;
bool obstacle_flag = false;

bool debug_mode = false;
bool super_debug = true;
bool debug_methods = false;
bool debug_speed = false;

bool simple_mode = false;
bool ignore_obstacles = false;

bool gps_only = false;
bool ignore_heading = false;

double d_distance;
double d_angle;
double k1 = 0.5; //linear adjustment coefficient
double k2 = 1.0; //angular adjustment coefficient

double k_stable = 2.0; //depart/arrive strength

double max_v = 1.0;
double max_w = 1.0;

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
double dist_to_goal(double x, double y, double theta) {
	return dist(x, y, end_pose.pose.position.x, end_pose.pose.position.y);
}
double dist_to_start(double x, double y, double theta) {
	return dist(x, y, start_pose.pose.position.x, start_pose.pose.position.y);
}
double dist_to_obstacle(double x, double y, double scan_x, double scan_y, double r, double theta) {
	return dist(x, y, scan_x+r*cos(theta), scan_y+r*sin(theta));
}
// end helper functions

	
//pre-declare functions
void startPoseCB(const snowmower_steering::CmdPose& cmd);
void endPoseCB(const snowmower_steering::CmdPose& cmd);
void robotPoseCB(const geometry_msgs::Pose& message_pose);
void robotSimPoseCB(const nav_msgs::Odometry& msg);
void obstacleCB(const snowmower_steering::Obstacle& cmd);
/* Calculation Structure
void update(ros::Publisher& feedback_pub): call this function
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
void update(ros::Publisher& feedback_pub);
	void generate_potential_map();
		void update_obstacles();
		void update_waypoints();
	double evaluate_point(double x, double y, double theta);
		double depart_component(double x, double y, double theta); 
		double arrive_component(double x, double y, double theta); 
		double motive_component(double x, double y, double theta); //mov
		double obstacle_component(double x, double y, double theta); //obs
		//declare assistant functions to determine gains for dep/arr/mov/obs
		double dep_gain(double x, double y, double theta);
		double arr_gain(double x, double y, double theta);
		double mov_gain(double x, double y, double theta);
		double obs_gain(double x, double y, double theta);
	void convert_to_twist();
// end predeclared functions

void startPoseCB(const snowmower_steering::CmdPose& cmd) {
	std::stringstream ss;
	ss << "[" << cmd.pose.position.x << "," << cmd.pose.position.y << "," << cmd.pose.orientation.w << "]";
	ROS_INFO("Recieved new start pose %s", ss.str().c_str());
	start_pose.pose = cmd.pose;
	start_pose.v_arrive = cmd.v_arrive;
	start_pose.v_depart = cmd.v_depart;
	start_pose.plow_angle = cmd.plow_angle;
	start_pose.sleep_time = cmd.sleep_time;
	try {
		jackie.data = start_pose.plow_angle;
		//ROS_INFO("|||| plow angle from start pose: %d ||||", jackie.data);
		plow_angle_pub.publish(jackie);
	}
	catch(int e) {
		ROS_WARN("int error");
	}
	start_flag = true;
	new_route = true;
}

void endPoseCB(const snowmower_steering::CmdPose& cmd) {
	std::stringstream ss;
	ss << "[" << cmd.pose.position.x << "," << cmd.pose.position.y << "," << cmd.pose.orientation.w << "]";
	ROS_INFO("Recieved new end pose %s", ss.str().c_str());
	end_pose.pose = cmd.pose;
	end_pose.v_arrive = cmd.v_arrive;
	end_pose.v_depart = cmd.v_depart; 
	end_flag = true;
	new_route = true;
}

void robotPoseCB(const geometry_msgs::Pose& message_pose) {
	ROS_INFO("Recieved new robot pose");
	robot_pose.position = message_pose.position;
	robot_pose.orientation = message_pose.orientation;
	robot_flag = true;
}
void robotSimPoseCB(const nav_msgs::Odometry& msg) {
	std::stringstream ss;
	ss << "Recieved new robot-odom pose [" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y << ", +-" << 
		2.0*acos(msg.pose.pose.orientation.w) << "]";
	robot_heading = 2.0*acos(msg.pose.pose.orientation.w);
	if (msg.pose.pose.orientation.x < 0.0 || msg.pose.pose.orientation.y < 0.0 || msg.pose.pose.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
	ROS_INFO("%s", ss.str().c_str());
/*
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
    geometry_msgs/Quaternion orientation
*/
	robot_pose.position = msg.pose.pose.position;
	robot_pose.orientation = msg.pose.pose.orientation;
	robot_flag = true;
}
void robotGPSPoseCB(const geometry_msgs::PoseStamped& msg) {
	robot_heading = 2.0*acos(msg.pose.orientation.w);
	if (msg.pose.orientation.x < 0.0 || msg.pose.orientation.y < 0.0 || msg.pose.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
	robot_pose.position = msg.pose.position;
	robot_pose.orientation = msg.pose.orientation;
	robot_flag = true;
}
void robotRealPoseCB(const geometry_msgs::PoseWithCovariance& msg) {
	std::stringstream ss;
	ss << "Recieved new robot-estim pose [" << msg.pose.position.x << "," << msg.pose.position.y << ", +-" << 
		2.0*acos(msg.pose.orientation.w) << "]";
	robot_heading = 2.0*acos(msg.pose.orientation.w);
	if (msg.pose.orientation.x < 0.0 || msg.pose.orientation.y < 0.0 || msg.pose.orientation.z < 0.0) {
		robot_heading = -robot_heading;
	}
	ROS_INFO("%s",ss.str().c_str());
/*
[geometry_msgs/PoseWithCovariance]:
geometry_msgs/Pose pose
  geometry_msgs/Point position
  geometry_msgs/Quaternion orientation
*/
	robot_pose.position = msg.pose.position;
	robot_pose.orientation = msg.pose.orientation;
	robot_flag = true;
}
///*
void obstacleCB(const snowmower_steering::Obstacle& cmd) {
	//obstacle_flag = true;
	if(!ignore_obstacles) { obstacles.push_back(cmd); }
	/*
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
				obstacles[i].size = cmd.size;
				obstacles[i].r = cmd.r;
				obstacles[i].theta = cmd.theta;
			}
		}
	}
	*/
}
//*/
void update(ros::Publisher& feedback_pub, ros::Publisher& output_pub) {
	if (debug_mode) { ROS_INFO("updating"); }
	if (start_flag && end_flag && robot_flag) {
		feedback.data = "repeat";
		if (dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_heading) < 0.10) {
			//at goal point w/in 5 cm
			std::stringstream ss;
			ss << "\n(" << robot_pose.position.x << "," << robot_pose.position.y << "," << robot_heading << ")" 
				<< " is near goal location... (" 
				<< end_pose.pose.position.x << "," << end_pose.pose.position.y << "," << end_pose.pose.orientation.w << ")" 
				<< " at distance " << dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_heading) << " m" ;
			ROS_INFO("%s", ss.str().c_str());
			double turn = end_pose.pose.orientation.w - robot_heading;
			double pi = 3.14159;
			while(turn >= 2.0*pi) { turn = turn - 2.0*pi; }
			while(turn <= -2.0*pi) { turn = turn + 2.0*pi; }
			//TODO check math
			if(turn >= pi) { turn = -2.0*pi + turn; }
			if(turn <= -pi) { turn = 2.0*pi + turn; }

			if (std::abs(turn) < 0.03 || ignore_heading) {
				//at goal orientation w/in 3 degrees
				int turn_min = std::abs(turn) < 0.03;
				std::stringstream s2;
				s2 << "...near goal pose	std::abs( " << turn  <<" ), " << std::abs(turn) << " . iftest( " << turn_min << " )";
				ROS_INFO("%s", s2.str().c_str());
				send_feedback = true;
				//if( feedback.data.compare("next")==0 ) { feedback.data = "repeat"; ROS_INFO("next string minimized");}
				//else{ feedback.data = "next"; }
				feedback.data = "next";
				//super_debug = true;
				feedback_pub.publish(feedback);
				ROS_INFO("msg: Next out");
				control_output.linear.x = 0.0;
				control_output.angular.z = 0.0;
				output_pub.publish(control_output);
				ros::Rate joe(1);
				ros::spinOnce();
				ROS_INFO("joe.sleep start. sleeping for %d", (int) (start_pose.sleep_time));
				for(int i = 0; i < start_pose.sleep_time; i++) {
					ros::spinOnce();
					joe.sleep();
				}
				ros::spinOnce();
				ROS_INFO("end joe sleep");
				feedback.data = "repeat";
				feedback_pub.publish(feedback);
				ROS_INFO("msg: Repeat out");
				d_angle = 0.0;
				d_distance = 0.0;
			}
			else {
				ROS_INFO(".rotating to goal pose");
				send_feedback = true;
				feedback.data = "repeat";
				double turn = end_pose.pose.orientation.w - robot_heading;
				double pi = 3.14159;
				while(turn >= 2.0*pi) { turn = turn - 2.0*pi; }
				while(turn <= -2.0*pi) { turn = turn + 2.0*pi; }
				//TODO check math
				if(turn >= pi) { turn = -2.0*pi + turn; }
				if(turn <= -pi) { turn = 2.0*pi + turn; }
				d_angle = turn;
				d_distance = 0.0;
			}
		}
		else {	
			//main work function for the steering
			if (debug_mode) { ROS_INFO("following potential function.."); }
			generate_potential_map();
			d_angle = evaluate_point(robot_pose.position.x, robot_pose.position.y, robot_heading);
			d_distance = dist_to_goal(robot_pose.position.x, robot_pose.position.y, robot_heading);

		}

		//converts work function output to message format
		convert_to_twist();
	}
	else {
		ROS_INFO("Waiting on poses, etc.");
	}
}
void generate_potential_map() {
	if (debug_methods) { ROS_INFO("---- generate_potential_map"); }
	update_obstacles();
	update_waypoints();
	
}
void update_obstacles() {
	if (debug_methods) { ROS_INFO("---- update_obstacles"); }
	//TODO 1
	//need to add an obstacle message, callback, etc.
}
void update_waypoints() {
	if (debug_methods) { ROS_INFO("---- update_waypoints"); }
	//TODO 1
	// this and update obstacles maybe unnecessary or shifted (maybe moved to callbacks)
}

double evaluate_point(double x, double y, double theta) {
	double dep = depart_component(x,y,theta);
	double arr = arrive_component(x,y,theta);
	double mov = motive_component(x,y,theta);
	double obs = obstacle_component(x,y,theta);
	if (debug_methods || super_debug) { 
		ROS_INFO("---- evaluate_point");
		std::stringstream ss;
		ss << "\n      + dep: " << dep_gain(x,y,theta) << "*" << dep << " " << (dep/3.14*180.0) << "\n      + arr: " << arr_gain(x,y,theta) << "*" << arr << " " << (arr/3.14*180.0) << "\n      + mov: "  << mov_gain(x,y,theta) << "*" << mov << " " << (mov/3.14*180.0) << "\n      + obs: " << obs_gain(x,y,theta) << "*" << obs << " " << (obs/3.14*180.0) << "" ;
		ROS_INFO("%s", ss.str().c_str());
	}
	double dx = 0.0;
	double dy = 0.0;
	//TODO **Note: soften obstacle edge
	if (obs == -77.7) {
		if (debug_methods) { ROS_INFO("no obstacles mode"); }
		dx = dep_gain(x,y,theta)*cos(dep) + arr_gain(x,y,theta)*cos(arr) + mov_gain(x,y,theta)*cos(mov);
		dy = dep_gain(x,y,theta)*sin(dep) + arr_gain(x,y,theta)*sin(arr) + mov_gain(x,y,theta)*sin(mov);
	}
	else {
		if (debug_methods) { ROS_INFO("obstacles considered"); }
		dx = obs_gain(x,y,theta)*cos(obs) + dep_gain(x,y,theta)*cos(dep) + arr_gain(x,y,theta)*cos(arr) + mov_gain(x,y,theta)*cos(mov);
		dy = obs_gain(x,y,theta)*sin(obs) + dep_gain(x,y,theta)*sin(dep) + arr_gain(x,y,theta)*sin(arr) + mov_gain(x,y,theta)*sin(mov);
	}
	double dth = atan2(dy, dx); //desired theta angle
	//need to do the [robocode] min_turn analysis
	double turn = dth - robot_heading;
	std::stringstream ss;
	ss << "eval point out -->\n          " << "dx: " << dx << " dy:" << dy << " \n          dth:" << dth << " robot_pose.or...w: " << robot_heading << " turn-in: " << turn << "";
	ROS_INFO("%s", ss.str().c_str());
	double pi = 3.14159;
	while(turn >= 2.0*pi) { turn = turn - 2.0*pi; }
	while(turn <= -2.0*pi) { turn = turn + 2.0*pi; }
	//TODO check math
	if(turn >= pi) { turn = -2.0*pi + turn; }
	if(turn <= -pi) { turn = 2.0*pi + turn; }
	return turn;
}
double depart_component(double x, double y, double theta) {
	//dist from line source
	double angle_offset = atan(k_stable*dist_from_line(x,y, start_pose.pose.position.x, start_pose.pose.position.y, cos(start_pose.pose.orientation.w), sin(start_pose.pose.orientation.w)));

	return start_pose.pose.orientation.w - angle_offset;
	//return 0.0;
}
double dep_gain(double x, double y, double theta) {

	if (simple_mode) { return 0.0; }
	else {
		double d_start = dist_to_start(x,y,theta);
		double d_full = dist_to_goal(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.orientation.w);
		double d_limit = 1.0;
		if (d_full / 3.0 < d_limit) { d_limit =  d_full / 3.0; }
		if(d_start < d_limit) {
			return 0.5;
		}
		else {
			return std::min(0.5, d_limit / pow(dist_to_start(x,y,theta),3));
		}
	}
}
double arrive_component(double x, double y, double theta) {
	//dist from line sink
	double angle_offset = atan(-1.0*k_stable*dist_from_line(x,y, 
		end_pose.pose.position.x, end_pose.pose.position.y, 
		cos(end_pose.pose.orientation.w), sin(end_pose.pose.orientation.w)));

	return end_pose.pose.orientation.w + angle_offset;
}
double arr_gain(double x, double y, double theta) {
	if (simple_mode) { return 0.0; }
	else if(dist_to_goal(x,y,theta) < 0.5) {
		//when close, decrease influence, focus on getting there and then rotating
		return dist_to_goal(x,y,theta);
	}
	else {
		return std::min(0.5, 0.5 / pow(dist_to_goal(x,y,theta),3));
	}
} 
double motive_component(double x, double y, double theta) {
	//single radial sink
	double dy = end_pose.pose.position.y - robot_pose.position.y;
	double dx = end_pose.pose.position.x - robot_pose.position.x;
	if (debug_methods) { ROS_INFO("MOTIVE CALC"); }
	std::stringstream ss;
	ss << "\ndy: " << dy << " dx: " << dx << "";
	if (debug_methods) { ROS_INFO("%s", ss.str().c_str()); }
	return atan2(dy,dx);
}
double mov_gain(double x, double y, double theta) {
	return 1.0;
}
double obstacle_component(double x, double y, double theta) {
	//multiple radial source
	/*
		sums the vectors for nearby objects to establish the obstacle avoidance vector
		TODO if given a direction to the object, return the effect based on distance in the opposite direction. local avoidance. bam.
		TODO 1 get a effect function for distance to give each object influence. Also, assume that new obstacles are published every step to be avoided?
	*/
	double dx = 0.0;
	double dy = 0.0;

	for (int i = 0; i < obstacles.size(); i++) {
		ROS_INFO("obstacle num %d", i+1);
		dx = dx + std::min(1.0,obstacles[i].size/pow(obstacles[i].r, 3.0))*cos(obstacles[i].theta+3.14+robot_heading);
		dy = dy + std::min(1.0,obstacles[i].size/pow(obstacles[i].r, 3.0))*sin(obstacles[i].theta+3.14+robot_heading);
	}
	// error or no obstacle effect
	if (std::abs(dx) <= 0.03 && std::abs(dy) <= 0.03) {
		if (debug_mode) { ROS_INFO("No obstacles."); }
		return -77.7;
	}
	else {
		if (debug_mode) { ROS_INFO("Obstacles nearby"); }
		return atan2(dy, dx);
	}
	obstacles.clear();
}
double obs_gain(double x, double y, double theta) {
	if(simple_mode || ignore_obstacles) return 0.0;
	return 0.5;
}
void convert_to_twist() {
	//takes controller input and makes it into a Twist

	//2d non-holonomic constraints
	control_output.linear.y = 0;
	control_output.linear.z = 0;
	control_output.angular.x = 0;
	control_output.angular.y = 0;

	//commands
	control_output.linear.x = std::max(-max_v, std::min(max_v,k1*d_distance+end_pose.v_arrive));
	control_output.angular.z = std::max(-max_w, std::min(max_w, k2*d_angle));
	std::stringstream ss;
	ss << "--> Twist Out: v: "<< control_output.linear.x <<" w: " << control_output.angular.z << "\n---> from k1*d_dist: " << k1*d_distance << " and k2*d_angle: " << k2*d_angle;
	ROS_INFO("updating to new commands \n          %s", ss.str().c_str());
}

void debug_method_check() {
	ROS_INFO("start verbose method check");
	ROS_INFO("Helper methods: ");
/*
	double dist(double x1, double y1, double x2, double y2) { return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)); }
	double dist_from_line(double x, double y, double xt0, double yt0, double xt, double yt) {
	double dist_to_goal(double x, double y, double theta);
*/
	std::stringstream ss;
	ss << " -- dist(0,0,3,4) => " <<  dist(0,0,3,4) << " ? 5";
	ROS_INFO("%s", ss.str().c_str());
		ss.clear();
		ss.str("");
	ss << " -- dist_from_line(3,4,0,0,1,0) => " << dist_from_line(3,4,0,0,1,0) << " ? 4";
	ROS_INFO("%s", ss.str().c_str());
		ss.clear();
		ss.str("");
	ss << " -- dist_to_goal(4.0,4.0,0.0) => " << dist_to_goal(4.0, 4.0, 0.0) << " ? ?";
	ROS_INFO("%s", ss.str().c_str());
		ss.clear();
		ss.str("");
	ss << " -- abs( " << abs(.045) << " ) / std::abs( " << std::abs(.045) << " )";
	ROS_INFO("%s", ss.str().c_str());
	ROS_INFO("end verbose method check");
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"baskin_steering");
	ROS_INFO("argc --> %d", argc);
	for(int i = 0; i < argc; i++) {
		ROS_INFO("argv --> %s", *(argv+i));
	}
	if(argc > 1 && strcmp(*(argv+1), "true")*strcmp(*(argv+1), "1") == 0) {
		super_debug = true;
	}
	else {
		super_debug = false;
	}
	if(argc > 2 && strcmp(*(argv+2), "true")*strcmp(*(argv+2), "1") == 0) {
		ignore_obstacles = true;
	}
	else {
		ignore_obstacles = false;
	}
	ros::NodeHandle n;
	ros::Rate timer(20);
	if (debug_methods || debug_mode || true) { debug_method_check(); }
	if (debug_speed) {timer = ros::Rate(1); }
	ROS_INFO("ROS init for baskin_steering");

	///*

	ros::Publisher output_pub = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("/steering/feedback",1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
	plow_angle_pub = n.advertise<std_msgs::UInt8>("/plow/angle",1);
	ros::Subscriber sub_start = n.subscribe ("/steering/start_pose", 1, startPoseCB);
	ros::Subscriber sub_end = n.subscribe ("/steering/end_pose", 1, endPoseCB);
	ros::Subscriber sub_robot = n.subscribe ("/steering/robot_pose", 1, robotPoseCB);
	ros::Subscriber sub_robot_odom = n.subscribe ("/robot0/odom", 1, robotSimPoseCB);
	ros::Subscriber sub_robot_est = n.subscribe ("/pose_est" , 1, robotRealPoseCB);
	ros::Subscriber sub_obstacle = n.subscribe ("/steering/obstacle", 1, obstacleCB);

	ros::spinOnce();
	d_distance = 1.0;
	d_angle = 2.0;
	send_feedback = true;
	feedback.data = "restart";
	feedback_pub.publish(feedback);
	feedback_pub.publish(feedback);
	feedback_pub.publish(feedback);

	control_output.linear.x = 0.0;
	control_output.angular.z = 0.0;
	std::stringstream ss;
	ss << "--> Twist Out: v: "<< control_output.linear.x <<" w: " << control_output.angular.z << " ";
	ROS_INFO("updating to new commands %s", ss.str().c_str());
	output_pub.publish(control_output);

	int input = 1;
	/*std::cout << "begin operation?" << std::endl;
	std::cin >> input;
	std::cout << "\n\n\n\n" << std::endl;*/
	if (input >=0) {

	//visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = robot_pose;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.lifetime = ros::Duration();
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	int counter = 0 ;
	while(ros::ok())
	{
		ros::spinOnce();
		//publish
		//do something with control_output
		send_feedback = false;
		update(feedback_pub, output_pub);
		obstacles.clear();
		new_route = false;
		if (debug_mode) { ROS_INFO("Publishing control commands for robot"); }
		ros::spinOnce();
		if (super_debug) { 
			ros::spinOnce();
			int input = 1;
			std::cout << "publish message?" << std::endl;
			std::cin >> input;
			if(input > 10) { super_debug = false; }
			if(input > 0) {
				output_pub.publish(control_output); 
			}
			else if (input == 0) {
				control_output.linear.x = 0.0;
				control_output.angular.z = 0.0;
				output_pub.publish(control_output);
			}
			else { break; }
			std::cout << "\n\n\n\n" << std::endl;
		}
		else { 
			ros::spinOnce();
			output_pub.publish(control_output);
		}
		if (counter % 100 == 0) {
			//update marker
			marker.header.stamp = ros::Time::now();
			marker.pose = robot_pose;
			ROS_INFO("marker published");
//			marker_array.markers.pushback(marker);
//			marker_pub.publish(marker_array);
			marker_pub.publish(marker);
		}
		counter = counter + 1;

		// request refresh from planner
		if(send_feedback || true) { feedback_pub.publish(feedback); }

		//subscribe
		ros::spinOnce();
		if (ros::isShuttingDown()) {
			control_output.linear.x = 0.0;
			control_output.angular.z = 0.0;
			std::stringstream ss;
			ss << "--> Twist Out: v: "<< control_output.linear.x <<" w: " << control_output.angular.z << " ";
			ROS_INFO("updating to new commands %s", ss.str().c_str());
			output_pub.publish(control_output);
		}
		timer.sleep();
	}
	//*/
	//on node terminate, stop
	control_output.linear.x = 0.0;
	control_output.angular.z = 0.0;
	std::stringstream ss;
	ss << "--> Twist Out: v: "<< control_output.linear.x <<" w: " << control_output.angular.z << " ";
	ROS_INFO("updating to new commands %s", ss.str().c_str());
	output_pub.publish(control_output);
	}
	return 0;
}
