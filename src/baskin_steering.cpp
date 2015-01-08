#include <ros/ros.h>
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

double d_distance;
double d_angle;
double k1; //linear adjustment coefficient
double k2; //angular adjustment coefficient

geometry_msgs::Twist control_output; // linear.x = forward, angular.z = turn
std_msgs::String feedback;
	
//pre-declare functions
void startPoseCB(const baskin_steering::CmdPose& cmd);
void endPoseCB(const baskin_steering::CmdPose& cmd);
void robotPoseCB(const geometry_msgs::Pose& message_pose);
void update();
void convert_to_twist();
//end pre-declare functions

void startPoseCB(const baskin_steering::CmdPose& cmd) { 
	start_pose.pose = cmd.pose;
	start_pose.v_arrive = cmd.v_arrive;
	start_pose.v_depart = cmd.v_depart;

}

void endPoseCB(const baskin_steering::CmdPose& cmd) {
	end_pose.pose = cmd.pose;
	end_pose.v_arrive = cmd.v_arrive;
	end_pose.v_depart = cmd.v_depart; 
}

void robotPoseCB(const geometry_msgs::Pose& message_pose) {
	robot_pose.position = message_pose.position;
	robot_pose.orientation = message_pose.orientation;

}

void update() {
	//main work function for the steering
	d_distance = 3-d_distance;
	d_angle = 3-d_angle;
	//converts work function output to message format
	convert_to_twist();

}
void convert_to_twist() {
	//takes controller input and makes it into a Twist

	//2d non-holonomic constraints
	control_output.linear.y = 0;
	control_output.linear.z = 0;
	control_output.angular.x = 0;
	control_output.angular.y = 0;

	//commands
	control_output.linear.x = k1*d_distance;
	control_output.angular.z = k2*d_angle;
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
