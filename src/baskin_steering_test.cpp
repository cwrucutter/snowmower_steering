#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <snowmower_steering/CmdPose.h>
#include <snowmower_steering/Obstacle.h>

ros::Publisher start_pose_pub;
ros::Publisher end_pose_pub;
ros::Publisher robot_pose_pub;
ros::Publisher obstacle_pub;

bool debug_mode = true;
bool sim_mode = true;

geometry_msgs::Pose robot_pose;

//data set
std::vector< snowmower_steering::CmdPose > waypoints;
int waypoints_index = 0;

void outputCB(const geometry_msgs::Twist& cmd) {
	std::stringstream ss;
	ss << "--> Twist Out: v: "<< cmd.linear.x <<" w: " << cmd.angular.z << " ";
	ROS_INFO("%s", ss.str().c_str()) ;
}

void feedbackCB(const std_msgs::String& cmd) {
	std::stringstream ss;
	ss << "--> Feedback: " << std::string(cmd.data) << " ";
	ROS_INFO("%s", ss.str().c_str()) ;

	if (cmd.data.compare("restart") == 0) { 
		waypoints_index = 0 ; 
	}
	else if (cmd.data.compare("next") == 0) {
		waypoints_index = (waypoints_index + 1)% waypoints.size() ;
	}
	else if (cmd.data.compare("back") == 0) {
		waypoints_index = (waypoints_index - 1 + waypoints.size())% waypoints.size() ;
	}
	else { // (cmd.data == "resfresh")
		waypoints_index = waypoints_index % waypoints.size() ;
	}

	try { 
		ROS_INFO("new waypoints out");
		start_pose_pub.publish(waypoints[waypoints_index % waypoints.size()]);
		end_pose_pub.publish(waypoints[(waypoints_index+1) % waypoints.size()]);
	}
	catch(int e) { ROS_INFO("int Feedback error"); }
}

void setup_test() {
	ROS_INFO("Setting up test.");
	snowmower_steering::CmdPose c;
/*
[snowmower_steering/CmdPose]:
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
float64 v_arrive
float64 v_depart

*/
	c.v_arrive = 0.0;
	c.v_depart = 0.0;
	geometry_msgs::Pose p;
		p.position.x = 7.5;
		p.position.y = 4.0;
		p.position.z = 0.0;
		p.orientation.x = 0.0;
		p.orientation.y = 0.0;
		p.orientation.z = 1.0;
		p.orientation.w = 0.0;
	c.pose = p;
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test..");

	c.pose.position.x = 9.5;	
	c.pose.position.y = 4.0;
	c.pose.orientation.w = 0.0;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test...");

	c.pose.position.x = 9.5;	
	c.pose.position.y = 5.0;
	c.pose.orientation.w = 3.14;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test....");

	c.pose.position.x = 7.5;	
	c.pose.position.y = 7.0;
	c.pose.orientation.w = 3.14;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test..");

	c.pose.position.x = 7.5;
	c.pose.position.y = 8.0;
	c.pose.orientation.w = 0.0;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test..");

	c.pose.position.x = 8.5;	
	c.pose.position.y = 8.5;
	c.pose.orientation.w = -0.785;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Setting up test..");

	c.pose.position.x = 8.5;	
	c.pose.position.y = 6.5;
	c.pose.orientation.w = -2.35;
	
	waypoints.insert(waypoints.end(), c);
	ROS_INFO("Test setup complete.");

}

int main(int argc, char** argv) {
	ROS_INFO("ROS init for baskin_steering_test");
	ros::init(argc,argv,"baskin_steering_test");
	ros::NodeHandle n;
	ros::Rate timer(10);
	setup_test();	
	/*
	//Steering talk/listen
	ros::Publisher output_pub = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("/steering/feedback",1);

	ros::Subscriber sub_start = n.subscribe ("/steering/start_pose", 1, startPoseCB);
	ros::Subscriber sub_end = n.subscribe ("/steering/end_pose", 1, endPoseCB);
	ros::Subscriber sub_robot = n.subscribe ("/steering/robot_pose", 1, robotPoseCB);
	ros::Subscriber sub_robot_odom = n.subscribe ("/robot0/odom", 1, robotSimPoseCB);
	ros::Subscriber sub_obstacle = n.subscribe ("/steering/obstacle", 1, obstacleCB);
	*/
	//Test send-interaction points:
	start_pose_pub = n.advertise<snowmower_steering::CmdPose>("/steering/start_pose",1);
	end_pose_pub = n.advertise<snowmower_steering::CmdPose>("/steering/end_pose",1);
	robot_pose_pub = n.advertise<geometry_msgs::Pose>("/steering/robot_pose",1);
	obstacle_pub = n.advertise<snowmower_steering::Obstacle>("/steering/obstacle",1);
	//Test recieve-interaction points:
	ros::Subscriber output_sub = n.subscribe ("/robot0/cmd_vel", 1, outputCB);
	ros::Subscriber feedback_sub = n.subscribe ("/steering/feedback", 1, feedbackCB);

	ROS_INFO("Publishers and Subscribers set.");
	
	start_pose_pub.publish(waypoints[(waypoints_index+0)%waypoints.size()]);
	end_pose_pub.publish(waypoints[(waypoints_index+1)%waypoints.size()]);
	ROS_INFO("Published first waypoint.");

	while(ros::ok())
	{
		ros::spinOnce();
		//publish robot positions
		double x;
		double y;
		double theta;
		if (!sim_mode) {
			if (!ros::ok()) {break;}
			ROS_INFO("Choosing robot position");
			std::cout << "\nChoose robot_x\n" << std::endl;
			std::cin >> x;
			if (!ros::ok()) {break;}
			std::cout << "\nChoose robot_y\n" << std::endl;
			std::cin >> y;
			if (!ros::ok()) {break;}
			std::cout << "\nChoose robot_theta\n" << std::endl;
			std::cin >> theta;

			robot_pose.position.x = x;
			robot_pose.position.y = y;
			robot_pose.position.z = 0.0;
			robot_pose.orientation.x = 0.0;
			robot_pose.orientation.y = 0.0;
			robot_pose.orientation.z = 1.0;
			robot_pose.orientation.w = theta;

			ROS_INFO("Publishing new position");
			robot_pose_pub.publish(robot_pose);
		}
		else {
			//ROS_INFO("Running robot in simulator mode");
		}
		//subscribe
		ros::spinOnce();
		timer.sleep();
	}
	//*/
	return 0;
}
