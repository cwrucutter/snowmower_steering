#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
	ros::init(argc,argv,"baskin_steering");
	ros::NodeHandle n;
	ros::Publisher output_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	double w_correction = 0.0;
	double run_time = 3.0;
	
	if(argc > 2) {
		w_correction = atof(*(argv+1));
		run_time = atof(*(argv+2));
	}
	
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	double hz = 10;	
	ros::Rate timer(hz);
	output_pub.publish(msg);
	for(double i = 0; i/hz < run_time; i = i +1.0) {
		msg.linear.x = 1.0;
		msg.angular.z = 0.0+w_correction;
	}
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	output_pub.publish(msg);
	
	return 0;
	
}
