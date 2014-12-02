#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

//rviz has a button that says "2D Nav Goal". When this button is pressed, a PoseStamped message is published to the topic "/move_base_simple/goal". This callback function is called when a message is published to that topic, converts the PoseStamped message into a stamped transform, and broadcasts it. 
void goalPose_CB(const geometry_msgs::PoseStamped& msg){
  //Create a broadcaster object (should this be done once in main?)
  static tf::TransformBroadcaster br;

  //Create a transform object (should this be done once in main?)
  tf::Transform transform;

  //Set the origin of the transform using the x,y,z members of msg.pose.position. msg is the PoseStamped that's being passed into the callback funtion.
  transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));

  //Set the orientation of the transform using the x,y,z,w members msg.pose.orientation. msg is the PoseStamped that's being passed into the callback funtion.
  transform.setRotation(tf::Quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w));

  //Finally, broadcast the Transform. It is possible that this should be done over and over at a specific rate.
  br.sendTransform(tf::StampedTransform(transform, msg.header.stamp,"map_static","goalPose"));
}

int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "PoseStamped_to_TF");
  ros::NodeHandle nh;

  //Create a subscriber object which subscribes to the topic that rviz publishes to when a "2D Nav Goal" is selected.
  ros::Subscriber goalPose_sub = nh.subscribe("/move_base_simple/goal",1,goalPose_CB);

  //Spin, listening for a published message
  while (ros::ok()){
    ros::spin();
  }
  return 0;
}
