#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

void goalPose_CB(const geometry_msgs::PoseStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));
  tf::Quaternion q;
  q.setRPY(0,0,msg.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg.header.stamp,"map_static","goalPose"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "2Dpose_to_TF");
  ros::NodeHandle nh;

  ros::Subscriber goalPose_sub = nh.subscribe("/move_base_simple/goal",1,goalPose_CB);

  while (ros::ok()){
    ros::spin();
  }
  return 0;
}
