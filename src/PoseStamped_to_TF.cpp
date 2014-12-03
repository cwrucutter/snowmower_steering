#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Make a global PoseStamped variable. This is reassigned by goalPose_CB everytime rviz publishes a PoseStamped when the "2D Nav Goal" button is pressed.
geometry_msgs::PoseStamped g_pose;

//rviz has a button that says "2D Nav Goal". When this button is pressed, a PoseStamped message is published to the topic "/move_base_simple/goal". This callback function is called when a message is published to that topic, converts the PoseStamped message into a stamped transform, and broadcasts it. 
void goalPose_CB(const geometry_msgs::PoseStamped& msg){
  //Write published PoseStamped to the global variable
  g_pose = msg;
}

int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "PoseStamped_to_TF");
  ros::NodeHandle nh;

  //Create a subscriber object which subscribes to the topic that rviz publishes to when a "2D Nav Goal" is selected.
  ros::Subscriber goalPose_sub = nh.subscribe("/move_base_simple/goal",1,goalPose_CB);

  tf::TransformListener listener;

  tf::StampedTransform tf_temp;

  //This variable indicates whether a transform was found or not.
  bool test = true;
  while (nh.ok() && test){
    try{
      //set the test to false, which will break the loop! A transform was found!
      test = false;
      listener.lookupTransform("/map", "/robot0",ros::Time(0), tf_temp);
    }
    catch (tf::TransformException ex){
      //But, set the test back to true if an error was found.
      test = true;
      //Print the error and wait a second.
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  //Initialize the position and orientation of g_pose with the origin and the rotation of this transform.
  g_pose.pose.position.x = tf_temp.getOrigin().x();
  g_pose.pose.position.y = tf_temp.getOrigin().y();
  g_pose.pose.position.z = tf_temp.getOrigin().z();
  g_pose.pose.orientation.x = tf_temp.getRotation().getAxis().x();
  g_pose.pose.orientation.y = tf_temp.getRotation().getAxis().y();
  g_pose.pose.orientation.z = tf_temp.getRotation().getAxis().z();
  g_pose.pose.orientation.w = tf_temp.getRotation().getW();

  //Create a broadcaster object
  static tf::TransformBroadcaster br;

  //Create a transform object
  tf::Transform transform;

  ros::Rate naptime(10);

  //Spin, listening for a published message
  while (ros::ok()){
    //Check for published message
    ros::spinOnce();

    //Set the origin of the transform using the x,y,z members of msg.pose.position. msg is the PoseStamped that's being passed into the callback funtion.
    transform.setOrigin(tf::Vector3(g_pose.pose.position.x, g_pose.pose.position.y, 0.0));

    //Set the orientation of the transform using the x,y,z,w members msg.pose.orientation. msg is the PoseStamped that's being passed into the callback funtion.
    transform.setRotation(tf::Quaternion(g_pose.pose.orientation.x,g_pose.pose.orientation.y,g_pose.pose.orientation.z,g_pose.pose.orientation.w));

    //Finally, broadcast the Transform. It is possible that this should be done over and over at a specific rate.
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"map_static","goalPose"));

    //Sleep for rest of loop cyle
    naptime.sleep();
  }
  return 0;
}
