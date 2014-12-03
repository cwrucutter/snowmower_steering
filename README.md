snowmower_steering
==================

Two nodes:

simple_steering.cpp
This node looks up the transform between the goalPose tf and the robot0 tf and publishes a Twist message on the topic /robot0/cmd_vel to get the robot to goalPose. Currently, the algorithm only gets the robot to the position, not the orientation.

PoseStamped_to_TF.cpp
This node is a bridge between the PoseStamped published to topic /move_base_simple/goal by rviz when the "2D Nav Goal" button is pressed and the mouse is clicked and dragged on the screen. The node subscribes to this topic and then broadcasts a transform called goalPose.

Currently for this to work, the Fixed Frame in rviz has to be map_static. Note that when gmapping is running, this changes the map_static transform and can kind of mess things up, I think. I need to dig into gmapping to see what it's actually changing.
