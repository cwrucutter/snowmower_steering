#snowmower_steering#

##Nodes##

###simple_steering.cpp###


This node looks up the transform between the `goalPose` tf and the `robot0` tf and publishes a Twist message on the topic `/robot0/cmd_vel` to get the robot to `goalPose`. Currently, the algorithm only gets the robot to the position, not the orientation.

###PoseStamped_to_TF.cpp###

This node is a bridge between the PoseStamped published to topic `/move_base_simple/goal` by rviz when the "2D Nav Goal" button is pressed and the mouse is clicked and dragged on the screen. The node subscribes to this topic and then broadcasts a transform called `goalPose`.

Currently for this to work, the Fixed Frame in rviz has to be `map_static`. Note that when gmapping is running, this changes the `map_static` transform and can kind of mess things up, I think. I need to dig into gmapping to see what it's actually changing.

##Launch Files##

###stdr_and_gmapping.launch###

This launch file launches a stdr simulator, slam_gmapping, simple_steering and PoseStamped_to_TF, and rviz with an associated config file. Sometimes opening rviz at the same time as everything else results in errors. If so, close everything and relaunch. It usually works for me every other time. You can also remove rviz from the launch file and run it separately.

From rviz, press `g`, which is a keyboard short cut for clicking the button "2D Nav Goal". Then click anywhere on the rviz screen to guide the robot around the map.
