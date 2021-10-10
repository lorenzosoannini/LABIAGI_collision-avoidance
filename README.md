# LABIAGI_collision-avoidance

A simple ROS node for collision avoidance: it uses as inputs a cmd_vel and a LaserScan and publishes a new cmd_vel that avoids obstacles.

Tested with `stage_ros/stageros` and `teleop_twist_keyboard_cpp/teleop_twist_keyboard`

## Build and run 

* Copy all the contents of this repository in the `src` folder of your Workspace
* Open a terminal in your Workspace folder and build with `catkin_make`
* Run with `rosrun collision_avoidance SimNode`