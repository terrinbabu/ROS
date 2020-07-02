#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <tsupport/moveit_utils.h>

#include <moveit/move_group_interface/move_group_interface.h> 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  
  init(argc, argv,"planning_scene");
  NodeHandle nh;
  AsyncSpinner spinner(1);
  spinner.start(); 
  sleep(5.0);

  tsupport::basic::print_cyan ("\n Adding the table \n\n");
  std::vector<double> table_pose = {0,0,-0.8};
  std::vector<double> table_orient = {1, 0, 0, 0};
  std::vector<double> table_dim = {5, 5, 0.025};

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
   sleep(0.5);
 
  tsupport::moveit_utils::addBox(planning_scene_diff_publisher, table_pose, table_orient,table_dim,"world","world","table");

}
