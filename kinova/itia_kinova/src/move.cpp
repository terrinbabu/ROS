#include <iostream>
#include <fstream>

#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

#include <kinova_msgs/PoseVelocity.h>

using namespace std;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "move");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start(); 
  sleep(10.0);
     
  bool gazebo;  bool fake;
  itia::kinova_utils::active_controller(nh,"/move/gazebo","/move/fake",fake,gazebo);
  
  moveit::planning_interface::MoveGroupInterface group_arm("arm");  //srdf  
  
  itia::moveit_utils::group_info("arm",group_arm);
  
// PLAN_VISUALIZE_EXECUTE
  
  std::vector<double> start_joint_values; 
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
  
  std::vector<double> goal_joint = {0, 0, 0, 0, 0, 0};    
  goal_joint[0] = start_joint_values[0]+2.5;
  goal_joint[1] = start_joint_values[1]+1.5;
  goal_joint[2] = start_joint_values[2]+1.5;
  goal_joint[3] = start_joint_values[3];
  goal_joint[4] = start_joint_values[4];
  goal_joint[5] = start_joint_values[5];

  itia::moveit_utils::plan_visualize_execute(nh,group_arm,goal_joint);
  
  
// FINGER MOVEMENTS
  
  itia::kinova_utils::closehand(fake,gazebo);
  itia::kinova_utils::openhand(fake,gazebo);
  
  
// //   ros::Publisher pose_velocity_pub  = nh.advertise<kinova_msgs::PoseVelocity> ( "/j2n6s300_driver/in/cartesian_velocity", 1000 );
// //   
// //   kinova_msgs::PoseVelocity twist;
// //   twist.twist_linear_x = 0.25;
// //   twist.twist_linear_y = 0.0;
// //   twist.twist_linear_z = 0.0;
// //   twist.twist_angular_x = 0.0;
// //   twist.twist_angular_y = 0.0;
// //   twist.twist_angular_z = 0.0;  
// //   
// //   ros::Rate loop_rate(1000);
// //   
// //   while (ros::ok())
// //   {
// //     pose_velocity_pub.publish (twist);
// //     loop_rate.sleep();
// //   }
  
  itia::support::end();
  
  return 0;
}