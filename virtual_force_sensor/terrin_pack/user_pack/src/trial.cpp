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
  
  init(argc, argv,"trial");
  NodeHandle nh;
  AsyncSpinner spinner(1);
  spinner.start(); 
  sleep(3.0);
  
  tsupport::basic::print_cyan ("\n Group Info \n\n");
  
  const string arm_planning_group_name = "manipulator";
// //   const string end_effector_name = "ee_link";
  
  moveit::planning_interface::MoveGroupInterface group_arm(arm_planning_group_name);

// //   tsupport::basic::print_yellow ("Reference frame of the manipulator - ");
// //   cout   << group_arm.getPlanningFrame().c_str() << endl;
// //   
// //   tsupport::basic::print_yellow ("Current Position of the manipulator - ");
// //   cout   << "[ " << fixed << setprecision(2) 
// // 			<< group_arm.getCurrentPose().pose.position.x  << ","
// // 			<< group_arm.getCurrentPose().pose.position.y  << ","
// // 			<< group_arm.getCurrentPose().pose.position.z  << " ]" << endl;

// //   tsupport::basic::print_yellow ("Current Orientation of the manipulator - ");
// //   cout << "[ "  << fixed << setprecision(2)
// // 		 << group_arm.getCurrentPose().pose.orientation.w << ","
// // 		 << group_arm.getCurrentPose().pose.orientation.x << ","
// // 		 << group_arm.getCurrentPose().pose.orientation.y << ","
// // 		 << group_arm.getCurrentPose().pose.orientation.z << " ]" << endl;
// // 
// //   vector<double> current_rotational_joint_values;
// //   group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), current_rotational_joint_values);
// //   tsupport::basic::printvectordouble("current_rotational_joint_values",current_rotational_joint_values);
// // 
// //   tsupport::basic::print_cyan ("\n Adding the table \n\n");
// //   std::vector<double> table_pose = {0,0,-0.025};
// //   std::vector<double> table_orient = {1, 0, 0, 0};
// //   std::vector<double> table_dim = {5, 5, 0.025};
// // 
// //   ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
// //   while(planning_scene_diff_publisher.getNumSubscribers() < 1)
// //    sleep(0.5);
// //  
// //   tsupport::moveit_utils::addBox(planning_scene_diff_publisher, table_pose, table_orient,table_dim,"world","world","table");

  moveit::planning_interface::MoveItErrorCode success;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  geometry_msgs::Pose goal_pose;
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  
// // //   tsupport::basic::print_cyan ("\n Planning to pose goal \n\n");
// // //   
// // //   goal_pose.position.x = group_arm.getCurrentPose().pose.position.x ;
// // //   goal_pose.position.y = group_arm.getCurrentPose().pose.position.y + 0.1;
// // //   goal_pose.position.z = group_arm.getCurrentPose().pose.position.z ;
// // //   
// // //   goal_pose.orientation.w = group_arm.getCurrentPose().pose.orientation.w;
// // //   goal_pose.orientation.x = group_arm.getCurrentPose().pose.orientation.x;
// // //   goal_pose.orientation.y = group_arm.getCurrentPose().pose.orientation.y;
// // //   goal_pose.orientation.z = group_arm.getCurrentPose().pose.orientation.z;
// // //   
// // //   group_arm.setStartStateToCurrentState();
// // //   group_arm.setPoseTarget(goal_pose);
// // // 
// // //   success = group_arm.plan(my_plan);
// // // 
// // //   tsupport::basic::print_magenta ("plan 1 (pose goal)");
// // //   
// // //   if(success)
// // //   		tsupport::basic::print_green (" SUCCESS ");
// // //   else 
// // // 		tsupport::basic::print_red (" FAILED ");
// // //   
// // //   tsupport::basic::print_magenta ("Visualizing plan 1 ");  
// // // 
// // //   moveit_msgs::DisplayTrajectory display_trajectory_1;
// // //   display_trajectory_1.trajectory_start = my_plan.start_state_;
// // //   display_trajectory_1.trajectory.push_back(my_plan.trajectory_);
// // //   display_publisher.publish(display_trajectory_1);
// // //   sleep(3.0);
// // // 
// // //   if(tsupport::basic::check())
// // //   {
// // //   	 tsupport::basic::print_magenta ("Executing plan 1"); 
// // //     group_arm.execute(my_plan);
// // //   }



    std::vector<double> start_joint_values;
    group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
    
tsupport::basic::printvectordouble("start_joint_values",start_joint_values);
    
    std::vector<double> goal_joint = {0, 0, 0, 0, 0, 0};  
    goal_joint[0] = 0;
    goal_joint[1] = -3.14;
    goal_joint[2] = -3.14;
    goal_joint[3] = -3.14;
    goal_joint[4] = 0;
    goal_joint[5] = 0;

    group_arm.setStartStateToCurrentState();
    group_arm.setJointValueTarget(goal_joint); 
    success = group_arm.plan(my_plan);

    tsupport::basic::print_magenta ("plan 1 (joint space goal) : ");
    
    if(success)
    {
        tsupport::basic::print_green (" SUCCESS ");
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(1.0);
        
        if(tsupport::basic::check())
            group_arm.execute(my_plan);
    }
    else 
    {
        tsupport::basic::print_red (" FAILED ");
        cout << "\n" << endl;
        return 0;
    }

}
