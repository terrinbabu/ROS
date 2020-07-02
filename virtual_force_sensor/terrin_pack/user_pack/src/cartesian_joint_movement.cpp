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

#include "std_msgs/Bool.h"

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{

init(argc, argv,"cartesian_joint_movement");
NodeHandle nh;
AsyncSpinner spinner(1);
spinner.start(); 
sleep(1.0);

const string arm_planning_group_name = "manipulator";

moveit::planning_interface::MoveGroupInterface group_arm(arm_planning_group_name);

moveit::planning_interface::MoveItErrorCode success;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
int type_of_goal;

///////////////////////////////////////////////////////////
///                        1                          ////
/////////////////////////////////////////////////////////

if (!nh.getParam("/goal_1/type_of_goal",type_of_goal))
{
    ROS_ERROR (" No goal defined ");
    return 0;
}

cout << "type of goal : "  << type_of_goal << endl;

if(type_of_goal == 1)
{

    double tran_x, tran_y, tran_z;
    nh.getParam("/goal_1/move/x",tran_x);
    nh.getParam("/goal_1/move/y",tran_y);
    nh.getParam("/goal_1/move/z",tran_z);
    
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = group_arm.getCurrentPose().pose.position.x + tran_x;
    goal_pose.position.y = group_arm.getCurrentPose().pose.position.y + tran_y;
    goal_pose.position.z = group_arm.getCurrentPose().pose.position.z + tran_z;
    goal_pose.orientation.w = group_arm.getCurrentPose().pose.orientation.w;
    goal_pose.orientation.x = group_arm.getCurrentPose().pose.orientation.x;
    goal_pose.orientation.y = group_arm.getCurrentPose().pose.orientation.y;
    goal_pose.orientation.z = group_arm.getCurrentPose().pose.orientation.z;
    
    group_arm.setStartStateToCurrentState();
    group_arm.setPoseTarget(goal_pose);

    success = group_arm.plan(my_plan);

    tsupport::basic::print_magenta ("plan 1 (pose goal) :");
    
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

if(type_of_goal == 2)
{
    std::vector<double> start_joint_values;
    group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);

    double tran_j1, tran_j2, tran_j3, tran_j4, tran_j5, tran_j6;
    nh.getParam("/goal_1/move/j1",tran_j1);
    nh.getParam("/goal_1/move/j2",tran_j2);
    nh.getParam("/goal_1/move/j3",tran_j3);
    nh.getParam("/goal_1/move/j4",tran_j4);
    nh.getParam("/goal_1/move/j5",tran_j5);
    nh.getParam("/goal_1/move/j6",tran_j6);
    
    std::vector<double> goal_joint = {0, 0, 0, 0, 0, 0};  
    goal_joint[0] = start_joint_values[0]+tran_j1;
    goal_joint[1] = start_joint_values[1]+tran_j2;
    goal_joint[2] = start_joint_values[2]+tran_j3;
    goal_joint[3] = start_joint_values[3]+tran_j4;
    goal_joint[4] = start_joint_values[4]+tran_j5;
    goal_joint[5] = start_joint_values[5]+tran_j6;

//     goal_joint[0] = tran_j1;
//     goal_joint[1] = tran_j2;
//     goal_joint[2] = tran_j3;
//     goal_joint[3] = tran_j4;
//     goal_joint[4] = tran_j5;
//     goal_joint[5] = tran_j6;
    
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

///////////////////////////////////////////////////////////
///                        2                          ////
/////////////////////////////////////////////////////////

if (!nh.getParam("/goal_2/type_of_goal",type_of_goal))
{
    tsupport::basic::print_green (" End of Goals ");
    cout << "\n" << endl;
    return 0;
}

cout << "type of goal : "  << type_of_goal << endl;

if(type_of_goal == 1)
{

    double tran_x, tran_y, tran_z;
    nh.getParam("/goal_2/move/x",tran_x);
    nh.getParam("/goal_2/move/y",tran_y);
    nh.getParam("/goal_2/move/z",tran_z);
    
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = group_arm.getCurrentPose().pose.position.x + tran_x;
    goal_pose.position.y = group_arm.getCurrentPose().pose.position.y + tran_y;
    goal_pose.position.z = group_arm.getCurrentPose().pose.position.z + tran_z;
    goal_pose.orientation.w = group_arm.getCurrentPose().pose.orientation.w;
    goal_pose.orientation.x = group_arm.getCurrentPose().pose.orientation.x;
    goal_pose.orientation.y = group_arm.getCurrentPose().pose.orientation.y;
    goal_pose.orientation.z = group_arm.getCurrentPose().pose.orientation.z;
    
    group_arm.setStartStateToCurrentState();
    group_arm.setPoseTarget(goal_pose);

    success = group_arm.plan(my_plan);

    tsupport::basic::print_magenta ("plan 2 (pose goal) :");
    
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

if(type_of_goal == 2)
{
    std::vector<double> start_joint_values;
    group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);

    double tran_j1, tran_j2, tran_j3, tran_j4, tran_j5, tran_j6;
    nh.getParam("/goal_2/move/j1",tran_j1);
    nh.getParam("/goal_2/move/j2",tran_j2);
    nh.getParam("/goal_2/move/j3",tran_j3);
    nh.getParam("/goal_2/move/j4",tran_j4);
    nh.getParam("/goal_2/move/j5",tran_j5);
    nh.getParam("/goal_2/move/j6",tran_j6);
    
    std::vector<double> goal_joint = {0, 0, 0, 0, 0, 0};  
    goal_joint[0] = start_joint_values[0]+tran_j1;
    goal_joint[1] = start_joint_values[1]+tran_j2;
    goal_joint[2] = start_joint_values[2]+tran_j3;
    goal_joint[3] = start_joint_values[3]+tran_j4;
    goal_joint[4] = start_joint_values[4]+tran_j5;
    goal_joint[5] = start_joint_values[5]+tran_j6;

    group_arm.setStartStateToCurrentState();
    group_arm.setJointValueTarget(goal_joint); 
    success = group_arm.plan(my_plan);

    tsupport::basic::print_magenta ("plan 2 (joint space goal) : ");
    
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
}
