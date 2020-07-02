#include <iostream>
#include <string>
#include <math.h>
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

#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;
using namespace ros;

typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client ;

int main(int argc, char **argv)
{
    init(argc, argv,"sinusoidal_action");
    NodeHandle nh;
    AsyncSpinner spinner(1);
    spinner.start(); 
    sleep(3.0);
    
    const string arm_planning_group_name = "manipulator";
    const string end_effector_name = "ee_link";

    moveit::planning_interface::MoveGroupInterface group_arm(arm_planning_group_name);
    
    std::vector<double> start_joint_values;
    group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
    
    double time = 10;
    bool read_time =nh.getParam("/sinusoidal_action/time",time);
    
    double time_2_start_val = 0;
    double time_2_start_increment = 0.2;
    
    int points_num = (int) (time/time_2_start_increment);
    
    control_msgs::FollowJointTrajectoryGoal goal;
    
    goal.trajectory.header.stamp=ros::Time::now();
    goal.trajectory.header.frame_id= "world";
    goal.trajectory.joint_names = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};  // note the diffrence in joint name     list - elbow joint 1st and sholder pan joint 3rd
    
    goal.trajectory.points.resize(points_num);
    
    std::vector<double> zeros = {0,0,0,0,0,0};
    
    std::vector<double> position_0 = {start_joint_values[2] ,
                                      start_joint_values[1] ,
                                      start_joint_values[0] -0.05, 
                                      start_joint_values[3] , 
                                      start_joint_values[4] ,
                                      start_joint_values[5] };
                                      
    std::vector<double> position_1 = {start_joint_values[2],
                                      start_joint_values[1], 
                                      start_joint_values[0], 
                                      start_joint_values[3],
                                      start_joint_values[4],
                                      start_joint_values[5]};

    for (int i=0;i<points_num;i++)
    {

        goal.trajectory.points.at(i).time_from_start = ros::Duration(time_2_start_val);
        time_2_start_val = time_2_start_val + time_2_start_increment;
        
        if (i%2 == 0)
            goal.trajectory.points.at(i).positions = position_0;
        else
            goal.trajectory.points.at(i).positions = position_1;
        
        goal.trajectory.points.at(i).velocities = zeros;
        goal.trajectory.points.at(i).accelerations = zeros;

    }

    Client client("/follow_joint_trajectory", true);
    
    client.waitForServer();
    client.sendGoal(goal);  
}
