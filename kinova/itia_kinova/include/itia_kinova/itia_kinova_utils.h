#ifndef __ITIA_KINOVA_UTILS___H__
#define __ITIA_KINOVA_UTILS___H__

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_simple_grasps/custom_environment2.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_simple_grasps/simple_grasps.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <itia_kinova/support.h>

typedef  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> Client ;
typedef  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Client_fingers ;
typedef  actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> Client_joints ;

namespace itia 
{  

namespace kinova_utils
{  
  
bool closehand(bool fake,bool gazebo)
{
  ROS_INFO("%s closing hand ",GREEN);

  if (fake)   // can be used for all three : gazebo and real robot (but slower and error in creating plan)
      {
          moveit::planning_interface::MoveGroupInterface group_hand("gripper");  //srdf
          
          std::vector<double> start_hand_joint_values;
          group_hand.getCurrentState()->copyJointGroupPositions(group_hand.getCurrentState()->getRobotModel()->getJointModelGroup(group_hand.getName()), start_hand_joint_values);
          
          std::vector<double> hand_goal_joint = {0, 0, 0, 0};    
          hand_goal_joint[0] = start_hand_joint_values[0];
          hand_goal_joint[1] = start_hand_joint_values[1] + 1.3;
          hand_goal_joint[2] = start_hand_joint_values[2] + 1.3;
          hand_goal_joint[3] = start_hand_joint_values[3] + 1.3;
          
          group_hand.setStartStateToCurrentState();
          
          group_hand.setJointValueTarget(hand_goal_joint);
          
          //bool success;
          moveit::planning_interface::MoveGroupInterface::Plan my_hand_plan; 
          moveit::planning_interface::MoveItErrorCode success = group_hand.plan(my_hand_plan);
          
          if (!success)
            ROS_ERROR("Not able to close the hand");
          
          if (success)
            ROS_INFO("%s Success!! Success!! Success!! Success", BOLDGREEN );
          
          group_hand.execute(my_hand_plan);
      }
  else if (gazebo)
      {
        
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.points.resize(1);  
            goal.trajectory.points.at(0).time_from_start = ros::Duration(15);
            goal.trajectory.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
            goal.trajectory.points.at(0).positions = {1.3, 1.3, 1.3}; 
          
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2n6s300/effort_finger_trajectory_controller/follow_joint_trajectory",true);  

            ac.waitForServer(ros::Duration(0.0));
            goal.trajectory.header.stamp=ros::Time::now();
            ac.sendGoal(goal);
            
            ac.waitForResult(ros::Duration(20.0));
          
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Gazebo Action state: %s",state.toString().c_str());

      }
  else
      {
        
            Client_fingers client_fingers("/j2n6s300_driver/fingers_action/finger_positions", true); // true -> don't need ros::spin()
            client_fingers.waitForServer();

            kinova_msgs::SetFingersPositionGoal goal;
            goal.fingers.finger1 = 6780;
            goal.fingers.finger2 = 6756;                        
            goal.fingers.finger3 = 6804;

            client_fingers.sendGoal(goal);
            client_fingers.waitForResult(ros::Duration(5.0));
            if (client_fingers.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("%s Goal Completed", BOLDGREEN );

            std::printf("Current State: %s\n", client_fingers.getState().toString().c_str());
          
      }
}

bool openhand(bool fake,bool gazebo)
{
  ROS_INFO("%s opening hand ",GREEN);
    
  if (fake)   // can be used for all three : gazebo and real robot (but slower and error in creating plan)
      {
          moveit::planning_interface::MoveGroupInterface group_hand("gripper");  //srdf
          
          std::vector<double> start_hand_joint_values; 
          group_hand.getCurrentState()->copyJointGroupPositions(group_hand.getCurrentState()->getRobotModel()->getJointModelGroup(group_hand.getName()), start_hand_joint_values);
          
          std::vector<double> hand_goal_joint = {0, 0, 0, 0};    
          hand_goal_joint[0] = start_hand_joint_values[0];
          hand_goal_joint[1] = 0;
          hand_goal_joint[2] = 0;
          hand_goal_joint[3] = 0;
          
          group_hand.setStartStateToCurrentState();
          
          group_hand.setJointValueTarget(hand_goal_joint);
          
//          bool success;
          moveit::planning_interface::MoveGroupInterface::Plan my_hand_plan; 
          moveit::planning_interface::MoveItErrorCode success = group_hand.plan(my_hand_plan);
          
          if (!success)
            ROS_ERROR("Not able to open the hand");
          
          if (success)
            ROS_INFO("%s Success!! Success!! Success!! Success", BOLDGREEN );
          
          group_hand.execute(my_hand_plan);
      }
  else if (gazebo)
      {
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.points.resize(1);  
            goal.trajectory.points.at(0).time_from_start = ros::Duration(15);
            goal.trajectory.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
            goal.trajectory.points.at(0).positions = {0, 0, 0}; 
          
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2n6s300/effort_finger_trajectory_controller/follow_joint_trajectory",true);  

            ac.waitForServer(ros::Duration(0.0));
            goal.trajectory.header.stamp=ros::Time::now();
            ac.sendGoal(goal);
            
            ac.waitForResult(ros::Duration(20.0));
          
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Gazebo Action state: %s",state.toString().c_str());

      }
  else
      {

            Client_fingers client_fingers("/j2n6s300_driver/fingers_action/finger_positions", true); // true -> don't need ros::spin()
            client_fingers.waitForServer();

            kinova_msgs::SetFingersPositionGoal goal;
            goal.fingers.finger1 = 0;
            goal.fingers.finger2 = 0;                        
            goal.fingers.finger3 = 0;

            client_fingers.sendGoal(goal);
            client_fingers.waitForResult(ros::Duration(5.0));
            if (client_fingers.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("%s Goal Completed", BOLDGREEN );

            std::printf("Current State: %s\n", client_fingers.getState().toString().c_str());
          
      }
}

void active_controller(ros::NodeHandle& nh,
                       const std::string gazebo_param,
                       const std::string fake_param,
                       bool& fake,
                       bool& gazebo)
{
  bool read_gazebo =nh.getParam(gazebo_param,gazebo);
  bool read_fake =nh.getParam(fake_param,fake);
  
  if (fake)
      itia::support::printString("FAKE CONTROLLER IS ACTIVE");
  else if (gazebo)
      itia::support::printString("GAZEBO CONTROLLER IS ACTIVE");
  else
      itia::support::printString("REAL CONTROLLER IS ACTIVE");
}

void gohome()
{
  ROS_INFO("%s going home ",GREEN);
  
  kinova_msgs::ArmPoseGoal goal;

  goal.pose.header.frame_id = "j2n6s300_link_base";
  goal.pose.pose.position.x = 0.212194576859;
  goal.pose.pose.position.y = -0.25703433156;
  goal.pose.pose.position.z = 0.505373299122;
  goal.pose.pose.orientation.w = 0.498322844505;
  goal.pose.pose.orientation.x = -0.503124177456;
  goal.pose.pose.orientation.y = 0.499893963337;
  goal.pose.pose.orientation.z = -0.498644709587;
  
  Client client("/j2n6s300_driver/pose_action/tool_pose", true);
  client.waitForServer();

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  printf("Current State: %s\n", client.getState().toString().c_str());
}

void move(const std::string string,double x, double y, double z, double qw, double qx, double qy, double qz)
{
  ROS_INFO("%s action : %s",GREEN,string.c_str());
  
  kinova_msgs::ArmPoseGoal goal;

  goal.pose.header.frame_id = "j2n6s300_link_base";
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = z;
  goal.pose.pose.orientation.w = qw;
  goal.pose.pose.orientation.x = qx;
  goal.pose.pose.orientation.y = qy;
  goal.pose.pose.orientation.z = qz;
  
  Client client("/j2n6s300_driver/pose_action/tool_pose", true);
  client.waitForServer();

  client.sendGoal(goal);
//   client.waitForResult(ros::Duration(5.0));
// 
//   printf("Current State: %s\n", client.getState().toString().c_str());

}

void move_joint(const std::string string,double j1, double j2, double j3, double j4, double j5, double j6)
{
  ROS_INFO("%s action : %s",GREEN,string.c_str());
  
  Client_joints client_joints("/j2n6s300_driver/joints_action/joint_angles", true);
  client_joints.waitForServer();

  kinova_msgs::ArmJointAnglesGoal goal;

  goal.angles.joint1 = j1 * 180 / M_PI ;
  goal.angles.joint2 = j2 * 180 / M_PI ;
  goal.angles.joint3 = j3 * 180 / M_PI ;
  goal.angles.joint4 = j4 * 180 / M_PI ;
  goal.angles.joint5 = j5 * 180 / M_PI ;
  goal.angles.joint6 = j6 * 180 / M_PI ;

  client_joints.sendGoal(goal);
  client_joints.waitForResult(ros::Duration(5.0));
  
  printf("Current State: %s\n", client_joints.getState().toString().c_str());
}

}}

#endif
