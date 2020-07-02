#include <iostream>
#include <chrono> // clock
// ros libraries

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>

//  moveit libraries

#include <moveit/move_group_interface/move_group_interface.h> // all moveit operations
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>

// messages

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h> // all moveit_operations
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <std_msgs/Float32MultiArray.h>
 #include "std_msgs/Bool.h"
 
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/PoseVelocity.h>

#include "tf2_msgs/TFMessage.h"

#include <itia_human_prediction/PoseArrays.h>

#include <apriltags/AprilTagDetections.h>

#include <sound_play/SoundRequest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <moveit_visual_tools/moveit_visual_tools.h>

// include files

#include <itia_futils/itia_futils.h> // BOLDCYAN etc
#include <itia_tutils/itia_tutils.h> // std::string
#include <itia_rutils/itia_rutils.h> // message_reciever

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>
#include <itia_kinova/itia_grasp_utils.h>

#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/custom_environment2.h>
#include <moveit_simple_grasps/grasp_data.h>

typedef  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> Client ;

int main(int argc, char **argv)
{
  
 
    
  ros::init(argc, argv,"trial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start(); 
  sleep(10.0);
  
  ROS_INFO("%s Group Info ",BOLDYELLOW);
   
  const std::string arm_planning_group_name = "arm";
  const std::string hand_planning_group_name = "gripper";
  const std::string end_effector_name = "j2n6s300_end_effector";
    
  moveit::planning_interface::MoveGroupInterface group_arm(arm_planning_group_name);
  moveit::planning_interface::MoveGroupInterface group_hand(hand_planning_group_name);
  
  ROS_INFO ( "%s Reference frame of the group -  %s : %s", BOLDCYAN ,arm_planning_group_name.c_str(), group_arm.getPlanningFrame().c_str() );
  
  ROS_INFO ( "%s Current Position of the group - %s : [%f %f %f] ", BOLDCYAN ,arm_planning_group_name.c_str(), 
                                                                              group_arm.getCurrentPose().pose.position.x,
                                                                              group_arm.getCurrentPose().pose.position.y,
                                                                              group_arm.getCurrentPose().pose.position.z );
  
  ROS_INFO ( "%s Current Orientation of the group - %s : [%f %f %f %f] ", BOLDCYAN ,arm_planning_group_name.c_str(), 
                                                                                    group_arm.getCurrentPose().pose.orientation.w,
                                                                                    group_arm.getCurrentPose().pose.orientation.x,
                                                                                    group_arm.getCurrentPose().pose.orientation.y,
                                                                                    group_arm.getCurrentPose().pose.orientation.z );
  std::vector<double> current_rotational_joint_values; 
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), current_rotational_joint_values);
  itia::support::printvectordouble("current_rotational_joint_values",current_rotational_joint_values);
  
  // bool success;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  geometry_msgs::Pose goal_pose;
  
/////////////////////////////////////////////////////////////////////////////////
/////                                 1                                  ///////
///////////////////////////////////////////////////////////////////////////////
  
  ROS_INFO("%s Planning to pose goal ",BOLDYELLOW);
   
  goal_pose.position.x = group_arm.getCurrentPose().pose.position.x + 0.1;
  goal_pose.position.y = group_arm.getCurrentPose().pose.position.y + 0.2;
  goal_pose.position.z = group_arm.getCurrentPose().pose.position.z + 0.1;
            
  goal_pose.orientation.w = group_arm.getCurrentPose().pose.orientation.w;
  goal_pose.orientation.x = group_arm.getCurrentPose().pose.orientation.x;
  goal_pose.orientation.y = group_arm.getCurrentPose().pose.orientation.y;
  goal_pose.orientation.z = group_arm.getCurrentPose().pose.orientation.z;
  
  group_arm.setStartStateToCurrentState();
  group_arm.setPoseTarget(goal_pose);

  moveit::planning_interface::MoveItErrorCode success = group_arm.plan(my_plan);

  ROS_INFO("%s plan 1 (pose goal) : %s",BOLDGREEN, success?"SUCCESS":"FAILED");

  ROS_INFO("%s Visualizing plan 1 ",BOLDCYAN);
  moveit_msgs::DisplayTrajectory display_trajectory_1;
  display_trajectory_1.trajectory_start = my_plan.start_state_;
  display_trajectory_1.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_1);
  sleep(3.0);
  
  if(itia::support::check())
  {
  ROS_INFO("%s Executing plan 1 ",BOLDCYAN);
  group_arm.execute(my_plan);
  }

  
  
/////////////////////////////////////////////////////////////////////////////////
/////                                 2                                  ///////
///////////////////////////////////////////////////////////////////////////////
  
  itia::support::check();
  
  std::vector<double> start_joint_values;
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
  
  std::vector<double> goal_joint = {0, 0, 0, 0, 0, 0};  
  
  goal_joint[0] = 635.5 * M_PI / 180;
  goal_joint[1] = 167.4 * M_PI / 180;
  goal_joint[2] = 57.4  * M_PI / 180;
  goal_joint[3] = 240.4 * M_PI / 180;
  goal_joint[4] = 83  * M_PI / 180;
  goal_joint[5] = 435.4 * M_PI / 180;
  
  group_arm.setStartStateToCurrentState();
  group_arm.setJointValueTarget(goal_joint); 
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan to home : %s",BOLDGREEN, success?"SUCCESS":"FAILED");
  
  moveit_msgs::DisplayTrajectory display_trajectory_02;
  display_trajectory_02.trajectory_start = my_plan.start_state_;
  display_trajectory_02.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_02);
  sleep(3.0);
  
  if(itia::support::check())
  {
  group_arm.execute(my_plan);
  }
  
  itia::support::check();
  
  ROS_INFO("%s Planning to joint space goal ",BOLDYELLOW);
    
   
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
    
  goal_joint[0] = start_joint_values[0]+2;
  goal_joint[1] = start_joint_values[1];
  goal_joint[2] = start_joint_values[2];
  goal_joint[3] = start_joint_values[3];
  goal_joint[4] = start_joint_values[4];
  goal_joint[5] = start_joint_values[5];
  
  group_arm.setStartStateToCurrentState();
  group_arm.setJointValueTarget(goal_joint); 
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan 2 (joint space goal) : %s",BOLDGREEN, success?"SUCCESS":"FAILED");
  
  ROS_INFO("%s Visualizing plan 2 ",BOLDCYAN);
  moveit_msgs::DisplayTrajectory display_trajectory_2;
  display_trajectory_2.trajectory_start = my_plan.start_state_;
  display_trajectory_2.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_2);
  sleep(3.0);
  
  if(itia::support::check())
  {
  ROS_INFO("%s Executing plan 2 ",BOLDCYAN);
  group_arm.execute(my_plan);
  }
  
/////////////////////////////////////////////////////////////////////////////////
/////                                 3                                  ///////
///////////////////////////////////////////////////////////////////////////////
  
  itia::support::check();
    
  ROS_INFO("%s Forward Kinematics, Inverse kinematics and Jacobian ",BOLDYELLOW);
    
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(arm_planning_group_name);
  
  ROS_INFO("%s Forward Kinematics ",BOLDCYAN);
    
  std::vector<double> current_joint_values; 
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), current_joint_values);
  
  kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_name);
  
  ROS_INFO("Translation ");
  ROS_INFO_STREAM("" << end_effector_state.translation());
  ROS_INFO("Rotation ");
  ROS_INFO_STREAM("" << end_effector_state.rotation());
  
  ROS_INFO("%s Inverse Kinematics ",BOLDCYAN);
  
  geometry_msgs::Pose current_pose;
  current_pose.position.x = group_arm.getCurrentPose().pose.position.x ;
  current_pose.position.y = group_arm.getCurrentPose().pose.position.y ;
  current_pose.position.z = group_arm.getCurrentPose().pose.position.z ;
            
  current_pose.orientation.w = group_arm.getCurrentPose().pose.orientation.w;
  current_pose.orientation.x = group_arm.getCurrentPose().pose.orientation.x;
  current_pose.orientation.y = group_arm.getCurrentPose().pose.orientation.y;
  current_pose.orientation.z = group_arm.getCurrentPose().pose.orientation.z;
  
  bool found_ik;
  found_ik = kinematic_state->setFromIK(joint_model_group, current_pose, 20, 0.5);

  std::vector<double> joint_value;
  
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_value);
    itia::support::printvectordouble("current_joint_value",joint_value);
  }  
  else
    ROS_INFO("%s Did not find IK solution", BOLDRED );

  ROS_INFO("%s Jacobian ",BOLDCYAN);
  
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position,
                               jacobian);
  
  ROS_INFO_STREAM("" << jacobian);

/////////////////////////////////////////////////////////////////////////////////
/////                                 4                                  ///////
///////////////////////////////////////////////////////////////////////////////
  
  itia::support::check();
  
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
    
  goal_joint[0] = 635.5 * M_PI / 180;
  goal_joint[1] = 167.4 * M_PI / 180;
  goal_joint[2] = 57.4  * M_PI / 180;
  goal_joint[3] = 240.4 * M_PI / 180;
  goal_joint[4] = 83  * M_PI / 180;
  goal_joint[5] = 435.4 * M_PI / 180;
  
  group_arm.setStartStateToCurrentState();
  group_arm.setJointValueTarget(goal_joint); 
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan to home : %s",BOLDGREEN, success?"SUCCESS":"FAILED");
  
  moveit_msgs::DisplayTrajectory display_trajectory_04;
  display_trajectory_04.trajectory_start = my_plan.start_state_;
  display_trajectory_04.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_04);
  sleep(3.0);
  
  if(itia::support::check())
  {
  group_arm.execute(my_plan);
  }
  
  itia::support::check();
    
  ROS_INFO("%sObstacle Avoidance", BOLDCYAN);
  
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  sleep(0.5);
  
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "root";     
  attached_object.object.header.frame_id = "world";
  attached_object.object.id = "bottle";

  moveit_msgs::PlanningScene planning_scene1;

  geometry_msgs::Pose bottle_pose;
  bottle_pose.position.x = group_arm.getCurrentPose().pose.position.x - 0.2;
  bottle_pose.position.y = group_arm.getCurrentPose().pose.position.y ;
  bottle_pose.position.z = group_arm.getCurrentPose().pose.position.z ;  
  bottle_pose.orientation.w = 1;
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.16; //length
  primitive.dimensions[1] = 0.03; //radius

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(bottle_pose);

  attached_object.object.operation = attached_object.object.ADD;
  
    
  planning_scene1.world.collision_objects.push_back(attached_object.object);
  planning_scene1.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene1);
  sleep(2.0);
  
  itia::support::check();
  
  std::vector<double> start_joint_values_2; 
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values_2);
  
  std::vector<double> goal_joint_2 = {0, 0, 0, 0, 0, 0};    
  goal_joint_2[0] = start_joint_values_2[0]+2;
  goal_joint_2[1] = start_joint_values_2[1];
  goal_joint_2[2] = start_joint_values_2[2];
  goal_joint_2[3] = start_joint_values_2[3];
  goal_joint_2[4] = start_joint_values_2[4];
  goal_joint_2[5] = start_joint_values_2[5];
  
  group_arm.setStartStateToCurrentState();
  group_arm.setJointValueTarget(goal_joint_2); 
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan 3 (Obstacle Avoidance) : %s",BOLDGREEN, success?"SUCCESS":"FAILED");
  
  ROS_INFO("%s Visualizing plan 3 ",BOLDCYAN);
  moveit_msgs::DisplayTrajectory display_trajectory_5;
  display_trajectory_5.trajectory_start = my_plan.start_state_;
  display_trajectory_5.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_5);
  sleep(3.0);
  
  if(itia::support::check())
  {
  ROS_INFO("%s Executing plan 3 ",BOLDCYAN);
  group_arm.execute(my_plan);
  }
  
  
/////////////////////////////////////////////////////////////////////////////////
/////                                 5                                  ///////
///////////////////////////////////////////////////////////////////////////////
  
  itia::support::check();
  
  group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), start_joint_values);
    
  goal_joint[0] = 635.5 * M_PI / 180;
  goal_joint[1] = 167.4 * M_PI / 180;
  goal_joint[2] = 57.4  * M_PI / 180;
  goal_joint[3] = 240.4 * M_PI / 180;
  goal_joint[4] = 83  * M_PI / 180;
  goal_joint[5] = 435.4 * M_PI / 180;
  
  group_arm.setStartStateToCurrentState();
  group_arm.setJointValueTarget(goal_joint); 
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan to home : %s",BOLDGREEN, success?"SUCCESS":"FAILED");
  
  moveit_msgs::DisplayTrajectory display_trajectory_03;
  display_trajectory_03.trajectory_start = my_plan.start_state_;
  display_trajectory_03.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_03);
  sleep(3.0);
  
  if(itia::support::check())
  {
  group_arm.execute(my_plan);
  }
  
  itia::support::check();
  
  ROS_INFO("%s Planning with path constraint ",BOLDYELLOW); 
  
  goal_pose.position.x = group_arm.getCurrentPose().pose.position.x;
  goal_pose.position.y = group_arm.getCurrentPose().pose.position.y;
  goal_pose.position.z = group_arm.getCurrentPose().pose.position.z;
  
  goal_pose.orientation.w = 0.016;
  goal_pose.orientation.x = 0.781;
  goal_pose.orientation.y = 0.623;
  goal_pose.orientation.z = -0.0199;
        
  group_arm.setStartStateToCurrentState();
  group_arm.setPoseTarget(goal_pose);
  
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan A : %s",BOLDGREEN, success?"SUCCESS":"FAILED");

  ROS_INFO("%s Visualizing plan A ",BOLDCYAN);
  moveit_msgs::DisplayTrajectory display_trajectory_3;
  display_trajectory_3.trajectory_start = my_plan.start_state_;
  display_trajectory_3.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_3);
  sleep(3.0);
  
  if(itia::support::check())
  {
  ROS_INFO("%s Executing plan A ",BOLDCYAN);
  group_arm.execute(my_plan);
  }
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "j2n6s300_end_effector";
  ocm.header.frame_id = "world";
  ocm.orientation.w = 0.0164;
  ocm.orientation.x = 0.781;
  ocm.orientation.y = 0.623;
  ocm.orientation.z = -0.0199;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group_arm.setPathConstraints(test_constraints);

  ROS_INFO("%s Plan B ",BOLDYELLOW);
    
  goal_pose.position.x = group_arm.getCurrentPose().pose.position.x + 0.2;
  goal_pose.position.y = group_arm.getCurrentPose().pose.position.y;
  goal_pose.position.z = group_arm.getCurrentPose().pose.position.z;
            
  goal_pose.orientation.w = 0.016;
  goal_pose.orientation.x = 0.781;
  goal_pose.orientation.y = 0.623;
  goal_pose.orientation.z = -0.0199;
        
  group_arm.setStartStateToCurrentState();
  group_arm.setPoseTarget(goal_pose);
  
  success = group_arm.plan(my_plan);

  ROS_INFO("%s plan B : %s",BOLDGREEN, success?"SUCCESS":"FAILED");

  ROS_INFO("%s Visualizing plan B ",BOLDCYAN);
  moveit_msgs::DisplayTrajectory display_trajectory_4;
  display_trajectory_4.trajectory_start = my_plan.start_state_;
  display_trajectory_4.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory_4);
  sleep(3.0);
  
  if(itia::support::check())
  {
  ROS_INFO("%s Executing B ",BOLDCYAN);
  group_arm.execute(my_plan);
  }
  
  itia::support::end();
  return 0;
}