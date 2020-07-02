#ifndef __ITIA_GRASP_UTILS___H__
#define __ITIA_GRASP_UTILS___H__

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

#include <itia_kinova/support.h>


namespace itia 
{  
  
namespace grasp_utils 
{  
  

void initGrasp(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools_,
               moveit_simple_grasps::SimpleGraspsPtr& simple_grasps_)
{ 
  ROS_INFO("%s Intializing simple grasps", BOLDCYAN);
  simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) ); // Load grasp generator
}

void generatedGrasps(const std::string& hand_planning_group_name,
                     moveit_simple_grasps::SimpleGraspsPtr simple_grasps_,  
                     moveit_visual_tools::MoveItVisualToolsPtr visual_tools_,   
                     moveit_simple_grasps::GraspData& grasp_data_,                     
                     const std::vector<double>& pose, 
                     const std::vector<double>& orient,
                     std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  //to be used in case of more end_effectors
  //ros::NodeHandle nh("~");
  //std::string ee_group_name_;
  //nh.param("ee_group_name", ee_group_name_, std::string("j2n6a300_end_effector"));
  
  ROS_INFO("%s Generating grasps", BOLDCYAN);
    
  ros::NodeHandle nh_local("~"); 
  
  if (!grasp_data_.loadRobotGraspData(nh_local, hand_planning_group_name))     // Load grasp data specific to our robot
  {
    ROS_ERROR("Load grasp data failed");
    ros::shutdown();
  }
    
  ROS_INFO(" ");
  ROS_INFO("%s Generating Grasps ", BOLDCYAN); 
  possible_grasps.clear();

  geometry_msgs::Pose bottle_pose;
  itia::support::fromVecToPose(pose,orient,bottle_pose);
  simple_grasps_->generateBlockGrasps( bottle_pose, grasp_data_, possible_grasps);    // Generate set of grasps for the object
}


void visualizeAllGrasps ( const moveit_simple_grasps::SimpleGraspsPtr& simple_grasps_,  
                          moveit_visual_tools::MoveItVisualToolsPtr visual_tools_,   
                          moveit_simple_grasps::GraspData grasp_data_,
                          const std::vector<moveit_msgs::Grasp>& possible_grasps,
                          const std::string& hand_planning_group_name)
{
    ROS_INFO("%s Visualizing all grasps", BOLDCYAN);
    //TODO: this function only shows 1 marker per time. to be changed!
    for (int i=0; i<possible_grasps.size(); i++)
    {   
      geometry_msgs::Pose all_pose;
      visual_tools_->generateEmptyPose(all_pose);
      all_pose.position.x = possible_grasps[i].grasp_pose.pose.position.x;
      all_pose.position.y = possible_grasps[i].grasp_pose.pose.position.y;
      all_pose.position.z = possible_grasps[i].grasp_pose.pose.position.z;

      all_pose.orientation.w = possible_grasps[i].grasp_pose.pose.orientation.w;
      all_pose.orientation.x = possible_grasps[i].grasp_pose.pose.orientation.x;
      all_pose.orientation.y = possible_grasps[i].grasp_pose.pose.orientation.y;
      all_pose.orientation.z = possible_grasps[i].grasp_pose.pose.orientation.z;

      grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );         
      const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(hand_planning_group_name);      
      visual_tools_->publishEEMarkers(all_pose, ee_jmg, rviz_visual_tools::ORANGE, hand_planning_group_name);      
      //visual_tools_->triggerBatchPublish( );
      visual_tools_->trigger();
      ros::Duration(0.1).sleep();
    }    
}

void visualizeGrasp (const moveit_simple_grasps::SimpleGraspsPtr& simple_grasps_,  
                     moveit_visual_tools::MoveItVisualToolsPtr visual_tools_,   
                     moveit_simple_grasps::GraspData grasp_data_,
                     const moveit_msgs::Grasp& possible_grasp,
                     const std::string& hand_planning_group_name)
{
    ROS_INFO("%s Visualizing the grasp", BOLDCYAN);
    geometry_msgs::Pose all_pose;
    visual_tools_->generateEmptyPose(all_pose);
  
    all_pose.position.x = possible_grasp.grasp_pose.pose.position.x;
    all_pose.position.y = possible_grasp.grasp_pose.pose.position.y;
    all_pose.position.z = possible_grasp.grasp_pose.pose.position.z;

    all_pose.orientation.w = possible_grasp.grasp_pose.pose.orientation.w;
    all_pose.orientation.x = possible_grasp.grasp_pose.pose.orientation.x;
    all_pose.orientation.y = possible_grasp.grasp_pose.pose.orientation.y;
    all_pose.orientation.z = possible_grasp.grasp_pose.pose.orientation.z;

    grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );         
    const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(hand_planning_group_name);      
    visual_tools_->publishEEMarkers(all_pose, ee_jmg, rviz_visual_tools::ORANGE, hand_planning_group_name);
    ros::Duration(0.25).sleep();  
    visual_tools_->trigger();
    //visual_tools_->triggerBatchPublish( );
    ros::Duration(0.25).sleep();
}


void visualizeGrasps(moveit_simple_grasps::SimpleGraspsPtr simple_grasps_,  
                     moveit_visual_tools::MoveItVisualToolsPtr visual_tools_,   
                     moveit_simple_grasps::GraspData grasp_data_,
                     std::vector<moveit_msgs::Grasp> possible_grasps,
                     const std::string& end_effector_name )
{
   ROS_INFO("%s Visualize the grasps", BOLDCYAN);       
  for (int x=0; x<possible_grasps.size(); x++)
    visualizeGrasp (simple_grasps_,  
                    visual_tools_, 
                    grasp_data_,
                    possible_grasps[x],
                    end_effector_name);
}

void idMaxQualityGrasp(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                       bool printResult,
                       std::vector<moveit_msgs::Grasp>& sorted_possible_grasps)
{
   ROS_INFO("%s Maximum Quality", BOLDCYAN);
  sorted_possible_grasps = possible_grasps;
  std::sort( sorted_possible_grasps.begin(), sorted_possible_grasps.end(), []( moveit_msgs::Grasp& i1, moveit_msgs::Grasp& i2 ) { return i1.grasp_quality < i2.grasp_quality; } );
  std::reverse( sorted_possible_grasps.begin(), sorted_possible_grasps.end() );
  double max_quality = sorted_possible_grasps.front().grasp_quality;
  
  if(printResult)
  {
    ROS_INFO("Best grasp quality value is %f", max_quality);
    ROS_INFO("Position x: %f", sorted_possible_grasps[0].grasp_pose.pose.position.x);
    ROS_INFO("Position y: %f", sorted_possible_grasps[0].grasp_pose.pose.position.y);
    ROS_INFO("Position z: %f", sorted_possible_grasps[0].grasp_pose.pose.position.z);
    ROS_INFO("Orientatio w: %f", sorted_possible_grasps[0].grasp_pose.pose.orientation.w);
    ROS_INFO("Orientatio x: %f", sorted_possible_grasps[0].grasp_pose.pose.orientation.x);
    ROS_INFO("Orientatio y: %f", sorted_possible_grasps[0].grasp_pose.pose.orientation.y);
    ROS_INFO("Orientatio z: %f", sorted_possible_grasps[0].grasp_pose.pose.orientation.z);
    ROS_INFO("Sorted Quality: %f", sorted_possible_grasps[0].grasp_quality);
  }

}

void simple_box_grasp_tsr (geometry_msgs::PoseArray& object_tsr,
                           const std::vector<double>& object_pos,
                           const std::vector<double>& object_dim )
{
  geometry_msgs::Pose tsr;
  
  tsr.position.x = object_pos[0] - object_dim[0]/4;
  tsr.position.y = object_pos[1];
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0;
  tsr.orientation.y =  0.707;
  tsr.orientation.z =  0;
  tsr.orientation.w =  0.707;
  
  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0];
  tsr.position.y = object_pos[1] + object_dim[1]/4;
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0.5;
  tsr.orientation.y =  0.5;
  tsr.orientation.z = -0.5;
  tsr.orientation.w =  0.5;
  
  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0];
  tsr.position.y = object_pos[1] - object_dim[1]/4;
  tsr.position.z = object_pos[2];

  tsr.orientation.x = -0.5;
  tsr.orientation.y =  0.5;
  tsr.orientation.z =  0.5;
  tsr.orientation.w =  0.5;

  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0] + object_dim[0]/4;
  tsr.position.y = object_pos[1];
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0.707;
  tsr.orientation.y =  0;
  tsr.orientation.z = -0.707;
  tsr.orientation.w =  0;
  
  object_tsr.poses.push_back(tsr);
  
// //   tsr.position.x = object_pos[0];
// //   tsr.position.y = object_pos[1];
// //   tsr.position.z = object_pos[2] + object_dim[2]/2;
// //   
// //   tsr.orientation.x =  1;
// //   tsr.orientation.y =  0;
// //   tsr.orientation.z =  0;
// //   tsr.orientation.w =  0;
// //     
// //   object_tsr.poses.push_back(tsr);
}

void simple_cyl_grasp_tsr (geometry_msgs::PoseArray& object_tsr,
                           const std::vector<double>& object_pos,
                           const std::vector<double>& object_dim )
{
  geometry_msgs::Pose tsr;
  
  tsr.position.x = object_pos[0] - object_dim[1]/2;
  tsr.position.y = object_pos[1];
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0;
  tsr.orientation.y =  0.707;
  tsr.orientation.z =  0;
  tsr.orientation.w =  0.707;
  
  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0];
  tsr.position.y = object_pos[1] + object_dim[1]/2;
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0.5;
  tsr.orientation.y =  0.5;
  tsr.orientation.z = -0.5;
  tsr.orientation.w =  0.5;
  
  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0];
  tsr.position.y = object_pos[1] - object_dim[1]/2;
  tsr.position.z = object_pos[2];

  tsr.orientation.x = -0.5;
  tsr.orientation.y =  0.5;
  tsr.orientation.z =  0.5;
  tsr.orientation.w =  0.5;

  object_tsr.poses.push_back(tsr);
  
  tsr.position.x = object_pos[0] + object_dim[0]/2;
  tsr.position.y = object_pos[1];
  tsr.position.z = object_pos[2];
  
  tsr.orientation.x =  0.707;
  tsr.orientation.y =  0;
  tsr.orientation.z = -0.707;
  tsr.orientation.w =  0;
  
  object_tsr.poses.push_back(tsr);
  
// //   tsr.position.x = object_pos[0];
// //   tsr.position.y = object_pos[1];
// //   tsr.position.z = object_pos[2] + object_dim[0]/2;
// //   
// //   tsr.orientation.x =  1;
// //   tsr.orientation.y =  0;
// //   tsr.orientation.z =  0;
// //   tsr.orientation.w =  0;
// //     
// //   object_tsr.poses.push_back(tsr);
}

}
}

#endif