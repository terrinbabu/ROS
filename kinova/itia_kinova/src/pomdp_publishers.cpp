#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

#include <std_msgs/Float32MultiArray.h>
 
#include <itia_human_prediction/PoseArrays.h>

using namespace std;

int main(int argc, char **argv)
{
  
  itia::support::printString("intializing");
    
  ros::init(argc, argv, "pomdp_publishers");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(1.0);

  
  
  
  itia::support::printString("Publisher-1 :  objects poses");

  geometry_msgs::PoseArray object_poses;
  
// //   object_poses.poses.clear();
// //   object_poses.header.stamp = ros::Time::now();
// //   object_poses.header.frame_id = "/root";
  
  geometry_msgs::Pose object_pose1;
  std::vector<double> obj1_pose = { 0.20,-0.365,0.08}; //z = 0.08
  std::vector<double> obj1_orient = {1, 0, 0, 0};
  std::vector<double> obj1_dim = {0.16,0.04};
    
  geometry_msgs::Pose object_pose2;
  std::vector<double> obj2_pose = { 0.40,-0.365,0.08}; //z = 0.08
  std::vector<double> obj2_orient = {1, 0, 0, 0};
  std::vector<double> obj2_dim = {0.16,0.04};
    
  geometry_msgs::Pose object_pose3;
  std::vector<double> obj3_pose = { 0.60,-0.365,0.08}; //z = 0.08
  std::vector<double> obj3_orient = {1, 0, 0, 0};
  std::vector<double> obj3_dim = {0.16,0.04};
  
  itia::support::fromVecToPose(obj1_pose,
                               obj1_orient,
                               object_pose1);

  itia::support::fromVecToPose(obj2_pose,
                               obj2_orient,
                               object_pose2);
  
  itia::support::fromVecToPose(obj3_pose,
                               obj3_orient,
                               object_pose3);
    
  object_poses.poses.push_back(object_pose1);
  object_poses.poses.push_back(object_pose2);
  object_poses.poses.push_back(object_pose3);
  
  ROS_INFO("%s object poses [Array]",BOLDYELLOW);
  std::cout << object_poses << std::endl ; 
  
  
  
  itia::support::printString("Publisher-2 :  Objects tsrs");

  geometry_msgs::PoseArray object1_tsr;
  itia::grasp_utils::simple_cyl_grasp_tsr(object1_tsr,obj1_pose,obj1_dim);
  geometry_msgs::PoseArray object2_tsr;
  itia::grasp_utils::simple_cyl_grasp_tsr(object2_tsr,obj2_pose,obj2_dim);
  geometry_msgs::PoseArray object3_tsr;
  itia::grasp_utils::simple_cyl_grasp_tsr(object3_tsr,obj3_pose,obj3_dim);
  itia_human_prediction::PoseArrays all_tsr;

  all_tsr.poses.push_back(object1_tsr);
  all_tsr.poses.push_back(object2_tsr);
  all_tsr.poses.push_back(object2_tsr);
  
  ROS_INFO("%s objects TSRs [ Multi Arrays ]",BOLDYELLOW);

// // //   std::cout << all_tsr << std::endl ;


// // //   geometry_msgs::PoseArray object1_tsr;
// // //   
// // // // //   object1_tsr.poses.clear();
// // // // //   object1_tsr.header.stamp = ros::Time::now();
// // // // //   object1_tsr.header.frame_id = "/root";  
// // // 
// // //   geometry_msgs::PoseArray object2_tsr;
// // //   
// // // // //   object2_tsr.poses.clear();
// // // // //   object2_tsr.header.stamp = ros::Time::now();
// // // // //   object2_tsr.header.frame_id = "/root";
// // //   
// // //   geometry_msgs::Pose tsr1;
// // //   geometry_msgs::Pose tsr2;
// // //   
// // //   ROS_INFO(" ");
// // //   ROS_INFO("%s Intializing grasping", BOLDCYAN);
// // // 
// // //   ros::NodeHandle nh_("~");
// // //   
// // //   moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;  // Grasp generator
// // //   moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;   // class for publishing stuff to rvi
// // //   moveit_simple_grasps::GraspData grasp_data_;  // robot-specific data for generating grasps
// // //   std::string ee_group_name_;
// // // 
// // //   nh_.param("ee_group_name", ee_group_name_, std::string("gripper"));
// // //   
// // //   if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))     // Load grasp data specific to our robot
// // //     ros::shutdown();
// // // 
// // //   simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );     // Load grasp generator
// // //   
// // //   ROS_INFO(" ");
// // //   ROS_INFO("%s Generating Grasps for object 1", BOLDCYAN);
// // //   
// // //   std::vector<moveit_msgs::Grasp> possible_grasps;
// // //   possible_grasps.clear();
// // // 
// // //   simple_grasps_->generateBlockGrasps( object_pose1, grasp_data_, possible_grasps);    // Generate set of grasps for the object
// // // 
// // //   for (int x=0; x<possible_grasps.size(); x++)
// // //   {
// // //     if ( possible_grasps[x].grasp_quality ==1)
// // //       {
// // //       tsr1.position.x = possible_grasps[x].grasp_pose.pose.position.x;
// // //       tsr1.position.y = possible_grasps[x].grasp_pose.pose.position.y;
// // //       tsr1.position.z = possible_grasps[x].grasp_pose.pose.position.z;
// // // 
// // //       tsr1.orientation.x = possible_grasps[x].grasp_pose.pose.orientation.x;
// // //       tsr1.orientation.y = possible_grasps[x].grasp_pose.pose.orientation.y;
// // //       tsr1.orientation.z = possible_grasps[x].grasp_pose.pose.orientation.z;
// // //       tsr1.orientation.w = possible_grasps[x].grasp_pose.pose.orientation.w;
// // //       
// // //       object1_tsr.poses.push_back(tsr1);
// // //       
// // //       ros::Duration(0.25).sleep();
// // //       }
// // //   }
// // // 
// // //   ROS_INFO(" ");
// // //   ROS_INFO("%s Generating Grasps for object 2", BOLDCYAN);
// // //   
// // //   possible_grasps.clear();
// // // 
// // //   simple_grasps_->generateBlockGrasps( object_pose2, grasp_data_, possible_grasps);    // Generate set of grasps for the object
// // //    
// // // 
// // //   for (int x=0; x<possible_grasps.size(); x++)
// // //   {
// // //     if ( possible_grasps[x].grasp_quality ==1)
// // //       {
// // //       tsr2.position.x = possible_grasps[x].grasp_pose.pose.position.x;
// // //       tsr2.position.y = possible_grasps[x].grasp_pose.pose.position.y;
// // //       tsr2.position.z = possible_grasps[x].grasp_pose.pose.position.z;
// // // 
// // //       tsr2.orientation.x = possible_grasps[x].grasp_pose.pose.orientation.x;
// // //       tsr2.orientation.y = possible_grasps[x].grasp_pose.pose.orientation.y;
// // //       tsr2.orientation.z = possible_grasps[x].grasp_pose.pose.orientation.z;
// // //       tsr2.orientation.w = possible_grasps[x].grasp_pose.pose.orientation.w;
// // //       
// // //       object2_tsr.poses.push_back(tsr2);
// // // 
// // //       ros::Duration(0.25).sleep();
// // //       }
// // //   }
// // //  
// // //  itia_human_prediction::PoseArrays all_tsr;
// // //  
// // //  all_tsr.poses.push_back(object1_tsr);
// // //  all_tsr.poses.push_back(object2_tsr);
// // //   
// // //  ROS_INFO("%s objects TSRs [ Multi Arrays ]",BOLDYELLOW);
// // //  
// // //  std::cout << all_tsr << std::endl ;
 
  
// //   moveit_simple_grasps::SimpleGraspsPtr simple_grasps_; 
// //   moveit_simple_grasps::GraspData grasp_data_;
// //   std::vector<moveit_msgs::Grasp> possible_grasps;
// //   moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
// //   const std::string hand_planning_group_name = "gripper";
// //   
// //   itia::moveit_utils::initVisualTool(visual_tools_);
// //   
// //   itia::grasp_utils::initGrasp(visual_tools_, simple_grasps_);
// //   
// //   
// //   geometry_msgs::PoseArray object1_tsr;
// //   
// //   object1_tsr.poses.clear();
// //   object1_tsr.header.stamp = ros::Time::now();
// //   object1_tsr.header.frame_id = "/root";
// //   
// //   geometry_msgs::Pose tsr1;
// //   
// //   itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
// //                                       simple_grasps_, visual_tools_, 
// //                                       grasp_data_, obj1_pose, obj1_orient, 
// //                                       possible_grasps);
// //   
// //   std::vector<moveit_msgs::Grasp> sorted_possible_grasps;
// //   itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps);
// //   
// //     for( int x = 0; x< 5; x++ )
// //   {
// //     std::cout << sorted_possible_grasps[x] << std::endl ; 
// //     tsr1 = sorted_possible_grasps[x].grasp_pose.pose;
// //     object1_tsr.poses.push_back(tsr1);
// //   }
// //   
// //   geometry_msgs::PoseArray object2_tsr;
// //   
// //   object2_tsr.poses.clear();
// //   object2_tsr.header.stamp = ros::Time::now();
// //   object2_tsr.header.frame_id = "/root";
// //   
// //   geometry_msgs::Pose tsr2;
// //     
// //   itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
// //                                     simple_grasps_, visual_tools_, 
// //                                     grasp_data_, obj2_pose, obj2_orient, 
// //                                     possible_grasps);
// //   
// //   itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps);
// //   
// //     for( int x = 0; x< 5; x++ )
// //   {
// //     std::cout << sorted_possible_grasps[x] << std::endl ; 
// //     tsr2 = sorted_possible_grasps[x].grasp_pose.pose;
// //     object2_tsr.poses.push_back(tsr2);
// //   }
// // 
// //   itia_human_prediction::PoseArrays all_tsr;
// //   
// //   all_tsr.poses.push_back(object1_tsr);
// //   all_tsr.poses.push_back(object2_tsr);
    
// //   ROS_INFO("%s objects TSRs [ Multi Arrays ]",BOLDYELLOW);
// //   std::cout << all_tsr << std::endl ; 
  
  
  

  
  itia::support::printString("Publisher-3 :  Human goal probability");
  
  vector<double> vec1 = { 0.9, 0.05 , 0.01, 0.05};
  std_msgs::Float32MultiArray goal_prob;

  goal_prob.layout.dim.push_back(std_msgs::MultiArrayDimension());
  goal_prob.layout.dim[0].size = vec1.size();
  goal_prob.layout.dim[0].stride = 1;
  goal_prob.layout.dim[0].label = "Human goal probability";

  goal_prob.data.clear();
  goal_prob.data.insert(goal_prob.data.end(), vec1.begin(), vec1.end());

  ROS_INFO("%s Human goal probability on the objects [Array]",BOLDYELLOW);
  std::cout << goal_prob << std::endl ;
  
  
  
  itia::support::printString("Publishing 2 topics");
  
   ros::Publisher obj_pos_pub = node_handle.advertise<geometry_msgs::PoseArray>("/env_obj/pos", 1000);
//   ros::Publisher tsr_pub = node_handle.advertise<itia_human_prediction::PoseArrays>("/env_obj/tsr", 1000);
   ros::Publisher goal_prob_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/skel/prob_goal_user", 1000); 
  
  ros::Rate loop_rate(100);
  
  while (ros::ok())
  {
    
//    obj_pos_pub.publish(object_poses); 
//    tsr_pub.publish(all_tsr);
     goal_prob_pub.publish(goal_prob);

//     std::cout << goal_prob << std::endl ;
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  itia::support::end();
  
  return 0;
}

