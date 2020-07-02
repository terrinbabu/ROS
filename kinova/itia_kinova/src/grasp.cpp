#include <itia_futils/itia_futils.h> 

#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>
#include <itia_kinova/itia_grasp_utils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10.0);
  
  bool gazebo; bool fake;
  
  itia::kinova_utils::active_controller(nh,"/grasp/gazebo","/grasp/fake",fake,gazebo);

  
  itia::support::printString("Adding the environment");
  
  ROS_INFO("%sAdding the table", BOLDCYAN);
  std::vector<double> table_pose;
  std::vector<double> table_orient;
  std::vector<double> table_dim;
  
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
   sleep(0.5);
  
  if (!nh.getParam("/scene_configuration/table/position",table_pose))
    ROS_ERROR("Error in reading table pose");  
  if (!nh.getParam("/scene_configuration/table/orientation",table_orient))
    ROS_ERROR("Error in reading table orientation");    
  if (!nh.getParam("/scene_configuration/table/dimensions",table_dim))
    ROS_ERROR("Error in reading table dim");
    
//   itia::moveit_utils::addBox(planning_scene_diff_publisher, 
//                         table_pose, 
//                         table_orient,
//                         table_dim,
//                         "root",      
//                         "world",
//                         "table");
  
  ROS_INFO("%sAdding the bottle", BOLDCYAN);
  std::vector<double> box_pose = { 0.7,-0.365,0.08}; 
  std::vector<double> box_orient = {1, 0, 0, 0};
  std::vector<double> box_dim = {0.16,0.03};
  itia::moveit_utils::addCylinder(planning_scene_diff_publisher, 
                        box_pose, 
                        box_orient,
                        box_dim,
                        "root",
                        "world",
                        "bottle");
  
  
  itia::kinova_utils::openhand(fake,gazebo);
    
  itia::support::printString("Grasping analysis");
  
  
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_; 
  moveit_simple_grasps::GraspData grasp_data_;
  std::vector<moveit_msgs::Grasp> possible_grasps;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const std::string end_effector_name = "j2n6s300_end_effector";
  const std::string arm_planning_group_name = "arm";
  const std::string hand_planning_group_name = "gripper";
  
  itia::moveit_utils::initVisualTool(visual_tools_);
  
  itia::grasp_utils::initGrasp(visual_tools_, simple_grasps_);
  
  itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
                                      simple_grasps_, visual_tools_, 
                                      grasp_data_, box_pose, box_orient, 
                                      possible_grasps);
  
  
  std::vector<moveit_msgs::Grasp> sorted_possible_grasps;
  itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps);
  
  moveit::planning_interface::MoveGroupInterface group_arm(arm_planning_group_name);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  int no_of_plans = 50;
  int plan_check = 0;
  
  itia::grasp_utils::visualizeAllGrasps(simple_grasps_, visual_tools_, grasp_data_, sorted_possible_grasps, hand_planning_group_name);
  
  for( int x = 0; x< 5; x++ )
  {  
    itia::grasp_utils::visualizeGrasp(simple_grasps_, visual_tools_, grasp_data_, sorted_possible_grasps[x], hand_planning_group_name);
    geometry_msgs::Pose best_grasp_pose = sorted_possible_grasps[x].grasp_pose.pose;
    
    
    itia::support::printString("Inverse kinematics of selected pose");
    
    std::vector<double> best_grasp_joint;
    
    if (!itia::moveit_utils::invKin(best_grasp_pose,robot_model_loader,arm_planning_group_name,best_grasp_joint))
     continue;
    
    itia::support::printvectordouble("best_grasp_joint",best_grasp_joint);
    
    itia::moveit_utils::allowCollisions(robot_model_loader,"bottle", hand_planning_group_name);
    
    if (itia::moveit_utils::checkCollisions(robot_model_loader,arm_planning_group_name,best_grasp_joint))
      continue;
    
    if(itia::moveit_utils::master_plan(robot_model_loader,arm_planning_group_name,end_effector_name,group_arm,best_grasp_joint,my_plan,no_of_plans))
    {
      plan_check = 1;
      break;
    }
  }    
  
  if (plan_check == 1)
  {
  double my_plan_length,st_line_length;
  
  itia::moveit_utils::plan_trajectory_length(robot_model_loader,arm_planning_group_name,end_effector_name,my_plan,st_line_length,my_plan_length);
  
  ROS_INFO (" %s st. line distance between start and end position : %f",BOLDCYAN,st_line_length);
  ROS_INFO (" %s approx. plan trajectory length : %f",BOLDCYAN,my_plan_length);
  
  itia::moveit_utils::visualize(nh,group_arm,my_plan);
  
  itia::support::printString("Execute");
  group_arm.execute(my_plan);
  
  itia::kinova_utils::closehand(fake,gazebo);
  }
  else
  ROS_ERROR("NO PLAN FOUND");
  
  itia::support::end();
  
  return 0;
}