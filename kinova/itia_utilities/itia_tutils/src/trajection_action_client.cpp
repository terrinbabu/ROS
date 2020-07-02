#include <itia_tutils/itia_tutils.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_client");
  ros::NodeHandle nh;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("follow_joint_trajectory",true);
  
  control_msgs::FollowJointTrajectoryGoal goal;
  
  if (!itia::tutils::getTrajectoryFromParam(nh,"/trajectory",goal.trajectory))
  {
    ROS_ERROR("ERROR LOADING TRAJECTORY");
    return -1;
  }
  
  control_msgs::JointTolerance tol;
  if (!nh.getParam("path_tolerance",tol.position))
    tol.position = 0.001;
  goal.path_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.path_tolerance.begin(),goal.path_tolerance.end(),tol);
  
  if (!nh.getParam("goal_tolerance",tol.position))
    tol.position = 0.00;
  goal.goal_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),tol);
  
  ROS_INFO("Waiting for action server");
  ac.waitForServer(ros::Duration(0.0));
  
  ac.cancelAllGoals();
  
  goal.trajectory.header.stamp=ros::Time::now();
  ROS_INFO("Waiting for trajectory execution");
  ac.sendGoalAndWait(goal);
  
  
}
