#ifndef __SUPPORT___H__
#define __SUPPORT___H__

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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <sound_play/SoundRequest.h>

typedef  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Client_fingers ;
typedef  actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> Client_joints ;

namespace itia 
{  

namespace support 
{  

bool printString(const std::string toPrint)
{
  ROS_INFO(" ");
  ROS_INFO("%s---------------------------------------------------",BOLDCYAN);
  ROS_INFO("%s%s", BOLDCYAN,toPrint.c_str());
  ROS_INFO("%s---------------------------------------------------",BOLDCYAN);
  ROS_INFO(" ");
  return true;
}

void printvectordouble(const std::string vector_name,
                       std::vector<double> vector)
{
    std::string temp ="[";
    for(std::size_t i=0; i < vector.size(); ++i)
    {
      temp.append(std::to_string(vector[i]));
      temp.append(" ");
    }
    temp.append("]");
    ROS_INFO("%s%s : %s",YELLOW,vector_name.c_str(),temp.c_str());
}

void printvectorint(const std::string vector_name,
                       std::vector<int> vector)
{ 
    std::string temp ="[";
    for(std::size_t i=0; i < vector.size(); ++i)
    {
      temp.append(std::to_string(vector[i]));
      temp.append(" ");
    }
    temp.append("]");
    ROS_INFO("%s%s : %s",YELLOW,vector_name.c_str(),temp.c_str());
}

void printgeomentrypose(const std::string pose_name,
                        geometry_msgs::Pose pose)
{
  ROS_INFO ( "%s %s - position : [%f %f %f] ", BOLDCYAN ,pose_name.c_str(), pose.position.x, pose.position.y, pose.position.z );
  ROS_INFO ( "%s %s - orentation : [%f %f %f %f] ", BOLDCYAN ,pose_name.c_str(), pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z );
}

bool check()
{
  int yes;
  ROS_INFO(" ");
  ROS_INFO("%sCheck (press 1 and enter if OK)", BOLDCYAN);     
  std::cin >> yes;
  if (yes == 1)
    return true;
  else
    return false;
}

bool MatrixToQuad(Eigen::MatrixXd& eep_rotation,
                  double& qw,double& qx,double& qy,double& qz)
{
  float m11,m12,m13;
  float m21,m22,m23;
  float m31,m32,m33;
  
  m11 = eep_rotation (0,0);m12 = eep_rotation (0,1);m13 = eep_rotation (0,2);
  m21 = eep_rotation (1,0);m22 = eep_rotation (1,1);m23 = eep_rotation (1,2);
  m31 = eep_rotation (2,0);m32 = eep_rotation (2,1);m33 = eep_rotation (2,2);

  float w,x,y,z;

  float fourWSquaredMinus1 = m11 + m22 + m33;
  float fourXSquaredMinus1 = m11 - m22 - m33;
  float fourYSquaredMinus1 = m22 - m11 - m33;
  float fourZSquaredMinus1 = m33 - m11 - m22;

  int biggestIndex = 0;
  float fourBiggestSquaredMinus1 = fourWSquaredMinus1;

  if(fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
      fourBiggestSquaredMinus1 = fourXSquaredMinus1;
      biggestIndex = 1;
  }
  if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
      fourBiggestSquaredMinus1 = fourYSquaredMinus1;
      biggestIndex = 2;
  }
  if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
      fourBiggestSquaredMinus1 = fourZSquaredMinus1;
      biggestIndex = 3;
  }
  
  float biggestVal = sqrt (fourBiggestSquaredMinus1 + 1.0f ) * 0.5f;
  float mult = 0.25f / biggestVal;
  
  switch (biggestIndex) 
    {
      case 0:
      w = biggestVal;
      x = (m23 - m32) * mult;
      y = (m31 - m13) * mult;
      z = (m12 - m21) * mult;
      break;
      case 1:
      x = biggestVal;
      w = (m23 - m32) * mult;
      y = (m12 + m21) * mult;
      z = (m31 + m13) * mult;
      break;
      case 2:
      y = biggestVal;
      w = (m31 - m13) * mult;
      x = (m12 + m21) * mult;
      z = (m23 + m32) * mult;
      break;
      case 3:
      z = biggestVal;
      w = (m12 - m21) * mult;
      x = (m31 + m13) * mult;
      y = (m23 + m32) * mult;
      break;
      }
  
  qx = x; qy = y; qz = z; qw = w;
}

void Matrix_to_quad(double& m11,double& m12,double& m13,
                    double& m21,double& m22,double& m23,
                    double& m31,double& m32,double& m33,
                    double& qw,double& qx,double& qy,double& qz)
{
  Eigen::Matrix3f mat;
  
  mat << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;
  
  Eigen::Quaternionf q(mat);
  
  qw =   q.w();
  qx =   q.x();
  qy =   q.y();
  qz =   q.z();
}

void quad_to_matrix(double& m11,double& m12,double& m13,
                    double& m21,double& m22,double& m23,
                    double& m31,double& m32,double& m33,
                    double& qw,double& qx,double& qy,double& qz)
{
  
  Eigen::Quaternion<double> quat;
  
  quat.w() = qw;
  quat.x() = qx;
  quat.y() = qy;
  quat.z() = qz;

  Eigen::Matrix3d rotm = quat.matrix();
  
  m11 = rotm (0,0);  m12 = rotm (0,1);  m13 = rotm (0,2);
  m21 = rotm (1,0);  m22 = rotm (1,1);  m23 = rotm (1,2);
  m31 = rotm (2,0);  m32 = rotm (2,1);  m33 = rotm (2,2);

}
void QuadtoEuler(double& qw,double& qx,double& qy,double& qz,double& roll,double& pitch,double& yaw)
{
        double ysqr = qy * qy;

        // roll (x-axis rotation)
        double t0 = +2.0 * (qw * qx + qy * qz);
        double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (qw * qy - qz * qx);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (qw * qz + qx * qy);
        double t4 = +1.0 - 2.0 * (ysqr + qz * qz);  
        yaw = std::atan2(t3, t4);
}

void EulertoQuad(double& roll,double& pitch,double& yaw,double& qw,double& qx,double& qy,double& qz)
{
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        qw = t0 * t2 * t4 + t1 * t3 * t5;
        qx = t0 * t3 * t4 - t1 * t2 * t5;
        qy = t0 * t2 * t5 + t1 * t3 * t4;
        qz = t1 * t2 * t4 - t0 * t3 * t5;
}

bool fromVecToPose(const std::vector<double>& pose, 
                   const std::vector<double>& orient,  
                   geometry_msgs::Pose& pose_msg)
{
  pose_msg.orientation.w = orient[0]; 
  pose_msg.orientation.x = orient[1]; 
  pose_msg.orientation.y = orient[2];
  pose_msg.orientation.z = orient[3];
  pose_msg.position.x = pose[0]; 
  pose_msg.position.y = pose[1]; 
  pose_msg.position.z = pose[2];
  return true;
}

bool fromPoseToVec(const geometry_msgs::Pose& pose_msg,
                   std::vector<double>& pose, 
                   std::vector<double>& orient)
{
  orient.clear();
  orient.push_back(pose_msg.orientation.w);
  orient.push_back(pose_msg.orientation.x);
  orient.push_back(pose_msg.orientation.y);
  orient.push_back(pose_msg.orientation.z);
  pose.clear();
  pose.push_back(pose_msg.position.x);
  pose.push_back(pose_msg.position.y);
  pose.push_back(pose_msg.position.z);
  return true;
}

void fromVec_EulertoPose(std::vector<double>& pose_Vec_Euler,
                         geometry_msgs::Pose& pose)
{
  pose.position.x = pose_Vec_Euler [0];
  pose.position.y = pose_Vec_Euler [1];
  pose.position.z = pose_Vec_Euler [2];
  
  double roll = pose_Vec_Euler [3];
  double pitch = pose_Vec_Euler [4];
  double yaw = pose_Vec_Euler [5];

  double qw,qx,qy,qz;
  itia::support::EulertoQuad(roll,pitch,yaw,qw,qx,qy,qz);
  
  pose.orientation.w = qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
}

void vect_scalar_mulp(std::vector<double>& vector,
                                   double& scalar,
                      std::vector<double>& result)
{
  for (int i = 0; i<vector.size();i++)
    result[i]=vector[i]*scalar;
}

void vect_vect_add(std::vector<double>& vector1,
                   std::vector<double>& vector2,
                   std::vector<double>& result)
{
  for (int i = 0; i<vector1.size();i++)
    result[i]=vector1[i]+vector2[i];
}

void end()
{
  ros::waitForShutdown();
  itia::support::printString("End Tutorial");
  sleep(4.0);
}

void tf_wrt_base(const std::string refrence_frame,
                 const std::string new_frame,
                 geometry_msgs::Pose& pose)
{
  tf::TransformListener listener;
  tf::StampedTransform maptransform;
  try
  {
      listener.waitForTransform ( refrence_frame, new_frame, ros::Time ( 0 ),ros::Duration ( 0.1 ) ); //0.03 for pomdp //0.1 for tf
      listener.lookupTransform ( refrence_frame, new_frame,  ros::Time ( 0 ), maptransform );
  }
  catch ( tf::TransformException &ex )
  {
      ROS_ERROR ( "%s",ex.what() );
  }

  pose.position.x = maptransform.getOrigin().getX();
  pose.position.y = maptransform.getOrigin().getY();
  pose.position.z = maptransform.getOrigin().getZ();

  pose.orientation.x = maptransform.getRotation().getX();
  pose.orientation.y = maptransform.getRotation().getY();
  pose.orientation.z = maptransform.getRotation().getZ();
  pose.orientation.w = maptransform.getRotation().getW();

}

void StampedTransform_to_pose (tf::StampedTransform& maptransform,
                               geometry_msgs::Pose& pose)
{
  pose.position.x = maptransform.getOrigin().getX();
  pose.position.y = maptransform.getOrigin().getY();
  pose.position.z = maptransform.getOrigin().getZ();

  pose.orientation.x = maptransform.getRotation().getX();
  pose.orientation.y = maptransform.getRotation().getY();
  pose.orientation.z = maptransform.getRotation().getZ();
  pose.orientation.w = maptransform.getRotation().getW();
}

void pose_to_pose (geometry_msgs::Pose& new_pose,
                               geometry_msgs::Pose& current_pose)
{
  new_pose.position.x = current_pose.position.x;
  new_pose.position.y = current_pose.position.y;
  new_pose.position.z = current_pose.position.z;
  
  new_pose.orientation.x = current_pose.orientation.x ;
  new_pose.orientation.y = current_pose.orientation.y ;
  new_pose.orientation.z = current_pose.orientation.z ;
  new_pose.orientation.w = current_pose.orientation.w ;
  
}

void creattf(const geometry_msgs::Pose& pose,
             const std::string ref_frame,
             const std::string new_frame)
{
  
  double x, y, z;
  x = pose.position.x;
  y = pose.position.y;
  z = pose.position.z;
  
  tf::Quaternion q;
  tf::quaternionMsgToTF ( pose.orientation, q );
  
  tf::Transform transform;
  transform.setOrigin ( tf::Vector3 ( x, y, z ) );
  transform.setRotation ( q );
  
  tf::TransformBroadcaster br;
  br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), ref_frame,new_frame ) ); 
  
}

void EigenVectoStdVec(Eigen::VectorXd&  EigenVec,std::vector<double>& StdVec)
{
          for ( std::size_t j = 0; j < EigenVec.size(); ++j )
            StdVec[j] = EigenVec[j];
}

void StdVectoEignVec(std::vector<double>& StdVec,Eigen::VectorXd&  EigenVec)
{
          for ( std::size_t j = 0; j < StdVec.size(); ++j )
            EigenVec[j] = StdVec[j];
}

void play_sound(ros::NodeHandle& nh,
                ros::Publisher& sound_pub,
                const std::string line_to_play )
{
  sound_play::SoundRequest sound;
  sound.sound = -3;
  sound.command = 1;
  sound.volume = 1.0;
  sound.arg = line_to_play;
  sound.arg2 = "voice_kal_diphone";
  
  sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 1000);
  ros::Rate poll_rate(100);
  while(sound_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  
  sound_pub.publish(sound);
}

void dis_bet_2_pose(double& st_line_length,
                    geometry_msgs::Pose& pose_1,
                    geometry_msgs::Pose& pose_2)
{
  st_line_length = sqrt(pow((pose_2.position.x - pose_1.position.x),2) + pow((pose_2.position.y - pose_1.position.y),2) + pow((pose_2.position.z - pose_1.position.z),2));
}

void dis_pose_from_orgin(double& st_line_length,
                         geometry_msgs::Pose& pose_1)
{
  st_line_length = sqrt(pow(pose_1.position.x,2) + pow(pose_1.position.y,2) + pow(pose_1.position.z,2));
}

void tf_to_pose(tf2_msgs::TFMessage tf_info,
                geometry_msgs::Pose& pose)
{
    pose.position.x = tf_info.transforms[0].transform.translation.x;
    pose.position.y = tf_info.transforms[0].transform.translation.y;
    pose.position.z = tf_info.transforms[0].transform.translation.z;

    pose.orientation.w = tf_info.transforms[0].transform.rotation.w;
    pose.orientation.x = tf_info.transforms[0].transform.rotation.x;
    pose.orientation.y = tf_info.transforms[0].transform.rotation.y;
    pose.orientation.z = tf_info.transforms[0].transform.rotation.z;
}

void min_value_in_array(double& min_value,
                        int& min_value_array_no,
                        std::vector<double>& array)
{
  min_value = array[0];
  min_value_array_no = 0;
  
  for(int i = 0; i != array.size(); ++i)
  {
      if(array[i] < min_value)
      {
          min_value = array[i];
          min_value_array_no = i;
      }
  }
}

void max_value_in_array(double& max_value,
                        int& max_value_array_no,
                        std::vector<double>& array)
{
  max_value = array[0];
  max_value_array_no = 0;
  
  for(int i = 0; i != array.size(); ++i)
  {
      if(array[i] > max_value)
      {
        max_value = array[i];
        max_value_array_no = i;
      }
  }
}


// // bool addBox(ros::Publisher& planning_scene_diff_publisher, 
// //         const std::vector<double>& pose, 
// //         const std::vector<double>& orient, 
// //         const std::vector<double>& dim,
// //         const std::string& link_name,
// //         const std::string& frame_id,
// //         const std::string& object_id)
// // {
// //   moveit_msgs::AttachedCollisionObject attached_object;
// //   attached_object.link_name = link_name; //"jaco2_link_hand";      
// //   attached_object.object.header.frame_id = frame_id;//"world";
// //   attached_object.object.id = object_id; //"bottle";
// // 
// //   moveit_msgs::PlanningScene planning_scene1;
// // 
// //   geometry_msgs::Pose bottle_pose;  
// //   fromVecToPose(pose,orient,bottle_pose);
// // 
// //   shape_msgs::SolidPrimitive primitive;
// //   primitive.type = primitive.BOX;
// //   primitive.dimensions.resize(3);
// //   primitive.dimensions[0] = dim[0]; //0.08;
// //   primitive.dimensions[1] = dim[1]; //0.08;
// //   primitive.dimensions[2] = dim[2]; //0.26;
// // 
// //   attached_object.object.primitives.push_back(primitive);
// //   attached_object.object.primitive_poses.push_back(bottle_pose);
// // 
// //   attached_object.object.operation = attached_object.object.ADD;
// //   
// //     
// //   planning_scene1.world.collision_objects.push_back(attached_object.object);
// //   planning_scene1.is_diff = true;
// //   planning_scene_diff_publisher.publish(planning_scene1);
// //   sleep(2.0);
// //   
// //   return true;
// // }

// // void initVisualTool(moveit_visual_tools::MoveItVisualToolsPtr& visual_tools_)
// // {
// //   visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world"));     // Load the Robot Viz Tools for publishing to Rviz      
// //   visual_tools_->setLifetime(120.0);
// //   visual_tools_->loadMarkerPub();  
// // } 

// // bool removeObj(ros::Publisher& planning_scene_diff_publisher, 
// //               const std::vector<double>& pose, 
// //               const std::vector<double>& orient, 
// //               const std::vector<double>& dim,
// //               const std::string& link_name,
// //               const std::string& frame_id,
// //               const std::string& object_id)
// // {
// //   moveit_msgs::AttachedCollisionObject attached_object;
// //   attached_object.link_name = link_name; //"jaco2_link_hand";      
// //   attached_object.object.header.frame_id = frame_id;//"world";
// //   attached_object.object.id = object_id; //"bottle";
// // 
// //   
// // 
// //   geometry_msgs::Pose bottle_pose;  
// //   fromVecToPose(pose,orient,bottle_pose);
// // 
// //   shape_msgs::SolidPrimitive primitive;
// //   primitive.type = primitive.BOX;
// //   primitive.dimensions.resize(3);
// //   primitive.dimensions[0] = dim[0]; //0.08;
// //   primitive.dimensions[1] = dim[1]; //0.08;
// //   primitive.dimensions[2] = dim[2]; //0.26;
// // 
// //   attached_object.object.primitives.push_back(primitive);
// //   attached_object.object.primitive_poses.push_back(bottle_pose);
// // 
// //   attached_object.object.operation = attached_object.object.ADD;
// //   
// //   moveit_msgs::PlanningScene planning_scene1;  
// //   planning_scene1.world.collision_objects.push_back(attached_object.object);
// //   planning_scene1.is_diff = true;
// //   planning_scene_diff_publisher.publish(planning_scene1);
// //   sleep(2.0);
// //   
// //   return true;
// // }
// // 
// // 
// // bool moveTo(ros::NodeHandle& nh, 
// //               const std::vector<double>& joint_positions, 
// //               const std::vector<std::string>& joint_names, 
// //               const double& time,
// //               const std::string& topic_name="follow_joint_trajectory",
// //               const double& path_tolerance=0.001,
// //               const double& goal_tolerance=0.001 )
// // {
// //   control_msgs::FollowJointTrajectoryGoal goal;
// //   
// //   
// //   if (joint_positions.size() == 0)
// //   {
// //     ROS_INFO("Positions with no points");
// //     return false;
// //   }
// //   if (joint_positions.size() != joint_names.size())
// //   {
// //     ROS_INFO("Number of position different from the number of points");
// //     return false;
// //   }
// //   
// //   int npnt = 1;
// //   int dimension = joint_positions.size();
// //     
// //   goal.trajectory.points.resize(npnt);
// //   goal.trajectory.joint_names = joint_names;
// //   
// //   goal.trajectory.points.at(0).positions = joint_positions;
// //   goal.trajectory.points.at(0).time_from_start = ros::Duration(time);
// // 
// //   control_msgs::JointTolerance tol;
// //   tol.position = path_tolerance;  
// //   goal.path_tolerance.resize(goal.trajectory.joint_names.size());
// //   std::fill(goal.path_tolerance.begin(),goal.path_tolerance.end(),tol);
// // 
// //   tol.position = goal_tolerance;   
// //   goal.goal_tolerance.resize(goal.trajectory.joint_names.size());
// //   std::fill(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),tol);
// //   
// //   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(topic_name,true);  
// //   //ROS_INFO("Waiting for action server");
// //   ac.waitForServer(ros::Duration(0.0));
// //   goal.trajectory.header.stamp=ros::Time::now();
// //   ac.sendGoal(goal);
// //     
// //   //wait for the action to return
// //   bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
// //   if (finished_before_timeout)
// //   {
// //     actionlib::SimpleClientGoalState state = ac.getState();
// //     //     control_msgs::FollowJointTrajectoryResultConstPtr res = ac.getResult();
// //     //     std::cout<<"error: "<<res->error_string<<std::endl;    
// //     ROS_INFO("Action finished: %s",state.toString().c_str());    
// //     
// //     if (state.toString()=="SUCCEEDED")      
// //       return true;
// //     else
// //       return false;
// // 
// //   }
// //   else
// //   {
// //     ROS_INFO("Action did not finish before the time out.");
// //     return false;
// //   }  
// // }
// // 
// // bool readJointValueFromURDF(robot_model::RobotModelConstPtr& robot_model,
// //                             const std::string& arm_planning_group_name,
// //                             const std::string& pose_name,
// //                             const bool& printPose,
// //                             std::map<std::string, double>& joint_values)
// // {
// //   bool found = false;
// //   std::vector< srdf::Model::GroupState> gs = robot_model->getSRDF()->getGroupStates();
// //   for( auto it = gs.begin(); it != gs.end(); it++)
// //   {
// //     if( (it->name_ == pose_name) && (it->group_ == arm_planning_group_name) )
// //     {
// //       found=true;
// //       if(printPose)
// //         for( auto jt = it->joint_values_.begin(); jt != it->joint_values_.end(); jt++ )
// //         {
// //           joint_values[jt->first]=jt->second[0];   
// //           //std::cout << " name: " <<jt->first << ", value: " << jt->second[0] << std::endl;
// //         }      
// //     
// //       break;
// //     } 
// //   }  
// //   return found;
// // }
// // 
// // /*
// // bool setCurrentPosePlanningSceneFromUrdfPose(ros::NodeHandle nh, 
// //                                             robot_model::RobotModelConstPtr& robot_model,
// //                                             const std::string& arm_planning_group_name,
// //                                             const std::string& ns,
// //                                             const std::string& pose_name,
// //                                             bool printPose)
// // {
// //   std::map<std::string, double> joint_values;
// //   if(!readJointValueFromURDF(robot_model,
// //                              arm_planning_group_name,
// //                              pose_name,
// //                              printPose,
// //                              joint_values))
// //   {
// //     ROS_INFO("Joint position not found in the URDF file");
// //     return false;
// //   }
// //   
// //   robot_state::RobotState robot_state_start(robot_model);
// //   robot_state_start.setVariablePositions( joint_values );
// //   robot_state_start.update(); 
// // 
// //   bool ok= itia::mvutils::setRobotStateNH(nh, robot_state_start, robot_model, arm_planning_group_name, ns);
// //   if( !ok )
// //     return false;
// //   
// // }*/
// // 
// // // bool setCurrentPoseFromUrdfPose(ros::NodeHandle nh, 
// // //                                 robot_model::RobotModelConstPtr& robot_model,
// // //                                 moveit::planning_interface::MoveGroupInterface& group_arm,
// // //                                 const std::string& arm_planning_group_name,
// // //                                 const std::string& ns,
// // //                                 const std::string& pose_name,
// // //                                 bool printPose)
// // // { 
// // //   if( !itia::support::setCurrentPosePlanningSceneFromUrdfPose(nh, robot_model,arm_planning_group_name, ns, "home_arm", true) )
// // //     return false;
// // //   else
// // //   {  
// // //     robot_state::RobotState robot_state_start(robot_model);
// // //     moveit_msgs::RobotState robot_state_start_msg;  
// // //     moveit::core::robotStateToRobotStateMsg (robot_state_start, robot_state_start_msg);
// // //     group_arm.setStartState( robot_state_start_msg );
// // //     return true;
// // //   }  
// // // }
// // 
// // bool invKin(const geometry_msgs::Pose& pose_value,
// //             robot_model_loader::RobotModelLoader& robot_model_loader,
// //             const std::string& arm_planning_group_name,
// //             std::vector<double>& joint_value)
// // {  
// //   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
// //   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));  
// //   const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(arm_planning_group_name);
// //   
// //   bool found_ik;
// //   
// //   ROS_INFO(" ");
// //   ROS_INFO("%s Pose: [%f %f %f][%f %f %f %f] ", BOLDCYAN, 
// //            pose_value.position.x, 
// //            pose_value.position.y, 
// //            pose_value.position.z, 
// //            pose_value.orientation.w, 
// //            pose_value.orientation.x, 
// //            pose_value.orientation.y, 
// //            pose_value.orientation.z );
// //   ROS_INFO(" ");
// //   
// //   found_ik = kinematic_state->setFromIK(joint_model_group, pose_value, 20, 0.5);
// // 
// //   if (found_ik)
// //   {
// //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_value);     
// // 
// //     std::string temp =" Joints: [";
// //     for(std::size_t i=0; i < joint_value.size(); ++i)
// //     {
// //       temp.append(std::to_string(joint_value[i]));
// //       temp.append(" ");
// //     }
// //     temp.append("]");
// //     ROS_INFO("%s%s",GREEN,temp.c_str());
// //     return true;
// //   }
// //   else
// //   {
// //     ROS_INFO("%s Did not find IK solution", BOLDRED );
// //     return false;
// //   }
// // 
// // }
// // 
// // bool allowCollisions(robot_model_loader::RobotModelLoader& robot_model_loader,
// //                      const std::string& obj1,
// //                      const std::string& obj2)
// // {
// //   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
// //   planning_scene::PlanningScene planning_scene2(kinematic_model);    
// //   collision_detection::AllowedCollisionMatrix acm = planning_scene2.getAllowedCollisionMatrix();
// //   acm.setEntry(obj1, obj2, true);
// // }
// // 
// // bool checkCollisions(robot_model_loader::RobotModelLoader& robot_model_loader,
// //                      const std::string& name_group_name,
// //                      const std::vector<double>& joint_value)
// // {
// //   robot_model::RobotModelConstPtr robot_model = robot_model_loader.getModel();
// //   collision_detection::CollisionRequest collision_request;
// //   planning_scene::PlanningScenePtr  planning_scene( new planning_scene::PlanningScene( robot_model ) );
// //   collision_detection::CollisionResult collision_result; 
// //   collision_detection::CollisionResult::ContactMap::const_iterator it;  
// // 
// //   robot_state::RobotState robot_state_start(robot_model);
// //   robot_state_start.setJointGroupPositions(name_group_name, joint_value);
// //   robot_state_start.update();
// //   
// //   collision_request.contacts = true;
// //   collision_request.max_contacts = 1000;
// //     
// //   collision_result.clear();
// //   planning_scene->checkCollision(collision_request, collision_result, robot_state_start, planning_scene->getAllowedCollisionMatrix());
// //     
// //   
// // //     for( int j=0;j<joint_value.size(); j++)
// // //       ROS_INFO(" Joint #%d: %f", j, joint_value[j] );
// //   
// //   if(collision_result.collision)
// //   {
// //     ROS_INFO(" %sIn collision", BOLDRED );
// //     
// //     for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
// //     {
// //       ROS_INFO(" Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
// //       return true;
// //     }
// //   }
// //   else
// //   {
// //     ROS_INFO(" %sNot in collision", BOLDGREEN );
// //     return false;
// //   }
// // }
// // 
// // bool setTrajectoryPlan(moveit::planning_interface::MoveGroupInterface& group_arm,
// //                       const std::vector<double>& goal_joint,
// //                       moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
// // {
// //  ROS_INFO(" ");
// //   ROS_INFO("Set the start position of the planner");
// //   group_arm.setStartStateToCurrentState();
// //   
// //   ROS_INFO(" ");
// //   ROS_INFO("Set the goal position of the planner");
// //   group_arm.setJointValueTarget(goal_joint);
// // }
// //  
// // bool trajectoryPlan(moveit::planning_interface::MoveGroupInterface& group_arm,
// //                     moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
// // {
// //   ROS_INFO(" ");
// //   ROS_INFO("Plan");  
// //   bool success = group_arm.plan(my_plan);
// // 
// //   if( !success )
// //   {
// //   //     ROS_INFO(" ");
// //   //     ROS_INFO("%s Planning Again", BOLDCYAN );
// //   //     success = group.plan(my_plan);
// //   //     
// //   //     if( !success )
// //   //     {
// //   //       ROS_INFO(" ");
// //   //       ROS_INFO("%s Planning Again & Again", BOLDCYAN);
// //   //       success = group.plan(my_plan);
// //   //     }  
// //   //     if( !success )
// //   //     {
// //   //      ROS_INFO(" ");
// //   //      ROS_INFO("%s Planning for last time", BOLDCYAN);
// //   //      success = group.plan(my_plan);
// //   //     } 
// //     
// //     ROS_INFO(" ");
// //     ROS_INFO("The planner was not able to find any feasible plan ...");
// //     return false;
// //   } 
// //   else
// //     return true;
// // }
// // 
// // bool pose_action(kinova_msgs::ArmPoseGoal& goal,
// //                  moveit::planning_interface::MoveGroupInterface& group_arm)
// // {
// //                         goal.pose.header.frame_id = "j2n6s300_link_base";
// //                         goal.pose.pose.position.x = group_arm.getCurrentPose().pose.position.x;
// //                         goal.pose.pose.position.y = group_arm.getCurrentPose().pose.position.y;
// //                         goal.pose.pose.position.z = group_arm.getCurrentPose().pose.position.z;
// //                         goal.pose.pose.orientation.w = 0.498322844505;
// //                         goal.pose.pose.orientation.x = -0.503124177456;
// //                         goal.pose.pose.orientation.y = 0.499893963337;
// //                         goal.pose.pose.orientation.z = -0.498644709587;
// // }
// // 
// // bool closehand(bool& fake,bool& gazebo)
// // {
// //   itia::support::printString("close hand");
// //     
// //   if (fake)   // can be used for all three : gazebo and real robot (but slower and error in creating plan)
// //       {
// //           moveit::planning_interface::MoveGroupInterface group_hand("gripper");  //srdf
// //           
// //           std::vector<double> start_hand_joint_values; 
// //           group_hand.getCurrentState()->copyJointGroupPositions(group_hand.getCurrentState()->getRobotModel()->getJointModelGroup(group_hand.getName()), start_hand_joint_values);
// //           
// //           std::vector<double> hand_goal_joint = {0, 0, 0, 0};    
// //           hand_goal_joint[0] = start_hand_joint_values[0];
// //           hand_goal_joint[1] = start_hand_joint_values[1] + 1.3;
// //           hand_goal_joint[2] = start_hand_joint_values[2] + 1.3;
// //           hand_goal_joint[3] = start_hand_joint_values[3] + 1.3;
// //           
// //           group_hand.setStartStateToCurrentState();
// //           
// //           group_hand.setJointValueTarget(hand_goal_joint);
// //           
// //           bool success;
// //           moveit::planning_interface::MoveGroupInterface::Plan my_hand_plan; 
// //           success = group_hand.plan(my_hand_plan);
// //           
// //           if (!success)
// //             ROS_ERROR("Not able to close the hand");
// //           
// //           if (success)
// //             ROS_INFO("%s Success!! Success!! Success!! Success", BOLDGREEN );
// //           
// //           group_hand.execute(my_hand_plan);
// //       }
// //   else if (gazebo)
// //       {
// //         
// //             control_msgs::FollowJointTrajectoryGoal goal;
// //             goal.trajectory.points.resize(1);  
// //             goal.trajectory.points.at(0).time_from_start = ros::Duration(15);
// //             goal.trajectory.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
// //             goal.trajectory.points.at(0).positions = {1.3, 1.3, 1.3}; 
// //           
// //             actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2n6s300/effort_finger_trajectory_controller/follow_joint_trajectory",true);  
// // 
// //             ac.waitForServer(ros::Duration(0.0));
// //             goal.trajectory.header.stamp=ros::Time::now();
// //             ac.sendGoal(goal);
// //             
// //             ac.waitForResult(ros::Duration(20.0));
// //           
// //             actionlib::SimpleClientGoalState state = ac.getState();
// //             ROS_INFO("Gazebo Action state: %s",state.toString().c_str());
// // 
// //       }
// //   else
// //       {
// //         
// //             Client_fingers client_fingers("/j2n6s300_driver/fingers_action/finger_positions", true); // true -> don't need ros::spin()
// //             client_fingers.waitForServer();
// // 
// //             kinova_msgs::SetFingersPositionGoal goal;
// //             goal.fingers.finger1 = 6780;
// //             goal.fingers.finger2 = 6756;                        
// //             goal.fingers.finger3 = 6804;
// // 
// //             client_fingers.sendGoal(goal);
// //             client_fingers.waitForResult(ros::Duration(5.0));
// //             if (client_fingers.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// //                 ROS_INFO("%s Goal Completed", BOLDGREEN );
// // 
// //             std::printf("Current State: %s\n", client_fingers.getState().toString().c_str());
// //           
// //       }
// // }
// // 
// // bool openhand(bool& fake,bool& gazebo)
// // {
// //   printString("open hand");
// //     
// //   if (fake)   // can be used for all three : gazebo and real robot (but slower and error in creating plan)
// //       {
// //           moveit::planning_interface::MoveGroupInterface group_hand("gripper");  //srdf
// //           
// //           std::vector<double> start_hand_joint_values; 
// //           group_hand.getCurrentState()->copyJointGroupPositions(group_hand.getCurrentState()->getRobotModel()->getJointModelGroup(group_hand.getName()), start_hand_joint_values);
// //           
// //           std::vector<double> hand_goal_joint = {0, 0, 0, 0};    
// //           hand_goal_joint[0] = start_hand_joint_values[0];
// //           hand_goal_joint[1] = 0;
// //           hand_goal_joint[2] = 0;
// //           hand_goal_joint[3] = 0;
// //           
// //           group_hand.setStartStateToCurrentState();
// //           
// //           group_hand.setJointValueTarget(hand_goal_joint);
// //           
// //           bool success;
// //           moveit::planning_interface::MoveGroupInterface::Plan my_hand_plan; 
// //           success = group_hand.plan(my_hand_plan);
// //           
// //           if (!success)
// //             ROS_ERROR("Not able to open the hand");
// //           
// //           if (success)
// //             ROS_INFO("%s Success!! Success!! Success!! Success", BOLDGREEN );
// //           
// //           group_hand.execute(my_hand_plan);
// //       }
// //   else if (gazebo)
// //       {
// //             control_msgs::FollowJointTrajectoryGoal goal;
// //             goal.trajectory.points.resize(1);  
// //             goal.trajectory.points.at(0).time_from_start = ros::Duration(15);
// //             goal.trajectory.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
// //             goal.trajectory.points.at(0).positions = {0, 0, 0}; 
// //           
// //             actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2n6s300/effort_finger_trajectory_controller/follow_joint_trajectory",true);  
// // 
// //             ac.waitForServer(ros::Duration(0.0));
// //             goal.trajectory.header.stamp=ros::Time::now();
// //             ac.sendGoal(goal);
// //             
// //             ac.waitForResult(ros::Duration(20.0));
// //           
// //             actionlib::SimpleClientGoalState state = ac.getState();
// //             ROS_INFO("Gazebo Action state: %s",state.toString().c_str());
// // 
// //       }
// //   else
// //       {
// //         
// //             Client_fingers client_fingers("/j2n6s300_driver/fingers_action/finger_positions", true); // true -> don't need ros::spin()
// //             client_fingers.waitForServer();
// // 
// //             kinova_msgs::SetFingersPositionGoal goal;
// //             goal.fingers.finger1 = 0;
// //             goal.fingers.finger2 = 0;                        
// //             goal.fingers.finger3 = 0;
// // 
// //             client_fingers.sendGoal(goal);
// //             client_fingers.waitForResult(ros::Duration(5.0));
// //             if (client_fingers.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// //                 ROS_INFO("%s Goal Completed", BOLDGREEN );
// // 
// //             std::printf("Current State: %s\n", client_fingers.getState().toString().c_str());
// //           
// //       }
// // }
// // 
// // void visualize(ros::NodeHandle& nh,
// //                moveit::planning_interface::MoveGroupInterface& group_arm,
// //                moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
// // {
// //   ROS_INFO(" ");      
// //   ROS_INFO("%s Visualizing plan ", BOLDCYAN );
// //   
// //   moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;   // class for publishing stuff to rviz
// //   itia::support::initVisualTool(visual_tools_);
// //   
// //   const robot_state::JointModelGroup *joint_model_group = group_arm.getCurrentState()->getJointModelGroup(group_arm.getName());
// //   visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
// //   visual_tools_->trigger();  
// //   sleep(1.0);
// //   
// //   ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
// //   moveit_msgs::DisplayTrajectory display_trajectory;
// //   
// //   display_trajectory.trajectory_start = my_plan.start_state_;
// //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
// //   display_publisher.publish(display_trajectory);
// //   sleep(5.0);
// //   check();
// // }
// // 
// // bool plan(moveit::planning_interface::MoveGroupInterface& group_arm,
// //           std::vector<double>& goal_joint,
// //           moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
// // {
// //   ROS_INFO(" ");
// //   ROS_INFO("Set the start position of the planner");
// //   group_arm.setStartStateToCurrentState();
// //   
// //   ROS_INFO(" ");
// //   ROS_INFO("Set the goal position of the planner");
// //   group_arm.setJointValueTarget(goal_joint);
// //   
// //   bool success = group_arm.plan(my_plan);
// //   
// //   if (!success)
// //   {
// //      ROS_ERROR("Not able to identify a plan");
// //     return false;
// //   }
// //   if (success)
// //   {
// //      ROS_INFO("%s Success!! Success!! Success!! Success", BOLDGREEN );
// //      return true;
// //   }
// //   
// //   }
// // 
// // void active_controller(ros::NodeHandle& nh,
// //                        const std::string gazebo_param,
// //                        const std::string fake_param,
// //                        bool& fake,
// //                        bool& gazebo)
// // {
// //   bool read_gazebo =nh.getParam(gazebo_param,gazebo);
// //   bool read_fake =nh.getParam(fake_param,fake);
// //   
// //   if (fake)
// //       itia::support::printString("FAKE CONTROLLER IS ACTIVE");
// //   else if (gazebo)
// //       itia::support::printString("GAZEBO CONTROLLER IS ACTIVE");
// //   else
// //       itia::support::printString("REAL CONTROLLER IS ACTIVE");
// // }
// // 
// // void plan_visualize_execute( ros::NodeHandle& nh,
// //                              moveit::planning_interface::MoveGroupInterface& group_arm,
// //                              std::vector<double>& goal_joint,
// //                              moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
// // {
// //   itia::support::plan(group_arm,goal_joint,my_plan);  
// //   
// //   moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;   // class for publishing stuff to rviz
// //   itia::support::initVisualTool(visual_tools_);
// //   
// //   itia::support::visualize(nh,group_arm,my_plan);
// //   
// //   itia::support::printString("Execute");
// //   group_arm.execute(my_plan);
// // }
// // 
// // void gohome()
// // {
// //   Client_joints client_joints("/j2n6s300_driver/joints_action/joint_angles", true);
// //   client_joints.waitForServer();
// // 
// //   kinova_msgs::ArmJointAnglesGoal goal;
// // 
// //   goal.angles.joint1 = 275.36 ;
// //   goal.angles.joint2 = 167.43 ;
// //   goal.angles.joint3 = 57.26 ;
// //   goal.angles.joint4 = -119.08 ;
// //   goal.angles.joint5 = 82.68 ;
// //   goal.angles.joint6 = 75.68 ;
// // 
// //   client_joints.sendGoal(goal);
// //   client_joints.waitForResult(ros::Duration(5.0));
// //   if (client_joints.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// //     ROS_INFO("%s Goal Completed", BOLDGREEN );
// // 
// //   printf("Current State: %s\n", client_joints.getState().toString().c_str());
// // }


}}

#endif