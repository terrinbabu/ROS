#ifndef __BASIC___H__
#define __BASIC___H__

#include <math.h>
#include <numeric>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace ros;

namespace tsupport 
{
namespace basic 
{  

void print_red (const std::string toPrint){cout << "\033[1;31m" << toPrint.c_str() << "\033[0m";}
void print_green (const std::string toPrint){cout << "\033[1;32m" << toPrint.c_str() << "\033[0m";}
void print_yellow (const std::string toPrint){cout << "\033[1;33m" << toPrint.c_str() << "\033[0m";}
void print_blue (const std::string toPrint){cout << "\033[1;34m" << toPrint.c_str() << "\033[0m";}
void print_magenta (const std::string toPrint){cout << "\033[1;35m" << toPrint.c_str() << "\033[0m";}
void print_cyan (const std::string toPrint){cout << "\033[1;36m" << toPrint.c_str() << "\033[0m";}
void print_white (const std::string toPrint){cout << "\033[1;37m" << toPrint.c_str() << "\033[0m";}

void printString(const std::string toPrint){
  cout << "\n \033[0;34m---------------------------------------------------\n" 
       << toPrint.c_str() 
       << "\n---------------------------------------------------\033[0m\n";}

void printvectordouble(const std::string vector_name,std::vector<double> vector){
    std::string temp ="[";
    for(std::size_t i=0; i < vector.size(); ++i)
    {
      temp.append(std::to_string(vector[i]));
      temp.append(" ");
    }
    temp.append("]");
    cout << "\033[0;33m" << vector_name.c_str() << " : " << temp.c_str() << " \033[0m" << endl;}

void printvectorint(const std::string vector_name,std::vector<int> vector){ 
    std::string temp ="[";
    for(std::size_t i=0; i < vector.size(); ++i)
    {
      temp.append(std::to_string(vector[i]));
      temp.append(" ");
    }
    temp.append("]");
    cout << "\033[0;33m" << vector_name.c_str() << " : " << temp.c_str() << " \033[0m" << endl;}

void printgeomentrypose(const std::string pose_name,geometry_msgs::Pose pose){
  cout << "\033[1;33m "<< pose_name.c_str() << " - Position : \033[0m [ " << fixed << setprecision(2) 
                                                                          << pose.position.x  << ","
                                                                          << pose.position.y  << ","
                                                                          << pose.position.z  << " ]"
                                                                          << endl;

  cout << "\033[1;33m " << pose_name.c_str() << "- Orientation : \033[0m [ "  << fixed << setprecision(2)
                                                                              << pose.orientation.w << ","
                                                                              << pose.orientation.x << ","
                                                                              << pose.orientation.y << ","
                                                                              << pose.orientation.z << " ]"
                                                                              << endl;}

bool check(){
  int yes;
  cout << endl << " \033[1;34m Check (press 1 and enter if OK) \033[0m" << endl;  
  std::cin >> yes;
  if (yes == 1)
    return true;
  else
    return false;}

void MatrixToQuad(Eigen::MatrixXd& eep_rotation,double& qw,double& qx,double& qy,double& qz){
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
  
  qx = x; qy = y; qz = z; qw = w;}

void Matrix_to_quad(double& m11,double& m12,double& m13,
                    double& m21,double& m22,double& m23,
                    double& m31,double& m32,double& m33,
                    double& qw,double& qx,double& qy,double& qz){
  Eigen::Matrix3f mat;
  
  mat << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;
  
  Eigen::Quaternionf q(mat);
  
  qw =   q.w();
  qx =   q.x();
  qy =   q.y();
  qz =   q.z();}

void quad_to_matrix(double& m11,double& m12,double& m13,
                    double& m21,double& m22,double& m23,
                    double& m31,double& m32,double& m33,
                    double& qw,double& qx,double& qy,double& qz){
  
  Eigen::Quaternion<double> quat;
  
  quat.w() = qw;
  quat.x() = qx;
  quat.y() = qy;
  quat.z() = qz;

  Eigen::Matrix3d rotm = quat.matrix();
  
  m11 = rotm (0,0);  m12 = rotm (0,1);  m13 = rotm (0,2);
  m21 = rotm (1,0);  m22 = rotm (1,1);  m23 = rotm (1,2);
  m31 = rotm (2,0);  m32 = rotm (2,1);  m33 = rotm (2,2);}
  
void QuadtoEuler(double& qw,double& qx,double& qy,double& qz,double& roll,double& pitch,double& yaw){
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
        yaw = std::atan2(t3, t4);}

void EulertoQuad(double& roll,double& pitch,double& yaw,double& qw,double& qx,double& qy,double& qz){
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        qw = t0 * t2 * t4 + t1 * t3 * t5;
        qx = t0 * t3 * t4 - t1 * t2 * t5;
        qy = t0 * t2 * t5 + t1 * t3 * t4;
        qz = t1 * t2 * t4 - t0 * t3 * t5;}

void fromVecToPose(const std::vector<double>& pose, const std::vector<double>& orient,geometry_msgs::Pose& pose_msg){
  pose_msg.orientation.w = orient[0]; 
  pose_msg.orientation.x = orient[1]; 
  pose_msg.orientation.y = orient[2];
  pose_msg.orientation.z = orient[3];
  pose_msg.position.x = pose[0]; 
  pose_msg.position.y = pose[1]; 
  pose_msg.position.z = pose[2];}

void fromPoseToVec(const geometry_msgs::Pose& pose_msg,std::vector<double>& pose, std::vector<double>& orient){
  orient.clear();
  orient.push_back(pose_msg.orientation.w);
  orient.push_back(pose_msg.orientation.x);
  orient.push_back(pose_msg.orientation.y);
  orient.push_back(pose_msg.orientation.z);
  pose.clear();
  pose.push_back(pose_msg.position.x);
  pose.push_back(pose_msg.position.y);
  pose.push_back(pose_msg.position.z);}

void fromVec_EulertoPose(std::vector<double>& pose_Vec_Euler,geometry_msgs::Pose& pose){
  pose.position.x = pose_Vec_Euler [0];
  pose.position.y = pose_Vec_Euler [1];
  pose.position.z = pose_Vec_Euler [2];
  
  double roll = pose_Vec_Euler [3];
  double pitch = pose_Vec_Euler [4];
  double yaw = pose_Vec_Euler [5];

  double qw,qx,qy,qz;
  basic::EulertoQuad(roll,pitch,yaw,qw,qx,qy,qz);
  
  pose.orientation.w = qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;}
  
void vect_scalar_mulp(std::vector<double>& vector,double& scalar,std::vector<double>& result){
  for (int i = 0; i<vector.size();i++)
    result[i]=vector[i]*scalar;}

void vect_vect_add(std::vector<double>& vector1,std::vector<double>& vector2,std::vector<double>& result){
  for (int i = 0; i<vector1.size();i++)
    result[i]=vector1[i]+vector2[i];}

void end(){
  ros::waitForShutdown();
  basic::printString("End of program");
  sleep(4.0);}

void tf_wrt_base(const std::string refrence_frame,const std::string new_frame,geometry_msgs::Pose& pose){
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
  pose.orientation.w = maptransform.getRotation().getW();}

void StampedTransform_to_pose (tf::StampedTransform& maptransform,geometry_msgs::Pose& pose){
  pose.position.x = maptransform.getOrigin().getX();
  pose.position.y = maptransform.getOrigin().getY();
  pose.position.z = maptransform.getOrigin().getZ();

  pose.orientation.x = maptransform.getRotation().getX();
  pose.orientation.y = maptransform.getRotation().getY();
  pose.orientation.z = maptransform.getRotation().getZ();
  pose.orientation.w = maptransform.getRotation().getW();}

void pose_to_pose (geometry_msgs::Pose& new_pose,geometry_msgs::Pose& current_pose){
  new_pose.position.x = current_pose.position.x;
  new_pose.position.y = current_pose.position.y;
  new_pose.position.z = current_pose.position.z;
  
  new_pose.orientation.x = current_pose.orientation.x ;
  new_pose.orientation.y = current_pose.orientation.y ;
  new_pose.orientation.z = current_pose.orientation.z ;
  new_pose.orientation.w = current_pose.orientation.w ;}

void creattf(const geometry_msgs::Pose& pose,const std::string ref_frame,const std::string new_frame){
  
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
  br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), ref_frame,new_frame ) ); }

void EigenVectoStdVec(Eigen::VectorXd&  EigenVec,std::vector<double>& StdVec){
          for ( std::size_t j = 0; j < EigenVec.size(); ++j )
            StdVec[j] = EigenVec[j];}

void StdVectoEignVec(std::vector<double>& StdVec,Eigen::VectorXd&  EigenVec){
          for ( std::size_t j = 0; j < StdVec.size(); ++j )
            EigenVec[j] = StdVec[j];}

void dis_bet_2_pose(double& st_line_length,geometry_msgs::Pose& pose_1,geometry_msgs::Pose& pose_2){
  st_line_length = sqrt(pow((pose_2.position.x - pose_1.position.x),2) + pow((pose_2.position.y - pose_1.position.y),2) + pow((pose_2.position.z - pose_1.position.z),2));}

void dis_pose_from_orgin(double& st_line_length,geometry_msgs::Pose& pose_1){
  st_line_length = sqrt(pow(pose_1.position.x,2) + pow(pose_1.position.y,2) + pow(pose_1.position.z,2));}

void tf_to_pose(tf2_msgs::TFMessage tf_info,geometry_msgs::Pose& pose){
    pose.position.x = tf_info.transforms[0].transform.translation.x;
    pose.position.y = tf_info.transforms[0].transform.translation.y;
    pose.position.z = tf_info.transforms[0].transform.translation.z;

    pose.orientation.w = tf_info.transforms[0].transform.rotation.w;
    pose.orientation.x = tf_info.transforms[0].transform.rotation.x;
    pose.orientation.y = tf_info.transforms[0].transform.rotation.y;
    pose.orientation.z = tf_info.transforms[0].transform.rotation.z;}

void min_value_in_array(double& min_value,int& min_value_array_no,std::vector<double>& array){
  min_value = array[0];
  min_value_array_no = 0;
  
  for(int i = 0; i != array.size(); ++i)
  {
      if(array[i] < min_value)
      {
          min_value = array[i];
          min_value_array_no = i;
      }
  }}

void max_value_in_array(double& max_value,int& max_value_array_no,std::vector<double>& array){
  max_value = array[0];
  max_value_array_no = 0;
  
  for(int i = 0; i != array.size(); ++i)
  {
      if(array[i] > max_value)
      {
        max_value = array[i];
        max_value_array_no = i;
      }
  }}

void vector_val_avg (double& avg,std::vector<double>& v){
    avg = accumulate( v.begin(), v.end(), 0.0)/v.size();}

void vector_max_min_avg (double& avg,std::vector<double>& v){
       double min,max;
       int min_num,max_num;

       tsupport::basic::min_value_in_array (min,min_num,v);
       tsupport::basic::max_value_in_array (max,max_num,v);
       avg = (max+min)/2;}

void MA_filter(Eigen::VectorXd& Input,int& N,Eigen::VectorXd& MA){
    
    int iSize = Input.size(); 
    MA = Input;

    for (int n=N;n<iSize;n++)
    {
        MA[n] = 0;
        
        for(int i=0;i<N;i++)
        {
            double temp = Input[n-i]/N;
            MA[n] = MA[n]+temp;
        }
    }}
    
void flip(Eigen::VectorXd& Input,Eigen::VectorXd& Flip){
    
    int iSize = Input.size(); 
    int fn = iSize-1;

    for (int i=0;i<iSize;i++)
    {
        Flip[fn] = Input[i];
        fn = fn-1;
    }}

void zero_phase_MA_filter(Eigen::VectorXd& Input,int& N,Eigen::VectorXd& zero_phase_MA){
    int iSize = Input.size(); 
    Eigen::VectorXd  MA(iSize),Flip_MA(iSize),Flip_FF(iSize);

    tsupport::basic::MA_filter (Input,N,MA);
    tsupport::basic::flip (MA,Flip_MA);
    tsupport::basic::MA_filter (Flip_MA,N,Flip_FF);
    tsupport::basic::flip (Flip_FF,zero_phase_MA);}


  
// void play_sound(ros::NodeHandle& nh,
//                 ros::Publisher& sound_pub,
//                 const std::string line_to_play )
// {
//   sound_play::SoundRequest sound;
//   sound.sound = -3;
//   sound.command = 1;
//   sound.volume = 1.0;
//   sound.arg = line_to_play;
//   sound.arg2 = "voice_kal_diphone";
//   
//   sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 1000);
//   ros::Rate poll_rate(100);
//   while(sound_pub.getNumSubscribers() == 0)
//     poll_rate.sleep();
//   
//   sound_pub.publish(sound);
// }

}
}

#endif
