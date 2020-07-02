#include <moveit_msgs/DisplayTrajectory.h>

#include <apriltags/AprilTagDetections.h>

#include <itia_futils/itia_futils.h>
#include <itia_rutils/itia_rutils.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

double fRand(int motion_amplitude)
{
  
  int discretization = rand() % 200 + (10);
  
  double rand_num = rand() % discretization;
  
  return ( -motion_amplitude + motion_amplitude * 2 * rand_num / discretization ) * 3.14/180.;
  
}

iiwa_msgs::JointPosition command_joint_position_new;

int JointMotionLim[] = {25, 25, 20, 25, 20, 35, 150};

void randomJointPositions(iiwa_msgs::JointPosition start_jnt_pos){
 
  double delta_jnt_pos;
  
  delta_jnt_pos = fRand(JointMotionLim[0]);
  command_joint_position_new.position.a1 = start_jnt_pos.position.a1 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[1]);
  command_joint_position_new.position.a2 = start_jnt_pos.position.a2 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[2]);
  command_joint_position_new.position.a3 = start_jnt_pos.position.a3 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[3]);
  command_joint_position_new.position.a4 = start_jnt_pos.position.a4 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[4]);
  command_joint_position_new.position.a5 = start_jnt_pos.position.a5 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[5]);
  command_joint_position_new.position.a6 = start_jnt_pos.position.a6 + delta_jnt_pos;
  delta_jnt_pos = fRand(JointMotionLim[6]);
  command_joint_position_new.position.a7 = start_jnt_pos.position.a7 + delta_jnt_pos;
  
}

void sleepForMotion(iiwa_ros::iiwaRos& iiwa, const double maxSleepTime) {
  
  double ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
  
  ros::Time start_wait = ros::Time::now();
  
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
    
    ros::Duration(0.5).sleep();
    ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
    
  }
  
  if (ttd > 0.0) {
    
    ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
    ros::Duration(ttd).sleep();
    
  } 
  
}

int N_POSE = 61;
int cont = 0;

geometry_msgs::Transform kinect_pose;
bool data_from_kinect;

void kinect_pose_sub_cb ( const ar_track_alvar_msgs::AlvarMarkersConstPtr msg )
{
  std::cout << msg->markers.size() << std::endl;

  if (msg->markers.size()==1)
  {

    data_from_kinect = true;
    
    kinect_pose.translation.x=msg->markers[msg->markers.size()-1].pose.pose.position.x;
    kinect_pose.translation.y=msg->markers[msg->markers.size()-1].pose.pose.position.y;
    kinect_pose.translation.z=msg->markers[msg->markers.size()-1].pose.pose.position.z;
    kinect_pose.rotation.x=msg->markers[msg->markers.size()-1].pose.pose.orientation.x;
    kinect_pose.rotation.y=msg->markers[msg->markers.size()-1].pose.pose.orientation.y;
    kinect_pose.rotation.z=msg->markers[msg->markers.size()-1].pose.pose.orientation.z;
    kinect_pose.rotation.w=msg->markers[msg->markers.size()-1].pose.pose.orientation.w;

  }
  else
    data_from_kinect = false;
  
}

geometry_msgs::Transform iiwa_pose;

void iiwa_pose_sub_cb ( const geometry_msgs::PoseStampedConstPtr msg )
{
  
  iiwa_pose.translation.x=msg->pose.position.x;
  iiwa_pose.translation.y=msg->pose.position.y;
  iiwa_pose.translation.z=msg->pose.position.z;
  iiwa_pose.rotation.x=msg->pose.orientation.x;
  iiwa_pose.rotation.y=msg->pose.orientation.y;
  iiwa_pose.rotation.z=msg->pose.orientation.z;
  iiwa_pose.rotation.w=msg->pose.orientation.w;

}

int main (int argc, char **argv) 

{  
  ros::init(argc, argv, "kinect_calibration_publisher");
  ros::NodeHandle nh("~");
  
  double ros_rate;
  nh.param("ros_rate", ros_rate, 25.0); // Hz
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
  ros::Subscriber kinect_pose_sub=nh.subscribe ("/ar_pose_marker",1,kinect_pose_sub_cb);
  ros::Subscriber iiwa_pose_sub=nh.subscribe ("/iiwa/state/CartesianPose",1,iiwa_pose_sub_cb);
  
  ros::Publisher kinect_pose_pub=nh.advertise<geometry_msgs::Transform> ("/camera_object",1,true);
  ros::Publisher iiwa_pose_pub=nh.advertise<geometry_msgs::Transform> ("/world_effector",1,true);
  
  ros::spinOnce();
  
  usleep ( 500 );
  
  kinect_pose_pub.publish (kinect_pose);
  iiwa_pose_pub.publish (iiwa_pose);
  
  ros::spinOnce();
  
  usleep ( 500 );
  
  ///////////////////////Initial Joint Position////////////////////////
  
  ros::spinOnce();
  
  iiwa_msgs::JointPosition command_joint_position;
  iiwa_msgs::JointPosition msr_joint_position;
  
  command_joint_position.position.a1 = -0.0169680453837;
  command_joint_position.position.a2 = 0.697795152664;
  command_joint_position.position.a3 = 0.00384897342883;
  command_joint_position.position.a4 = -1.108026874065;
  command_joint_position.position.a5 = -0.042765468359;
  command_joint_position.position.a6 = 0.30789976119995;
  command_joint_position.position.a7 = 0.;
  
  bool connection_flag = false;
  
  while (!connection_flag){
  
    if (my_iiwa.getRobotIsConnected()) {
    
      connection_flag = true;
      
      while (!my_iiwa.getJointPosition(msr_joint_position)) {}
      
      my_iiwa.setJointPosition(command_joint_position);
			  
      sleepForMotion(my_iiwa, 2.0);
      
    }
    else
    {
    
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
      
    }
  
    ros::spinOnce();
  
  }
  
  ros::spinOnce();
  
  ros::Duration(5.0).sleep();
  
  bool flag=true;
  
  while (ros::ok()) 
  
  {

    while ( flag ) {
      
      connection_flag = false;
      
      randomJointPositions(command_joint_position);
      
      while (!connection_flag){
	
	ros::spinOnce();
	
	if (my_iiwa.getRobotIsConnected()) {
	  
	  std::cout << "setto nuova posizione" << std::endl;
      
	  connection_flag = true;
	
	  while (!my_iiwa.getJointPosition(msr_joint_position)) {}
	
	  my_iiwa.setJointPosition(command_joint_position_new);
			    
	  sleepForMotion(my_iiwa, 5.0);
	
	}
	else
	{
      
	  ROS_WARN_STREAM("Robot is not connected...");
	  ros::Duration(5.0).sleep(); // 5 seconds
	
	}
    
      }
      
      ros::Duration(5.0).sleep();
      
      ros::spinOnce();
      
	
      if ( !ros::ok() ){
	    
	return 0;

      }   
      else if ( data_from_kinect ) {
	
	  ros::Duration(1.0).sleep();
	    
	  ros::spinOnce();
	  
	  if (kinect_pose.translation.z > 1.6 && kinect_pose.translation.z < 3.)
	  {
	  cont++;
	
	  kinect_pose_pub.publish (kinect_pose);
	  iiwa_pose_pub.publish (iiwa_pose);
	}
	else
	  std::cout << "bad position reading from kinect" << std::endl;
      }
      
      else 
      {
      std::cout << "cambio posa senza acquisire" << std::endl;
      }

      if (cont==N_POSE)
      {
	      flag=false;
      }
      
      ros::spinOnce();
      
    }
    
    return 0;
    
  }
    
}; 
