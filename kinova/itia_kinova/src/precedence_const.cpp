#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <itia_futils/itia_futils.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <iostream>
#include <string> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <vector>

#include <itia_tutils/itia_tutils.h>
#include <itia_rutils/itia_rutils.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

// Grazebo trajectory
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//publisher
#include <std_msgs/Float32MultiArray.h>
#include <itia_human_prediction/PoseArrays.h>


#include <moveit/robot_model/robot_model.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf2_msgs/TFMessage.h"



// #include <kinova_msgs/ArmJointAnglesAction.h>
// #include <kinova_msgs/ArmPoseAction.h>
// #include <kinova_msgs/SetFingersPositionAction.h>
// #include <kinova_msgs/PoseVelocity.h>
using namespace std;
class rosPS
{
private:
    
    ros::NodeHandle&                                                     nh;
    ros::Publisher&                                                      planning_scene_diff_publisher;
    ros::Subscriber                                                      prob_goal_user_sub;
    itia::rutils::MsgReceiver<std_msgs::Float32MultiArray>             	 msg_receiver;
    moveit::planning_interface::MoveGroupInterface                       group;
    std::vector<double> 						          goal_dist;
    std::vector<std::string>						      HOP;
    std::vector<std::string>						      HPA;
    std::string						 		              HCA;
    std::map<std::string,std::string>::iterator 		  it;
    std::map<std::string,std::string>::iterator		      jt;
    std::vector<std::string>						      ROP;
    std::vector<std::string>						      RPA;
    std::vector<std::string>						      RCA;
    std::vector<std::string>						      RCA_New;
     std::vector<std::string>                             RCA_New1;
    std::map<std::string,std::string> 					  Preced_const_list;
    std::vector<std::string>::iterator 			 		  kt1;
    std::vector<std::string>::iterator 			 		  lt1;
    std::vector<std::string>::iterator 			 		  mt1;
    std::vector<std::string>::iterator 			 		  nt1;
    std::vector<std::string>::iterator 			 		  ot1;
    std::vector<std::string>::iterator 			 		  pt1;
    std::vector<std::string>::iterator 					  qt1;
    std::vector<std::string>::iterator 					  result;
    std::vector<std::string>::iterator 				 	  result1;
    int 								                  call_id;
    int 								                  once_pub;
    bool found =false;
    ros::Subscriber                                       object_pose_sub;
    ros::Publisher                                        object_tsr_pub;
    tf::TransformBroadcaster                              br;
    tf::Transform                                         transform;
    tf::StampedTransform                                  mapTransform;
    tf::TransformListener                                 listener;
    geometry_msgs::PoseArray 						      obj_poses;
    std::multimap<std::string,double> 					  actions_mapping;
    std::vector<std::vector<double>>                      pose;
    std::vector<double>                                   pose1;
    std::vector<double>                                   pose2;
    std::vector<double>                                   pose3;
    std::vector<double>                                   pose4;
    std::vector<double>                                   pose5;
    std::vector<double>                                   pose6;
    itia_human_prediction::PoseArrays                     alltsr;
    std::vector<string>                                   NotRCA_New;
    
    
public:
    rosPS ( ros::NodeHandle& nh,
            ros::Publisher& planning_scene_diff_publisher, 
            const std::string& prob_goal_user_topic,
	    const std::string& obj_pose_topic
	  )
    : nh                ( nh )
    , msg_receiver      ( "goal_info_Receiver" )
    , planning_scene_diff_publisher (planning_scene_diff_publisher)
    , group             ( "arm" )
    {
      once_pub=0;
      call_id =1;
      
      object_pose_sub = nh.subscribe(obj_pose_topic, 1000, &rosPS::obj_pose_callback,this);
      object_tsr_pub =  nh.advertise<itia_human_prediction::PoseArrays>("/constraints/tsr", 1000);
      prob_goal_user_sub = nh.subscribe(prob_goal_user_topic, 1,&itia::rutils::MsgReceiver<std_msgs::Float32MultiArray>::callback, &msg_receiver);
    }

    ~rosPS() 
    {};
    
    void obj_pose_callback (const geometry_msgs::PoseArray& received_objpose_info)
   {
	if(once_pub==0)
	{
		for ( int i=0; i<received_objpose_info.poses.size(); i++ )
		{
		  geometry_msgs::Pose object_pose;
		  std::vector<double> obj_pose =  { received_objpose_info.poses[i].position.x,
						    received_objpose_info.poses[i].position.y,
						    received_objpose_info.poses[i].position.z };
		  std::vector<double> obj_orient ={ received_objpose_info.poses[i].orientation.w,
						    received_objpose_info.poses[i].orientation.x,
						    received_objpose_info.poses[i].orientation.y,
						    received_objpose_info.poses[i].orientation.z };

		  itia::support::fromVecToPose(obj_pose,obj_orient,object_pose);
		  obj_poses.poses.push_back(object_pose);	
		  }
		  
		  
		std::cout<<"obj_poses"<<std::endl;
		std::cout<<obj_poses<<std::endl;
		
		itia_human_prediction::PoseArrays all_tsr;
		  
		for (size_t i=0; i<obj_poses.poses.size(); i++)
		{
		      geometry_msgs::PoseArray object_tsr;
		      geometry_msgs::Pose tsr;

		      tsr.position.x = obj_poses.poses[i].position.x;
		      tsr.position.y = obj_poses.poses[i].position.y;
		      tsr.position.z = obj_poses.poses[i].position.z + 0.4;
		
		      tsr.orientation.x = -0.503124177456;
		      tsr.orientation.y =  0.499893963337;
		      tsr.orientation.z = -0.498644709587;
		      tsr.orientation.w =  0.498322844505;
		
		      object_tsr.poses.push_back(tsr);
		      all_tsr.poses.push_back(object_tsr);
		} // for
		  
	    //       ros::Rate poll_rate(100);
	    //       while(object_tsr_pub.getNumSubscribers() == 0)
	    //       poll_rate.sleep();
		  
	    //       std::cout << all_tsr << std::endl;

		std::vector<geometry_msgs::Pose> tsr_poses;
		
		for (size_t i=0; i<all_tsr.poses.size(); i++)
		{
		  geometry_msgs::Pose tsr;
		  
		  tsr.position.x = all_tsr.poses[i].poses[0].position.x;
		  tsr.position.y = all_tsr.poses[i].poses[0].position.y;
		  tsr.position.z = all_tsr.poses[i].poses[0].position.z;

		  tsr.orientation.x = all_tsr.poses[i].poses[0].orientation.x;
		  tsr.orientation.y = all_tsr.poses[i].poses[0].orientation.y;
		  tsr.orientation.z = all_tsr.poses[i].poses[0].orientation.z;
		  tsr.orientation.w = all_tsr.poses[i].poses[0].orientation.w;
		  
		  tsr_poses.push_back(tsr);
		}
		
		for (size_t i=0; i<tsr_poses.size(); i++)
		{
		  ROS_INFO ( "%s tsr_poses %zu - position : [%f %f %f] ", BOLDCYAN ,i, tsr_poses[i].position.x, tsr_poses[i].position.y, tsr_poses[i].position.z );
		  ROS_INFO ( "%s tsr_poses %zu - orentation : [%f %f %f %f] ", BOLDCYAN ,i, tsr_poses[i].orientation.x, tsr_poses[i].orientation.y, tsr_poses[i].orientation.z, tsr_poses[i].orientation.w );
		  
		  std::stringstream ss3;
		  ss3 << "R_"<<i+1;
		  std::string str3 = ss3.str();

		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].position.x));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].position.y));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].position.z));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].orientation.x));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].orientation.y));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].orientation.z));
		  actions_mapping.insert(pair<std::string, double>(str3, tsr_poses[i].orientation.w));
		}
		
		for (std::map<std::string,double>::iterator pt1=actions_mapping.begin(); pt1!=actions_mapping.end(); ++pt1)
		{
		  std::cout<<"actions_mapping list:"<< pt1->first << " => " << pt1->second << '\n'<<std::endl;
		}
	  
	}
	
	once_pub = 1;
	
	
    
     // object_tsr_pub.publish(all_tsr);
      /*ros::Rate loop_rate(10);
	while (ros::ok())
	{  
	  object_tsr_pub.publish(all_tsr);
	  ros::spinOnce();
	  loop_rate.sleep();
	}  */  
   }//void ob_pos_call_back
//    
//    
    void prob_goal_user_back (const std_msgs::Float32MultiArray& received_goal_info)
   {   
	  std::cout<<"received_goal_info.data.size():"<<received_goal_info.data.size()<<std::endl;
    
	  if (call_id == 1)
	  {
	      for ( int i=0; i<received_goal_info.data.size(); i++ )
	      {
		    
		  std::stringstream ss1;
		  ss1 << "H_"<<i+1;
		  std::string str1 = ss1.str();
		  std::cout<<"str :"<<str1<<std::endl;
		  HOP.push_back(str1);	
		  std::stringstream ss2;
		  ss2 << "R_"<<i+1;
		  std::string str2 = ss2.str();
		  std::cout<<"str1 :"<<str2<<std::endl;
		  ROP.push_back(str2);
		  
		      if (!nh.getParam("/Preced_const_list/",Preced_const_list))
			      ROS_ERROR("Error in reading Precd_const_list");   
		  
		      for (std::map<std::string,std::string>::iterator it5=Preced_const_list.begin(); it5!=Preced_const_list.end(); ++it5)
		      {
			      std::cout<<"precedence constraints list:"<< it5->first << " => " << it5->second << '\n'<<std::endl;
		      }
	      }
	  }
    
    for ( int i=0; i<received_goal_info.data.size(); i++ )
    {
      
	  ROS_INFO("%sreceived_goal_info of %d: %f ", BOLDCYAN,i+1,received_goal_info.data[i]);
      
	  if(received_goal_info.data[i]>0.8)
	  {
		ROS_INFO("%sHuman goal H_%d completed", BOLDCYAN,i+1);
		std::stringstream ss3;
		ss3 << "H_"<<i+1;  
		std::string str3 = ss3.str();
		HCA = str3;
		std::cout<<"HCA:"<<HCA<<std::endl;

		for (ot1=HPA.begin(); ot1!=HPA.end(); ++ot1)
		{
		      if(HCA == ot1->c_str())
		      {
			std::cout<<"111"<<std::endl;
			found =true;
			break;
		      }
		}
	    
    // 	
		if(!found)
		{
		      HPA.push_back(HCA);
		      for (nt1=HPA.begin(); nt1!=HPA.end(); ++nt1)
			    ROS_INFO( "HPA contains %s", nt1->c_str() );
		      HOP.erase(HOP.begin()+i);
		      for (kt1=HOP.begin(); kt1<HOP.end(); kt1++)
			    ROS_INFO( "HOP contains %s", kt1->c_str() );
		}
    // 	
//       }
//       
	    std::cout<<"HCA:"<<HCA<<std::endl;
	    std::cout<<"HPA.size():"<<HPA.size()<<std::endl;
	    std::cout<<"HOP.size():"<<HOP.size()<<std::endl;
//       
	    if(HCA.size()>0)
	    {
	      
		    for( std::vector<std::string>::iterator mt1=ROP.begin(); mt1!=ROP.end();++mt1)
		    {
			  for(std::map<std::string,std::string>::iterator it2=Preced_const_list.begin(); it2!=Preced_const_list.end(); ++it2)
			  {
			      std::cout<<"comparing ROP and precedence:"<< it2->first << " => " << mt1->c_str() << '\n'<<std::endl;
			      
				if(it2->first==mt1->c_str())
				{
				  std::cout<<"222"<<std::endl;
				  RCA.push_back(mt1->c_str());
				}
			  }
		    }
		    
		    for( std::vector<std::string>::iterator qt1=HPA.begin(); qt1!=HPA.end();++qt1)
		    {
			  for(std::map<std::string,std::string>::iterator it3=Preced_const_list.begin(); it3!=Preced_const_list.end(); ++it3)
			  {
				std::cout<<"comparing HPA and precedence:"<< it3->first << " => " << qt1->c_str() << '\n'<<std::endl; 
				
				if(it3->first==qt1->c_str())
				{
				  RCA.push_back(it3->second);
				}
			  }
		    }
      // 	
		    for( std::vector<std::string>::iterator lt1=RPA.begin(); lt1!=RPA.end();++lt1)
		    {
			  std::string value2 = lt1->c_str();
			  std::cout<<"value2:"<<value2<<std::endl;    
			  std::vector<std::string>::iterator result2 = find(RCA.begin(),RCA.end(),value2);
			  std::cout<<"*result2:"<<*result2<<std::endl;
			  
			  if(result2 == RCA.end())
			  {
			      continue;	      
			  }
			  else
			  {
			      RCA.erase(result2);
			  }
	    // 	
		    }
		    
		      std::vector<string>::iterator nt2;
		      std::sort(RCA.begin(), RCA.end());
		      nt2 = std::unique(RCA.begin(), RCA.end());
		      RCA.resize(std::distance(RCA.begin(), nt2));
				    
		    
		    for (std::vector<std::string>::iterator nt2=RCA.begin(); nt2!=RCA.end(); ++nt2)
			    ROS_INFO( "RCA contains %s", nt2->c_str() );
	      
      // 	 
		//    std::copy(RCA.begin(), RCA.end(), back_inserter(RCA_New)); 	
      // 	 
      // 	 
      //       
		    for(std::map<std::string,std::string>::iterator it4=Preced_const_list.begin(); it4!=Preced_const_list.end(); ++it4)
		    {
			  if(HCA=="H_2")
			  {
			    
			      continue;
			  }
			  else if(HCA == it4->first)
			  {
			      std::cout<<"444"<<std::endl;
			      std::string value = it4->second;
			      std::cout<<"value:"<<value<<std::endl;    
			      std::vector<std::string>::iterator result = find(RCA.begin(),RCA.end(),value);
			      std::cout<<"*result:"<<*result<<std::endl;      
			      
			      if(result == RCA.end())
			      {
				  std::cout<<"555"<<std::endl;
				  continue;	      
			      }
			      else
			      {
// 				for (std::multimap<std::string,double>::iterator it8=actions_mapping.begin(); it8!=actions_mapping.end(); ++it8)
// 				{
// 				    std::cout<<"it8->first = "<<it8->first<<"   result = "<<*result<<endl;
// 				    if(it8->first== it4->second)
// 				    {
// 				      std::cout<<"key = "<<it8->first<<"    Value = "<<it8->second<<endl;				  
// 				      pose.push_back(999);
// 				      
// 				    }
// 				}
				  std::cout<<"666"<<std::endl;
				  RCA.erase(result);
				  
				      for (pt1=RCA.begin(); pt1!=RCA.end(); ++pt1)
				      {
					ROS_INFO( "after deleting RCA_New contains %s", pt1->c_str() );
				      }
			      }
			    
			  }
		    }
		    
		    
		    for (std::vector<std::string>::iterator pt4=RCA.begin(); pt4!=RCA.end(); ++pt4)
            {
               ROS_INFO( "RCA_New contains %s", pt4->c_str() );
            }
           
           
            for( std::vector<std::string>::iterator mt1=ROP.begin(); mt1!=ROP.end();++mt1)
            {
              
              std::string value3 = mt1->c_str();
              std::cout<<"value3:"<<value3<<std::endl;    
              std::vector<std::string>::iterator result3=find(RCA.begin(),RCA.end(),value3);
              std::cout<<"*result3:"<<*result3<<std::endl;
              
              
              if(result3 == RCA.end())
              {
                  NotRCA_New.push_back(value3);       
              }
              else
              {
                  continue;
              }
            }
		    
		    
		    
		    
		    for (std::vector<std::string>::iterator pt2=RCA.begin(); pt2!=RCA.end(); ++pt2)
		    {
			  for (std::multimap<std::string,double>::iterator it6=actions_mapping.begin(); it6!=actions_mapping.end(); ++it6)
			  {
			    std::cout<<"it6->first = "<<it6->first<<"    pt2->c_str() = "<<pt2->c_str()<<endl;
				if(it6->first == pt2->c_str() )
				{
                  if(it6->first=="R_1")
                  {
				  std::cout<<"key = "<<it6->first<<"    Value = "<<it6->second<<endl;				  
				  pose1.push_back(it6->second);
                  }
                  if(it6->first=="R_2")
                  {
                  std::cout<<"key = "<<it6->first<<"    Value = "<<it6->second<<endl;                 
                  pose2.push_back(it6->second);
                  }
                  if(it6->first=="R_3")
                  {
                  std::cout<<"key = "<<it6->first<<"    Value = "<<it6->second<<endl;                 
                  pose3.push_back(it6->second);
                  }
				}		
			   }    
		    }  
		    
		   
            
          
            
            
            
            
            
            
            
            for (std::vector<std::string>::iterator nt3=NotRCA_New.begin(); nt3!=NotRCA_New.end(); ++nt3)
                ROS_INFO( "NotRCA_New contains %s", nt3->c_str() );
          
            
            
		    std::cout<<" pose1.size() = "<<pose1.size()<<endl;
            std::cout<<" pose2.size() = "<<pose2.size()<<endl;
            std::cout<<" pose3.size() = "<<pose3.size()<<endl;
		    for(int k=0;k<pose.size();k++)
            {
		    pose.push_back(pose1);
            pose.push_back(pose2);
            pose.push_back(pose3);
            }
		    std::cout<<" RCA_New.size() = "<<RCA.size()<<endl;
			
		    for(int m=0; m<pose.size();m++)
		     {
             for(int n=0; n<pose.size();n++)
             {
			std::cout<<" pose[m][n] = "<<pose[m][n]<<endl;
		     }
             }
		    std::cout<<" pose.size() = "<<pose.size()<<endl;
		
// 		    for(int m=0; m<pose.size();++m)
// 		    {  
// 		      std::cout<<"start m is:"<<m<<std::endl;
// 		      std::cout<<"m is:"<<m<<std::endl;
// 		      
// 			geometry_msgs::PoseArray objecttsr;
// 			geometry_msgs::Pose tsr;
// 			
// 			
// 			tsr.position.x = pose[m];
// 			tsr.position.y = pose[m+1];
// 			tsr.position.z = pose[m+2];
// 		      
// 		      
// 			tsr.orientation.x = pose[m+3];
// 			tsr.orientation.y = pose[m+4];
// 			tsr.orientation.z = pose[m+5];
// 			tsr.orientation.w = pose[m+6];
// 			objecttsr.poses.push_back(tsr);
// 			alltsr.poses.push_back(objecttsr);
// 			std::cout << alltsr << std::endl;
// 		      m=m+6;
// 		      std::cout<<"m is:"<<m<<std::endl;
// 		      
// 		    }
// 		    
// 		    
// 		    object_tsr_pub.publish(alltsr);
	    
	  
//       
// 	    
	 
	  
// 	  std::cout<<"777"<<std::endl;
// 	  bool kappa=true;
// 	  ROS_INFO( "pt->c_str() contains %s", pt2->c_str() );
// 	  if(kappa)
// 	  {
// 	    std::cout<<"888"<<std::endl;
// 	    RPA.push_back(pt2->c_str());
// 	    for( std::vector<std::string>::iterator lt2=RPA.begin(); lt2!=RPA.end();++lt2)
// 		  ROS_INFO( "RPA contains %s", lt2->c_str() );
// 	    std::string value1 = pt2->c_str();
// 	    std::cout<<"value1:"<<value1<<std::endl;    
// 	    std::vector<std::string>::iterator result1 = find(ROP.begin(),ROP.end(),value1);
// 	    std::cout<<"*result1:"<<*result1<<std::endl;
// 	    ROP.erase(result1);
// 	    for( std::vector<std::string>::iterator mt2=ROP.begin(); mt2!=ROP.end();++mt2)
// 		  ROS_INFO( "ROP contains %s", mt2->c_str() );
// 	    break;
// 	  }
	      alltsr.poses.clear();
	      pose.clear();
	      RCA.clear();
	      HCA.clear();
	      RCA_New.clear();
	      found =false;
	      }
	
	}
    
	
      }	
      call_id++ ;
     } 
    
    
    
    void run( )
      {
	  std_msgs::Float32MultiArray msg;
      
	  ros::Rate lp(10);
	  while (ros::ok())
	  {
	      if ( msg_receiver.isANewDataAvailable() )
	      {
		  msg = msg_receiver.getData();
		  prob_goal_user_back( msg);
	      }
	      lp.sleep();
	  }
      }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "precedence_const" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
  
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    sleep(0.5);
    
    ROS_INFO("%sAdding the environment", BOLDCYAN);
    std::vector<double> table_pose;
    std::vector<double> table_orient;
    std::vector<double> table_dim;

    if (!node_handle.getParam("/scene_configuration/table/position",table_pose))
      ROS_ERROR("Error in reading table pose");    
    if (!node_handle.getParam("/scene_configuration/table/orientation",table_orient))
      ROS_ERROR("Error in reading table orientation");    
    if (!node_handle.getParam("/scene_configuration/table/dimensions",table_dim))
      ROS_ERROR("Error in reading table dim");
      
    itia::moveit_utils::addBox(planning_scene_diff_publisher, table_pose, table_orient,table_dim,"root","world","table");

//     std::vector<double> obj1_pose = { 0.30,-0.365,0};
//     std::vector<double> obj1_orient = {1, 0, 0, 0};
//     std::vector<double> obj1_dim = {0.152,0.152,0.085};
// 
//     std::vector<double> obj2_pose = { 0.60,-0.365,0};
//     std::vector<double> obj2_orient = {1, 0, 0, 0};
//     std::vector<double> obj2_dim = {0.152,0.152,0.085};
//     
//     std::vector<double> obj3_pose = { 0.90,-0.365,0};
//     std::vector<double> obj3_orient = {1, 0, 0, 0};
//     std::vector<double> obj3_dim = {0.152,0.152,0.085};
//     
//     ROS_INFO("%sAdding the objects to world", BOLDCYAN);
// 
//     itia::moveit_utils::addBox(planning_scene_diff_publisher, obj1_pose, obj1_orient,obj1_dim,"world","world","bottle_1");
// 
//     itia::moveit_utils::addBox(planning_scene_diff_publisher, obj2_pose, obj2_orient,obj2_dim,"world","world","bottle_2");
//     
//     itia::moveit_utils::addBox(planning_scene_diff_publisher, obj3_pose, obj3_orient,obj3_dim,"world","world","bottle_3");
//     
//     itia::support::printString("Grasping analysis");
//   
//   
//     moveit_simple_grasps::SimpleGraspsPtr simple_grasps_; 
//     moveit_simple_grasps::GraspData grasp_data_;
//     std::vector<moveit_msgs::Grasp> possible_grasps;
//     moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
// //     const std::string arm_planning_group_name = "arm";
//     const std::string hand_planning_group_name = "gripper";
//     
//     itia::moveit_utils::initVisualTool(visual_tools_);
//     
//     itia::grasp_utils::initGrasp(visual_tools_, simple_grasps_);
//     
//     itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
// 					simple_grasps_, visual_tools_, 
// 					grasp_data_, obj1_pose, obj1_orient, 
// 					possible_grasps);
//     
//     
//     std::vector<moveit_msgs::Grasp> sorted_possible_grasps;
//     itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps);
//     
//      
//     itia::grasp_utils::visualizeGrasp(simple_grasps_, visual_tools_, grasp_data_, sorted_possible_grasps[0], hand_planning_group_name);
//     
//     itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
// 					simple_grasps_, visual_tools_, 
// 					grasp_data_, obj2_pose, obj2_orient, 
// 					possible_grasps);
//     std::vector<moveit_msgs::Grasp> sorted_possible_grasps2;
//     itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps2);
//     
//      
//     itia::grasp_utils::visualizeGrasp(simple_grasps_, visual_tools_, grasp_data_, sorted_possible_grasps2[0], hand_planning_group_name);
//     
//     itia::grasp_utils::generatedGrasps( hand_planning_group_name, 
// 					simple_grasps_, visual_tools_, 
// 					grasp_data_, obj3_pose, obj3_orient, 
// 					possible_grasps);
//     std::vector<moveit_msgs::Grasp> sorted_possible_grasps3;
//     itia::grasp_utils::idMaxQualityGrasp(possible_grasps, false, sorted_possible_grasps3);
//     
//      
//     itia::grasp_utils::visualizeGrasp(simple_grasps_, visual_tools_, grasp_data_, sorted_possible_grasps3[0], hand_planning_group_name);
//    
   
    rosPS topics ( node_handle,planning_scene_diff_publisher, 
                   "/skel/prob_goal_user","/env_obj/pos"/*"/env_obj/tsr"*/);
// 
//     topics.rob_current_pose();
// 
     
     topics.run(); 

    itia::support::end();
    return 0;
}

