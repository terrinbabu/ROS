#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h>
#include <itia_rutils/itia_rutils.h>

 #include <std_msgs/Float32MultiArray.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>
#include <boost/iterator/iterator_concepts.hpp>

#include <kinova_msgs/PoseVelocity.h>
 
 #include "std_msgs/Bool.h"

using namespace std;

typedef  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> Client ;

class rosPS
{
private:
    
    ros::NodeHandle&                                        nh;
    ros::Publisher&                                         planning_scene_diff_publisher;
    
    ros::Subscriber                                         rob_next_sub;
    ros::Subscriber                                         tsr_sub;
    ros::Subscriber                                         pomdp_stop_sub; 
    ros::Publisher                                          rob_pose_pub;
    ros::Publisher                                          pose_velocity_pub;
    ros::Publisher                                          goal_tsr_pub;
    ros::Publisher                                          restart_pub;
    
    itia::rutils::MsgReceiver<std_msgs::Float32MultiArray>  msg_receiver;
    
    geometry_msgs::Pose                                     new_eep;
    moveit::planning_interface::MoveGroupInterface          group;
    robot_model_loader::RobotModelLoader                    robot_model_loader;
    const std::string                                       arm_planning_group_name;
    const std::string                                       end_effector_name;
    
    int call_id;
    int stop;
    int once_pub;
    double x,y,z,qw,qx,qy,qz;
    int pomdp_stop_int;
    
public:
  
    rosPS ( ros::NodeHandle& nh,
            ros::Publisher& planning_scene_diff_publisher,
            const std::string& next_eep,
            const std::string& rob_eep,
            const std::string& min_tsr,
            const std::string& pose_velocity,
            const std::string& goal_tsr,
            const std::string& restart,
            const std::string& pomdp_stop)
    
    : nh                      ( nh )
    , msg_receiver            ( "NextPoseReceiver" )
    , robot_model_loader      ( "robot_description" )
    , planning_scene_diff_publisher (planning_scene_diff_publisher)
    , arm_planning_group_name ("arm")
    , end_effector_name       ("j2n6s300_end_effector")
    , group                   ( "arm" )
    {
        call_id = 1;
        stop = 0;
        once_pub=0;
        pomdp_stop_int = 0;
        
        pomdp_stop_sub = nh.subscribe(pomdp_stop, 1000, &rosPS::pomdp_stop_Callback,this);
        tsr_sub = nh.subscribe(min_tsr, 1, &rosPS::objtsrCallback,this);
// // //         rob_next_sub  = nh.subscribe(next_eep, 1,&itia::rutils::MsgReceiver<std_msgs::Float32MultiArray>::callback, &msg_receiver);
        rob_next_sub  = nh.subscribe(next_eep, 1,&rosPS::next_eep_Callback,this);
        rob_pose_pub  = nh.advertise<geometry_msgs::Pose> ( rob_eep, 1 );
        pose_velocity_pub  = nh.advertise<kinova_msgs::PoseVelocity> ( pose_velocity, 100 );
        goal_tsr_pub =  nh.advertise<geometry_msgs::Pose> ( goal_tsr, 100 );
        restart_pub = nh.advertise<std_msgs::Bool> ( restart, 100 );
        
    }

    ~rosPS() 
    {};

    void pomdp_stop_Callback(const std_msgs::Bool::ConstPtr& msg)
    {
      if(msg->data == true)
         pomdp_stop_int = 1;
      if(msg->data == false)
        pomdp_stop_int = 0;
    }
    
    void objtsrCallback(const std_msgs::Float32MultiArray& min_tsr_info)
    {
      if (stop == 0)
      { 
        std::vector<double> min_tsr_info_ = {0, 0, 0, 0,
                                             0, 0, 0, 0,
                                             0, 0, 0, 0,
                                             0, 0, 0, 0 };
        
        for ( std::size_t j = 0; j < min_tsr_info.data.size(); ++j )
            min_tsr_info_[j] = min_tsr_info.data[j];
        
        geometry_msgs::Pose goal_tsr;
        goal_tsr.position.x = min_tsr_info_[3];
        goal_tsr.position.y = min_tsr_info_[7];
        goal_tsr.position.z = min_tsr_info_[11];
        
        double tqw,tqx,tqy,tqz;
        
        itia::support::Matrix_to_quad(min_tsr_info_[0],min_tsr_info_[1],min_tsr_info_[2],
                                      min_tsr_info_[4],min_tsr_info_[5],min_tsr_info_[6],
                                      min_tsr_info_[8],min_tsr_info_[9],min_tsr_info_[10],
                                      tqw,tqx,tqy,tqz);
        
        goal_tsr.orientation.x = tqx;
        goal_tsr.orientation.y = tqy;
        goal_tsr.orientation.z = tqz;
        goal_tsr.orientation.w = tqw;
        
        double tolerance ;
        itia::support::dis_bet_2_pose(tolerance,new_eep,goal_tsr);
        
        if (tolerance < 0.03)
        {
          x = min_tsr_info_[3]; y = min_tsr_info_[7]; z = min_tsr_info_[11];
          qw = tqw; qx = tqx; qy = tqy; qz = tqz;
          stop = 1;
          
//           ros::Rate poll_rate(30);
//           
//           while(goal_tsr_pub.getNumSubscribers() == 0)
//           poll_rate.sleep();
          
          goal_tsr_pub.publish ( goal_tsr );
        }
      }
    }
    
  void next_eep_Callback (const std_msgs::Float32MultiArray& received_twist)
    { 
      once_pub=1;
      ros::Rate poll_rate(100);

      if(stop == 0)
      {
      
      std::vector<double> received_twist_ = {0, 0, 0, 0, 0, 0} ;

      if(pomdp_stop_int == 0)
      {
        for ( std::size_t j = 0; j < received_twist.data.size(); ++j )
            received_twist_[j] = received_twist.data[j];
      }
      else if (pomdp_stop_int == 1)
        ROS_INFO("%s Robot Stopped (complete the human action or exit)",BOLDRED);

      kinova_msgs::PoseVelocity twist;
      twist.twist_linear_x = received_twist_[0];
      twist.twist_linear_y = received_twist_[1];
      twist.twist_linear_z = received_twist_[2];
      twist.twist_angular_x = received_twist_[3];
      twist.twist_angular_y = received_twist_[4];
      twist.twist_angular_z = received_twist_[5];              
      pose_velocity_pub.publish (twist);

      itia::support::tf_wrt_base("/world","/j2n6s300_end_effector", new_eep);

// //       std_msgs::Bool restart_bool;
// //       restart_bool.data = false;
// // 
// //       restart_pub.publish (restart_bool);
    }
    else
    {
      itia::kinova_utils::move("adjusting to the goal",x,y,z,qw,qx,qy,qz);
      sleep(5.0);
      itia::kinova_utils::closehand(false,false);
      sleep(5.0);
      itia::kinova_utils::openhand(false,false);
      sleep(5.0);
      
      std_msgs::Bool restart_bool;
      restart_bool.data = true;
      
      while(restart_pub.getNumSubscribers() == 0)
      poll_rate.sleep();
      restart_pub.publish (restart_bool);
      sleep(5.0);
      
      stop = 0;
      
      itia::support::tf_wrt_base("/world","/j2n6s300_end_effector", new_eep);
      
      rob_pose_pub.publish ( new_eep );
    }
    }
    
  void rob_pose_pub_fun()
   {
     if(stop == 0 && once_pub == 1)
      {
        ros::Rate lp(100);
        while (ros::ok())
          {
            rob_pose_pub.publish ( new_eep );
            lp.sleep();
           }
      }
   }
// // // //     geometry_msgs::Pose nexteepCalc ( const std_msgs::Float32MultiArray& received_twist )
// // // //     { 
// // // //         once_pub=1;
// // // //         ros::Rate poll_rate(100);
// // // //         
// // // //         if(stop == 0)
// // // //         {
// // // // // // INFO 
// // // // 
// // // //         std::vector<double> received_twist_ = {0, 0, 0, 0, 0, 0} ;
// // // //         geometry_msgs::Pose new_eep_pose;
// // // //         
// // // //         if(pomdp_stop_int == 0)
// // // //         {
// // // //           for ( std::size_t j = 0; j < received_twist.data.size(); ++j )
// // // //               received_twist_[j] = received_twist.data[j];
// // // // 
// // // //         
// // // // // //  CLACULATING NEXT EEP POSE - PLAN AND EXECUTE & ACTION LIB
// // // //         
// // // //         
// // // // // //         double time_step = 0.2;
// // // // // //         itia::moveit_utils::twisttoPose(group,received_twist_,time_step,new_eep_pose);
// // // //         }
// // // //         else if (pomdp_stop_int == 1)
// // // //         {
// // // //           ROS_INFO("%s -------Robot Stopped (complete the human action or exit)----- ",BOLDRED);
// // // //           itia::moveit_utils::getcurrentpose(new_eep_pose,group);
// // // //         }
// // // // // //         itia::support::printgeomentrypose("Next end effector pose",new_eep_pose);      
// // // //    
// // // // // //  CLACULATING NEXT JOINT VALUES - PLAN AND EXECUTE & ACTION LIB
// // // // 
// // // // // //         std::vector<double> joint_twist = {0, 0, 0, 0, 0, 0} ;
// // // // // //         
// // // // // //         itia::moveit_utils::twisttoJointtwist(group,robot_model_loader,arm_planning_group_name,end_effector_name,received_twist_,joint_twist);
// // // // // //         itia::support::printvectordouble("joint_twist",joint_twist);
// // // // // //         
// // // // // //         std::vector<double> change_in_joint_values = {0, 0, 0, 0, 0, 0};
// // // // // //         double time_step = 0.2;
// // // // // //         itia::support::vect_scalar_mulp(joint_twist,time_step,change_in_joint_values);
// // // // // //         
// // // // // //         std::vector<double> current_joint_values; 
// // // // // //         group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), current_joint_values);
// // // // // // 
// // // // // //         std::vector<double> new_joint_values = {0, 0, 0, 0, 0, 0};
// // // // // //         itia::support::vect_vect_add(current_joint_values,change_in_joint_values,new_joint_values);
// // // //         
// // // // // //  PLAN AND EXECUTE - new pose
// // // //         
// // // // // //         bool gazebo;  bool fake;
// // // // // //
// // // // // //         itia::kinova_utils::active_controller(nh,"/pomdp/gazebo","/pomdp/fake",fake,gazebo);
// // // //         
// // // // // //         std::vector<double> joint_values;
// // // // // //         if (!itia::moveit_utils::invKin(new_eep_pose,robot_model_loader,arm_planning_group_name,joint_values))
// // // // // //           ROS_ERROR ( "NO IK solution" );
// // // // // //        
// // // // // //         itia::moveit_utils::plan_execute(group,joint_values);
// // // //        
// // // //         
// // // //         
// // // // // //  PLAN AND EXECUTE - new joint
// // // //         
// // // // // //         itia::moveit_utils::plan_execute(group,new_joint_values);
// // // // 
// // // // // //  ACTION LIB - new pose
// // // //        
// // // // // //        itia::kinova_utils::move("move",new_eep_pose.position.x,new_eep_pose.position.y,new_eep_pose.position.z,
// // // // // //                                        new_eep_pose.orientation.w,new_eep_pose.orientation.x,new_eep_pose.orientation.y,new_eep_pose.orientation.z);
// // // // 
// // // //         
// // // // // //  ACTION LIB - new joint values
// // // // 
// // // // //        itia::kinova_utils::move_joint("move_joint",new_joint_values[0],new_joint_values[1],new_joint_values[2],
// // // // //                                                    new_joint_values[3],new_joint_values[4],new_joint_values[5]);
// // // //         
// // // //         
// // // //         
// // // // // // //  VELOCITY PUBLISER
// // // //         
// // // // //                 std::cout << "Velocity publisher" << std::endl ;
// // // //                 kinova_msgs::PoseVelocity twist;
// // // //                 twist.twist_linear_x = received_twist_[0];
// // // //                 twist.twist_linear_y = received_twist_[1];
// // // //                 twist.twist_linear_z = received_twist_[2];
// // // //                 twist.twist_angular_x = received_twist_[3];
// // // //                 twist.twist_angular_y = received_twist_[4];
// // // //                 twist.twist_angular_z = received_twist_[5];              
// // // //                 pose_velocity_pub.publish (twist);
// // // // 
// // // // // // //  TF POSE
// // // //         
// // // // 
// // // //         itia::support::tf_wrt_base("/world","/j2n6s300_end_effector", new_eep);
// // // // // //                 itia::support::printgeomentrypose("new_eep_pose",new_eep);
// // // // 
// // // // // // // MOVEIT POSE
// // // //                 
// // // // // //                 geometry_msgs::Pose new_eep;                
// // // // // //                 itia::moveit_utils::getcurrentpose(new_eep,group);
// // // //       
// // // //       rob_pose_pub.publish ( new_eep );
// // // //       
// // // //       std_msgs::Bool restart_bool;
// // // //       restart_bool.data = false;
// // // //       
// // // //       restart_pub.publish (restart_bool);
// // // //          }
// // // //          else
// // // //          {
// // // //           geometry_msgs::Pose current_eep_pose ;
// // // //           itia::moveit_utils::getcurrentpose(current_eep_pose,group);
// // // //           itia::kinova_utils::move("adjusting to the goal",x,y,z,qw,qx,qy,qz);
// // // //           sleep(5.0);
// // // //           itia::kinova_utils::closehand(false,false);
// // // //           sleep(2.0);
// // // //           itia::kinova_utils::move("moving down",current_eep_pose.position.x,current_eep_pose.position.y,current_eep_pose.position.z - 0.15,qw,qx,qy,qz);
// // // //           sleep(2.0);
// // // //           itia::kinova_utils::move("moving up",current_eep_pose.position.x,current_eep_pose.position.y,current_eep_pose.position.z + 0.15,qw,qx,qy,qz);
// // // //           sleep(2.0);
// // // //           itia::kinova_utils::openhand(false,false);
// // // //           sleep(2.0);
// // // //           std_msgs::Bool restart_bool;
// // // //           restart_bool.data = true;
// // // //           
// // // //           while(restart_pub.getNumSubscribers() == 0)
// // // //           poll_rate.sleep();
// // // //           restart_pub.publish (restart_bool);
// // // //           stop = 0;
// // // //           
// // // //           itia::support::tf_wrt_base("/world","/j2n6s300_end_effector", new_eep);
// // // //           
// // // //           rob_pose_pub.publish ( new_eep );
// // // //           
// // // //           if (pomdp_shut_down_int == 1)
// // // //           {
// // // //             ROS_INFO("%s -------ALL ROBOT ACTION COMPLETED---THANK YOU-- ",BOLDRED);
// // // //             ros::shutdown();
// // // //           }
// // // //         }
// // // //     }
// // // // 
// // // //   void run( )
// // // //     {
// // // //         std_msgs::Float32MultiArray msg;
// // // //         kinova_msgs::PoseVelocity twist;
// // // // //         ros::Rate lp(100);
// // // //         while (ros::ok())
// // // //         {
// // // //             if ( msg_receiver.isANewDataAvailable() )
// // // //             {
// // // //                 msg = msg_receiver.getData();
// // // //                 nexteepCalc( msg );
// // // //                 
// // // //             }
// // // // //             lp.sleep();
// // // //         }
// // // //     }


  void rob_current_pose()
  {
     if (once_pub == 0)
     {
        ROS_INFO ( "%s Publishing robot pose for first time",BOLDYELLOW);

        geometry_msgs::Pose eep_pose;
        itia::moveit_utils::getcurrentpose(eep_pose,group);

        ros::Rate poll_rate(100);
        
        while(rob_pose_pub.getNumSubscribers() == 0)
            poll_rate.sleep();
        
        rob_pose_pub.publish ( eep_pose );
        
        std_msgs::Bool restart_bool;
        restart_bool.data = false;
        
        while(restart_pub.getNumSubscribers() == 0)
            poll_rate.sleep();
        
        restart_pub.publish (restart_bool);
        
        once_pub = 1;
     }
  }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "pomdp" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
  
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    sleep(0.5);

    rosPS topics ( node_handle,
                   planning_scene_diff_publisher,
                   "/herb/next_eep", 
                   "/herb/active_eep",
                   "/herb/mintsr",
                   "/j2n6s300_driver/in/cartesian_velocity",
                   "/goal_tsr",
                   "/restart",
                   "/pomdp_stop");

    topics.rob_current_pose();
    
    topics.rob_pose_pub_fun();
    
// // //     topics.run(); 

    itia::support::end();
    return 0;
}

