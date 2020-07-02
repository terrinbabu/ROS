#include <iostream>
#include <fstream>

#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

using namespace std;

class rosPS
{
private:
    
    ros::NodeHandle&                                                     nh;
    ros::Subscriber                                                      hand_sub;
    itia::rutils::MsgReceiver<tf2_msgs::TFMessage>                       msg_receiver;
    moveit::planning_interface::MoveGroupInterface                       group;
public:
    rosPS ( ros::NodeHandle& nh,
            const std::string& hand_topic )
    : nh                ( nh )
    , msg_receiver      ( "marker_info_Receiver" )
    , group                   ( "arm" )
    {
        hand_sub = nh.subscribe(hand_topic, 1000,&itia::rutils::MsgReceiver<tf2_msgs::TFMessage>::callback, &msg_receiver);
    }

    ~rosPS() 
    {};
    
    void hum_hand_pose (const geometry_msgs::Pose::ConstPtr& hand_pose_info)
   {     
     geometry_msgs::Pose hand_pose;
     hand_pose.position.x = hand_pose_info-> position.x;
     hand_pose.position.y = hand_pose_info-> position.y;
     hand_pose.position.z = hand_pose_info-> position.z;
     hand_pose.orientation.w = hand_pose_info-> orientation.w;
     hand_pose.orientation.x = hand_pose_info-> orientation.x;     
     hand_pose.orientation.y = hand_pose_info-> orientation.y;
     hand_pose.orientation.z = hand_pose_info-> orientation.z;
     
     geometry_msgs::Pose ee_pose;
     itia::moveit_utils::getcurrentpose(ee_pose,group);
     
     double dis;
     itia::support::dis_bet_2_pose(dis,hand_pose,ee_pose);
     
     std::cout << "ee to hand dis : " << dis <<std::endl;
  
   } // void
   
  void run( )
    {
        tf2_msgs::TFMessage msg;
    
        ros::Rate lp(100);
        while (ros::ok())
        {
            if ( msg_receiver.isANewDataAvailable() )
            {
                msg = msg_receiver.getData();
                hum_hand_pose ( msg);
            }
            lp.sleep();
        }
    }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "hand_dis" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
    
    rosPS topics ( node_handle,
                   "/left_hand");
    
    topics.run();
    
    itia::support::end();
    return 0;
}