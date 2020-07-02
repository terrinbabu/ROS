#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 
#include <itia_rutils/itia_rutils.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

class rosPS
{
private:
    
    ros::NodeHandle&                                                     nh;
    ros::Subscriber                                                      tf_sub;
    itia::rutils::MsgReceiver<tf2_msgs::TFMessage>                       msg_receiver;
    ros::Publisher                                                       left_hand_pub;
    double user1;
    double user2;
    double user3;
    double user4;
    double user5;
    double user6;
    geometry_msgs::Pose left_pose_1;
    geometry_msgs::Pose left_pose_2;
    geometry_msgs::Pose left_pose_3;
    geometry_msgs::Pose left_pose_4;
    geometry_msgs::Pose left_pose_5;
    geometry_msgs::Pose left_pose_6;
    geometry_msgs::Pose left_pose;
    tf::StampedTransform mapTransform;
    tf::TransformListener listener;

public:
    rosPS ( ros::NodeHandle& nh,
            const std::string& tf_topic )
    : nh                ( nh )
    , msg_receiver      ( "marker_info_Receiver" )
    {
        user1 = 999999;
        user2 = 999999;
        user3 = 999999;
        user4 = 999999;
        user5 = 999999;
        user6 = 999999;
        
        tf_sub = nh.subscribe(tf_topic, 1000,&rosPS::tf_topic_Callback,this);
        left_hand_pub = nh.advertise<geometry_msgs::Pose>("/left_hand", 1);
    }

    ~rosPS() 
    {};
    
    void tf_topic_Callback (const tf2_msgs::TFMessage& tf_info)
   {     
        ros::NodeHandle n;
        std::string parent_frame_id =  tf_info.transforms[0].header.frame_id.c_str();
        std::string frame_id =  tf_info.transforms[0].child_frame_id.c_str();
        

        
        if (parent_frame_id == "/head/skel_depth_frame")
        {
            if (frame_id == "/K2/user1/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user1/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user1/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_1);
              itia::support::dis_pose_from_orgin(user1,left_pose_1);
            }
            else if (frame_id == "/K2/user2/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user2/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user2/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_2);
              itia::support::dis_pose_from_orgin(user2,left_pose_2);
            }
            else if (frame_id == "/K2/user3/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user3/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user3/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_3);
              itia::support::dis_pose_from_orgin(user3,left_pose_3);
            }
            else if (frame_id == "/K2/user4/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user4/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user4/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_4);
              itia::support::dis_pose_from_orgin(user4,left_pose_4);
            }
            else if (frame_id == "/K2/user5/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user5/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user5/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_5);
              itia::support::dis_pose_from_orgin(user5,left_pose_5);
            }
            else if (frame_id == "/K2/user6/HandLeft")
            {
              try
              {
                  listener.waitForTransform ( "/world","/K2/user6/HandLeft", ros::Time ( 0 ),ros::Duration ( 0.4 ) );
                  listener.lookupTransform ( "/world", "/K2/user6/HandLeft",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
              
              itia::support::StampedTransform_to_pose(mapTransform,left_pose_6);
              itia::support::dis_pose_from_orgin(user6,left_pose_6);
              
            }
            
            if (user1 < user2 & user1 < user3 & user1 < user4 & user1 < user5 & user1 < user6)
              itia::support::pose_to_pose(left_pose,left_pose_1);
            
            else if (user2 < user1 & user2 < user3 & user2 < user4 & user2 < user5 & user2 < user6)
              itia::support::pose_to_pose(left_pose,left_pose_2);
            
            else if (user3 < user2 & user3 < user1 & user3 < user4 & user3 < user5 & user3 < user6)
              itia::support::pose_to_pose(left_pose,left_pose_3);
            
            else if (user4 < user2 & user4 < user3 & user4 < user1 & user4 < user5 & user4 < user6)
              itia::support::pose_to_pose(left_pose,left_pose_4);
            
            else if (user5 < user2 & user5 < user3 & user5 < user4 & user5 < user1 & user5 < user6)
              itia::support::pose_to_pose(left_pose,left_pose_5);
            
            else if (user6 < user2 & user6 < user3 & user6 < user4 & user6 < user5 & user6 < user1)
              itia::support::pose_to_pose(left_pose,left_pose_6);
        }
   } // void
   
   void left_hand_pub_fun ()
   {
      ros::Rate lp(100);
      while (ros::ok())
        {
           left_hand_pub.publish(left_pose);
           lp.sleep();
         }
   }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "tf_to_hand" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
    
    rosPS topics ( node_handle,
                   "/tf");
    
    topics.left_hand_pub_fun();
    
    itia::support::end();
    return 0;
}
