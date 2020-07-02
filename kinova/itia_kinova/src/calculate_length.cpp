#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 
#include <itia_rutils/itia_rutils.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

#include <vector>                                                               
#include <iostream>                                                             
#include <numeric> 

class rosPS
{
private:
    
    ros::NodeHandle&                                                     nh;
    ros::Subscriber                                                      tf_sub;
    ros::Subscriber                                                      left_hand_sub;  
    
    geometry_msgs::Pose finger_pose;
    tf::StampedTransform mapTransform;
    tf::TransformListener listener;
    std::vector<double>  st_line_length_array;
    int count_1,count_2;

public:
    rosPS ( ros::NodeHandle& nh,
            const std::string& tf_topic,
            const std::string& left_hand_topic)
    : nh                ( nh )
    {
        tf_sub = nh.subscribe(tf_topic, 1000,&rosPS::tf_topic_Callback,this);
        left_hand_sub = nh.subscribe(left_hand_topic, 1000,&rosPS::left_hand_topic_Callback,this);
        count_1 = 0;
        count_2 = 0;
    }

    ~rosPS() 
    {};
    
    void tf_topic_Callback (const tf2_msgs::TFMessage& tf_info)
   {     
        std::string parent_frame_id =  tf_info.transforms[0].header.frame_id.c_str();
        std::string frame_id =  tf_info.transforms[0].child_frame_id.c_str();
        
       if (parent_frame_id == "j2n6s300_link_6")
        {
          if (frame_id == "j2n6s300_link_finger_1")
            {
              try
              {
                  listener.waitForTransform ( "/world","j2n6s300_link_finger_1", ros::Time ( 0 ),ros::Duration ( 1.0 ) );
                  listener.lookupTransform ( "/world", "j2n6s300_link_finger_1",  ros::Time ( 0 ), mapTransform );
              }
              catch ( tf::TransformException &ex )
              {
                  ROS_ERROR ( "%s",ex.what() );
                  return;
              }
                    
              itia::support::StampedTransform_to_pose(mapTransform,finger_pose);
            }
        }
   } // void

   void left_hand_topic_Callback (const geometry_msgs::Pose::ConstPtr& left_hand_info)
   {     
     
     geometry_msgs::Pose left_hand_pose;
         
     left_hand_pose.position.x = left_hand_info-> position.x;
     left_hand_pose.position.y = left_hand_info-> position.y;
     left_hand_pose.position.z = left_hand_info-> position.z;
     left_hand_pose.orientation.w = left_hand_info-> orientation.w;
     left_hand_pose.orientation.x = left_hand_info-> orientation.x;     
     left_hand_pose.orientation.y = left_hand_info-> orientation.y;
     left_hand_pose.orientation.z = left_hand_info-> orientation.z;
     
     double st_line_length;
     itia::support::dis_bet_2_pose (st_line_length,finger_pose,left_hand_pose);
     
     if (st_line_length < 1.0)
     {
       st_line_length_array.push_back(st_line_length);
       count_1 = 1;
    }
    
    if (st_line_length > 1.0 && count_1 == 1 && st_line_length_array.size() > 1)
    {
      double min,max,average;
      int min_no,max_no;
      
      average = accumulate( st_line_length_array.begin(), st_line_length_array.end(), 0.0)/st_line_length_array.size();
      itia::support::min_value_in_array (min,min_no,st_line_length_array);
      itia::support::max_value_in_array (max,max_no,st_line_length_array);
      
      
      std::cout << "size of the array : " << st_line_length_array.size() << std::endl;
      std::cout << " average : " << average << std::endl;
      std::cout << " min : " << min << std::endl;
      std::cout << " max : " << max << std::endl;
      
      st_line_length_array.clear();
    }
   } // void
   
};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "calculate_length" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
    
    rosPS topics ( node_handle,
                   "/tf",
                   "/left_hand");
    
    itia::support::end();
    return 0;
}
