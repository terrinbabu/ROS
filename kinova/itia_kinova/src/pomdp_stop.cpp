#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h>
#include <itia_rutils/itia_rutils.h>

 #include <std_msgs/Float32MultiArray.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

#include <kinova_msgs/PoseVelocity.h>
 
#include "std_msgs/Bool.h"

#include <itia_human_prediction/PoseArrays.h>
 
class rosPS
{
private:
    
    ros::NodeHandle&                                        nh;
    
    ros::Subscriber                                         env_obj_tsr_sub;
    ros::Subscriber                                         prob_goal_user_sub;
    ros::Publisher                                          restart_pub;
    ros::Publisher                                          pomdp_stop_pub;
    
    
    std::vector<double>                                     goal_dist;
    std::vector<double>                                     work_done_info;
    
    int stop;
    
public:
  
    rosPS ( ros::NodeHandle& nh,
            const std::string& env_obj_tsr,
            const std::string& prob_goal_user,
            const std::string& restart,
            const std::string& pomdp_stop)
    : nh                      ( nh )
    {   
        stop = 0;
        prob_goal_user_sub = nh.subscribe(prob_goal_user, 1000, &rosPS::prob_goal_user_Callback, this);
        env_obj_tsr_sub = nh.subscribe(env_obj_tsr, 1000, &rosPS::env_obj_tsr_Callback,this);
        restart_pub = nh.advertise<std_msgs::Bool> ( restart, 100 );
        pomdp_stop_pub = nh.advertise<std_msgs::Bool> ( pomdp_stop, 100 );
    }

    ~rosPS() 
    {};
    
    void prob_goal_user_Callback (const std_msgs::Float32MultiArray::ConstPtr& msg)
    { 
        goal_dist.clear();
        
        for (size_t i=0; i<msg->data.size(); i++)
            goal_dist.push_back(msg->data[i]);
    }// void
    
    void env_obj_tsr_Callback(const itia_human_prediction::PoseArrays::ConstPtr& msg)
    {
      work_done_info.clear();
      int no_of_work_done = 0;
      
      for (int k = 0; k < goal_dist.size(); k++)
      {
        double work_done = 1.0;
        double work_un_done = 0.0;
        
        if ( msg-> poses[k].poses[0].position.x == 999 )
        {
          work_done_info.push_back(work_done);
          no_of_work_done = no_of_work_done+1;
        }
        else
        {
          work_done_info.push_back(work_un_done);
        }
      }
      
      itia::support::printvectordouble("work_done_info",work_done_info);
       
      std_msgs::Bool pomdp_non_stop;
      pomdp_non_stop.data = false;
      std_msgs::Bool pomdp_stop;
      pomdp_stop.data = true;
      
      double max_goal_dist,min_work_done_info;
      int max_no_goal_dist,min_no_work_done_info;
       
      if (no_of_work_done == goal_dist.size() - 1 )
      {
        
        itia::support::max_value_in_array(max_goal_dist,max_no_goal_dist,goal_dist);
        itia::support::min_value_in_array(min_work_done_info,min_no_work_done_info,work_done_info);
        
        if (max_no_goal_dist == min_no_work_done_info) 
        {
          stop = 1;
          std::cout << "pomdp_stop_pub 1 : pomdp_stop" << std::endl;
          pomdp_stop_pub.publish (pomdp_stop);
        }
        else
        {
          if(stop=1)
          {
            std_msgs::Bool restart_bool;
            restart_bool.data = true;
            restart_pub.publish (restart_bool);
            stop = 0;
          }
          
         std::cout << "pomdp_stop_pub 2 : pomdp_non_stop" << std::endl;
          pomdp_stop_pub.publish (pomdp_non_stop);

        }
      }
      else if (no_of_work_done == goal_dist.size())
      {
          stop = 1;
          std::cout << "pomdp_stop_pub 3 : pomdp_stop" << std::endl;
          pomdp_stop_pub.publish (pomdp_stop);  
      }
      else
      {
        
          if(stop=1)
          {
            std_msgs::Bool restart_bool;
            restart_bool.data = true;
            restart_pub.publish (restart_bool);
            stop = 0;
          }
          
        std::cout << "pomdp_stop_pub 4 : pomdp_non_stop" << std::endl;
        pomdp_stop_pub.publish (pomdp_non_stop);
      }
      
    }// void
    
}; // class

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );
    ros::init ( argc, argv, "pomdp_stop" );
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    
    std::cout << "-------1------- \n";
    
    rosPS topics ( nh,
                   "/env_obj/tsr",
                   "/skel/prob_goal_user",
                   "/restart",
                   "/pomdp_stop");
    
    std::cout << "------2------- \n";
    itia::support::end();
    return 0;
}