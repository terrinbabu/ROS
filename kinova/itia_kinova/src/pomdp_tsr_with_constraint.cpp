#include <chrono> 

#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 
#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

#include <itia_human_prediction/PoseArrays.h>
#include <std_msgs/Float32MultiArray.h>
class rosPS
{
private:
    
    ros::NodeHandle&                                        nh;
    
    ros::Subscriber                                         planning_scene_object_pose_sub;
    ros::Subscriber                                         goal_tsr_sub;
    ros::Subscriber                                         prob_goal_user_sub;
    ros::Publisher                                          env_obj_tsr_pub;

    geometry_msgs::PoseArray                                object_poses;
    std::vector<double>                                     goal_dist;
    std::vector<int>                                        constraint_array;
    std::vector<int>                                        Human_action_first;
    int once_pub,first_time,clock_for_goal_dist;
    std::chrono::_V2::system_clock::time_point t_start;
    
public:
  
    rosPS ( ros::NodeHandle& nh,
            const std::string& planning_scene_object_pose,
            const std::string& env_obj_tsr,
            const std::string& goal_tsr,
            const std::string& prob_goal_user)
    : nh                      ( nh )
    {
        first_time = 0;
        once_pub = 0;
        planning_scene_object_pose_sub = nh.subscribe(planning_scene_object_pose, 1000, &rosPS::planning_scene_object_pose_Callback,this);
        prob_goal_user_sub = nh.subscribe(prob_goal_user, 1000, &rosPS::prob_goal_user_Callback,this);
        env_obj_tsr_pub = nh.advertise<itia_human_prediction::PoseArrays>("/env_obj/tsr", 1000);
        goal_tsr_sub = nh.subscribe(goal_tsr, 1000, &rosPS::goal_tsr_Callback,this);
    }
    
    ~rosPS() 
    {};
    
    void planning_scene_object_pose_Callback(const geometry_msgs::PoseArray::ConstPtr& planning_scene_object_pose_info)
    {
    
    if (once_pub==0)
      {  
        object_poses.poses.clear();
        constraint_array.clear();
              
            for (int i=0;i<=10;i++)
            {
                int constraint_id;
                std::stringstream string_constraint_id;
                string_constraint_id << "/object_info/object_"<<i+1 <<"/precedence_constraint";
                std::string constraint_id_param = string_constraint_id.str();
                if (nh.getParam(constraint_id_param,constraint_id))
                constraint_array.push_back(constraint_id);

                if (!nh.getParam(constraint_id_param,constraint_id))
                break;
             }
          
          for (size_t i=0; i<planning_scene_object_pose_info->poses.size(); i++)
          {
              geometry_msgs::Pose object_pose;

              std::vector<double> obj_pose =  { planning_scene_object_pose_info-> poses[i].position.x,
                                                planning_scene_object_pose_info-> poses[i].position.y,
                                                planning_scene_object_pose_info-> poses[i].position.z };
              std::vector<double> obj_orient ={ planning_scene_object_pose_info-> poses[i].orientation.w,
                                                planning_scene_object_pose_info-> poses[i].orientation.x,
                                                planning_scene_object_pose_info-> poses[i].orientation.y,
                                                planning_scene_object_pose_info-> poses[i].orientation.z };

              itia::support::fromVecToPose(obj_pose,obj_orient,object_pose);
              object_poses.poses.push_back(object_pose);
          } // for
        once_pub = 1;
       } // if
    
    }// void
    
    void goal_tsr_Callback ( const geometry_msgs::Pose::ConstPtr& goal_tsr_info )
    {
      
     geometry_msgs::Pose goal_tsr_;
     goal_tsr_.position.x = goal_tsr_info-> position.x;
     goal_tsr_.position.y = goal_tsr_info-> position.y;
     goal_tsr_.position.z = goal_tsr_info-> position.z;
     goal_tsr_.orientation.w = goal_tsr_info-> orientation.w;
     goal_tsr_.orientation.x = goal_tsr_info-> orientation.x;     
     goal_tsr_.orientation.y = goal_tsr_info-> orientation.y;
     goal_tsr_.orientation.z = goal_tsr_info-> orientation.z;
     
     std::vector<double> dis_obj_goal_array;
     for (size_t i=0; i<object_poses.poses.size(); i++)
     {
       double dis_obj_goal;
       geometry_msgs::Pose object_pose;
       object_pose.position.x = object_poses.poses[i].position.x;
       object_pose.position.y = object_poses.poses[i].position.y;
       object_pose.position.z = object_poses.poses[i].position.z;
       object_pose.orientation.w = object_poses.poses[i].orientation.w;
       object_pose.orientation.x = object_poses.poses[i].orientation.x;
       object_pose.orientation.y = object_poses.poses[i].orientation.y;
       object_pose.orientation.z = object_poses.poses[i].orientation.z;
       
       itia::support::dis_bet_2_pose(dis_obj_goal,goal_tsr_,object_pose);
       dis_obj_goal_array.push_back(dis_obj_goal);
     }
     itia::support::printvectordouble("dis_obj_goal_array",dis_obj_goal_array);
     
     double min_dis;
     int min_dis_obj;
     itia::support::min_value_in_array(min_dis,min_dis_obj,dis_obj_goal_array);
     
     std::cout << min_dis_obj << std::endl;
     object_poses.poses[min_dis_obj].position.x = 999;
     object_poses.poses[min_dis_obj].position.y = 999;
     object_poses.poses[min_dis_obj].position.z = 999;
    }
    
    void prob_goal_user_Callback (const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        goal_dist.clear();
        
        for (size_t i=0; i<msg->data.size(); i++)
            goal_dist.push_back(msg->data[i]);
        
        itia_human_prediction::PoseArrays all_tsr;
        Human_action_first.clear();
        
        for (size_t i=0; i<object_poses.poses.size(); i++)
        {
          if (constraint_array[i] != 1)
          {
              geometry_msgs::PoseArray object_tsr;
              geometry_msgs::Pose tsr;

              tsr.position.x = object_poses.poses[i].position.x;
              tsr.position.y = object_poses.poses[i].position.y;
              tsr.position.z = object_poses.poses[i].position.z + 0.4;
        
              tsr.orientation.x = -0.503124177456;
              tsr.orientation.y =  0.499893963337;
              tsr.orientation.z = -0.498644709587;
              tsr.orientation.w =  0.498322844505;
       
              object_tsr.poses.push_back(tsr);
              all_tsr.poses.push_back(object_tsr);
          }
          else if (constraint_array[i] == 1)
          {
              Human_action_first.push_back(i);
              geometry_msgs::PoseArray object_tsr;
              geometry_msgs::Pose tsr;

              tsr.position.x = 999.00;
              tsr.position.y = 999.00;
              tsr.position.z = 999.00;
        
              tsr.orientation.x = -0.503124177456;
              tsr.orientation.y =  0.499893963337;
              tsr.orientation.z = -0.498644709587;
              tsr.orientation.w =  0.498322844505;
       
              object_tsr.poses.push_back(tsr);
              all_tsr.poses.push_back(object_tsr);
          }  
        } // for
      
      if(first_time == 0)
        {
              for (size_t i=0; i<Human_action_first.size(); i++)
              {
                if(goal_dist[Human_action_first[i]] > 0.9)
                  {
                    std::cout << "---clock start--- " << std::endl ;
                    t_start = std::chrono::high_resolution_clock::now();
                    first_time = 1;
                    clock_for_goal_dist = Human_action_first[i];
                  }
              }
        }
      
      if(first_time == 1)
      {
            if(goal_dist[clock_for_goal_dist] < 0.9)
              {
                auto t_end = std::chrono::high_resolution_clock::now();
                double duration = std::chrono::duration<double, std::milli>(t_end-t_start).count();
                std::cout << "duration of human action (on constraint in ms): " << duration << std::endl ;
                
                if (duration > 12000)
                {
                   std::cout << "Human action on object "<< clock_for_goal_dist+1 << " completed " << std::endl ;
                  all_tsr.poses[clock_for_goal_dist].poses[0].position.x = object_poses.poses[clock_for_goal_dist].position.x;
                  all_tsr.poses[clock_for_goal_dist].poses[0].position.y = object_poses.poses[clock_for_goal_dist].position.y;
                  all_tsr.poses[clock_for_goal_dist].poses[0].position.z = object_poses.poses[clock_for_goal_dist].position.z + 0.4;
                  constraint_array[clock_for_goal_dist] = 2;
                }
                first_time = 0;
              }
      }
      ros::Rate poll_rate(100);
      while(env_obj_tsr_pub.getNumSubscribers() == 0)
      poll_rate.sleep();
      
//       std::cout << all_tsr << std::endl;
      env_obj_tsr_pub.publish(all_tsr);
      
      
    }// void
    
    void tsr_pub ()
    {
      itia_human_prediction::PoseArrays all_tsr;
      
        for (size_t i=0; i<object_poses.poses.size(); i++)
        {
              geometry_msgs::PoseArray object_tsr;
              geometry_msgs::Pose tsr;

              tsr.position.x = object_poses.poses[i].position.x;
              tsr.position.y = object_poses.poses[i].position.y;
              tsr.position.z = object_poses.poses[i].position.z + 0.4;
        
              tsr.orientation.x = -0.503124177456;
              tsr.orientation.y =  0.499893963337;
              tsr.orientation.z = -0.498644709587;
              tsr.orientation.w =  0.498322844505;
       
              object_tsr.poses.push_back(tsr);
              all_tsr.poses.push_back(object_tsr);
        } // for
      
      ros::Rate poll_rate(100);
      while(env_obj_tsr_pub.getNumSubscribers() == 0)
      poll_rate.sleep();
      
      std::cout << all_tsr << std::endl;
      env_obj_tsr_pub.publish(all_tsr);
      
    } // void
}; // class

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );
    ros::init ( argc, argv, "pomdp_tsr_with_constraint" );
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();

    rosPS topics ( nh,
                   "/env_obj/pos",
                   "/env_obj/tsr",
                   "/goal_tsr",
                   "/skel/prob_goal_user");

    itia::support::end();
    return 0;
}