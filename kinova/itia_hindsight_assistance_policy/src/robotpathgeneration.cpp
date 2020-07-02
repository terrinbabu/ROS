//Code to test out the assistance policy
#include "AssistancePolicyBase.h"
#include "HuberRotationInstance.h"
#include "GoalPolicy.h"

#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>

#include "RosSupport/rosmanager.h"
#include "ros/ros.h"

#include <vector>

int main (int argc, char **argv) 
{
    try 
    {   
        ros::init(argc, argv, "hindsight");
        ros::NodeHandle n;
        
        bool _leastprob_isactive;
        ros::param::get("~leastprob_isactive", _leastprob_isactive); 
        
        rosmanager topics ( n, 
                            _leastprob_isactive,
                            "/skel/prob_goal_user", 
                            "/env_obj/pos", 
                            "/env_obj/tsr",
                            "/herb/active_eep", 
                            "/herb/next_eep",
                            "/herb/mintsr",
                            "/restart");
        
        assistance_policy::AssistancePolicy<assistance_policy::HuberRotationInstance> assist_policy(topics.target_poses, topics.object_poses, topics.goal_dist);

        int cnt_errors = 0;
        
        while(ros::ok())
        { 
            
            if (topics.objectGrasped == 0)
            {  
                assist_policy.UpdatePoseAndAction(topics.robot_pose, Eigen::Vector3d::Zero());
                
                
                if(topics.goal_dist.size() > 0 )
                {	    
                    assistance_policy::HuberRotationInstance::ActionVector twist = assist_policy.GetAssistedAction_Cached(topics.goal_dist);
		    
                    size_t max_index = std::max_element(topics.goal_dist.begin(), topics.goal_dist.end()) - topics.goal_dist.begin();
		           
                    Eigen::Affine3d minpose = assist_policy.GetMinValuePose_OneGoal(max_index);  

                    topics.updateRobotTwist(twist.block<6,1>(0,0));		
                    topics.updateMinTsrPose(minpose.matrix());
                    
//                     ros::Duration(0.01).sleep();  //200 hz
                    topics.update();
                }
                else
                {
                   ROS_ERROR( "Error (%d), the size of the vector 'goal_dist' is zero", cnt_errors );
                }
            }

            else if (topics.objectGrasped == 1)
            {
                //topics.update();
                assist_policy.Reinitialize( topics.target_poses, 
                                            topics.object_poses, 
                                            topics.goal_dist);
                topics.objectGrasped = 0;
            }// else_if
            
        }// while
    }// try
    catch(std::exception& e)
    {
        std::cerr << __FILE__  << " ERROR CAUGHT "  << e.what() << std::endl;
        return -1;
    }
    std::cout << "*********************"<< __FILE__  << " FINISH" << std::endl;
    return 0;
}