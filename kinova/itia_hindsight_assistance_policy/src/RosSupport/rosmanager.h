#ifndef ROSMANAGER_H
#define ROSMANAGER_H

#include <vector>
#include <string>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "itia_human_prediction/PoseArrays.h"
#include <time.h>

class rosmanager
{
    public:
        std::vector<Eigen::Affine3d> object_poses;
        std::vector< std::vector<Eigen::Affine3d> > target_poses;
        std::vector<double> goal_dist;
        Eigen::Affine3d robot_pose;  
        geometry_msgs::Pose new_robot_pose; 
        int objectGrasped;
        
        rosmanager(ros::NodeHandle& n, 
                   const bool& _leastprob_isactive, 
                   const std::string& prob_topic, 
                   const std::string& obj_pose_topic,
                   const std::string& obj_tsr_topic,
                   const std::string& robs_topic,
                   const std::string& robp_topic,
                   const std::string& mintsrp_topic, 
                    const std::string& restart_topic);
        ~rosmanager();
        void update();
        void updateRobotPose(Eigen::MatrixXd translation, Eigen::MatrixXd rotation);
        void updateRobotTwist(Eigen::MatrixXd twist);  
        void updateMinTsrPose(Eigen::Matrix4d pose);
//         void reachedObj();
//         void reachedObj(Eigen::Affine3d minpose);
        //void waitForNewTSR();
        
    private:
        bool leastprob_isactive;        
        bool prob_topic_r;
        bool obj_topic_r;
        bool rob_topic_r;
        bool tar_topic_r;
        bool restart_topic_r;
        ros::Subscriber prob_sub;
        ros::Subscriber obj_pose_sub;
        ros::Subscriber obj_tsr_sub;
        ros::Subscriber rob_sub;  
        ros::Subscriber restart_sub;  
        ros::Publisher rob_pub;
        ros::Publisher mintsr_pub;
        void probCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void objposCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
        void objtsrCallback(const itia_human_prediction::PoseArrays::ConstPtr& msg);
        void robposCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void restartCallback(const std_msgs::Bool::ConstPtr& msg);
        void notread();
     
};

#endif // ROSMANAGER_H
