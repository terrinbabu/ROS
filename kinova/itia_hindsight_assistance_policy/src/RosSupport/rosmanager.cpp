#include "rosmanager.h"

#include <iostream>
#include <fstream>



rosmanager::rosmanager(ros::NodeHandle& n, 
                       const bool& _leastprob_isactive, 
                       const std::string& prob_topic, 
                       const std::string& obj_pose_topic,
                       const std::string& obj_tsr_topic,
                       const std::string& robs_topic,
                       const std::string& robp_topic,
                       const std::string& mintsrp_topic,
                       const std::string& restart_topic)
{   
    prob_sub = n.subscribe(prob_topic, 1000, &rosmanager::probCallback, this);
    obj_pose_sub = n.subscribe(obj_pose_topic, 1000, &rosmanager::objposCallback, this);
    obj_tsr_sub = n.subscribe(obj_tsr_topic, 1000, &rosmanager::objtsrCallback, this);
    rob_sub = n.subscribe(robs_topic, 1000, &rosmanager::robposCallback, this);
    restart_sub = n.subscribe(restart_topic, 1000, &rosmanager::restartCallback, this);
    rob_pub = n.advertise<std_msgs::Float32MultiArray>(robp_topic, 1000);
    mintsr_pub = n.advertise<std_msgs::Float32MultiArray>(mintsrp_topic, 1000);
    
    leastprob_isactive = _leastprob_isactive;    
    objectGrasped = 0;    
    obj_topic_r = false;
    notread();

//     ros::Duration(0.01).sleep();
    update();
}

rosmanager::~rosmanager()
{}

void rosmanager::notread()
{
    prob_topic_r = false;
    obj_topic_r = false;
    rob_topic_r = false;
    tar_topic_r = false;
    restart_topic_r = false;
    
}

void rosmanager::update()
{ 
    do
    {
      ros::spinOnce();
//         ros::Duration(0.05).sleep(); 
    }while(!(obj_topic_r && prob_topic_r && rob_topic_r && tar_topic_r && restart_topic_r));
    

    
    notread();
}

void rosmanager::probCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_dist.clear();
    for (size_t i=0; i<msg->data.size(); i++)
        goal_dist.push_back(msg->data[i]);
    
    prob_topic_r = true;
}

void rosmanager::objposCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (obj_topic_r)
        return;
    
    if (msg->poses.size()<=1)
    {
        ROS_ERROR("Zero or One goal!!!! POMPD is closing");
        ros::shutdown();
    }
    
    object_poses.clear();
    for (size_t i=0; i<msg->poses.size(); i++)
    {       
        Eigen::Quaternion<double> quat;
        quat.w() = (double)msg-> poses[i].orientation.w;
        quat.x() = (double)msg-> poses[i].orientation.x;
        quat.y() = (double)msg-> poses[i].orientation.y;
        quat.z() = (double)msg-> poses[i].orientation.z;
        
        Eigen::Matrix3d rotm = quat.matrix();
        Eigen::Matrix4d rottm;
        rottm <<    rotm(0,0), rotm(0,1), rotm(0,2), (double)msg-> poses[i].position.x,
                    rotm(1,0), rotm(1,1), rotm(1,2), (double)msg-> poses[i].position.y,
                    rotm(2,0), rotm(2,1), rotm(2,2), (double)msg-> poses[i].position.z,
                    0. ,0., 0., 1.;
        Eigen::Affine3d eigen(rottm);
        object_poses.push_back(eigen);
    }
    obj_topic_r = true;
}


void rosmanager::objtsrCallback(const itia_human_prediction::PoseArrays::ConstPtr& msg)
{ 
    target_poses.clear();    
    if (leastprob_isactive)
    {
        //  TO  EACH POSE ASSOCIATE THE TSR OF ALL THE REMAINING POSES
        // ON THE BASIS OF THE GOAL PROBABILITY.
        // WE WANT THE ROBOT TO REACH THE FAREST OBSTACLE   
        for (size_t i=0; i<msg->poses.size(); i++)   //PoseArray
        {   
            
            std::vector<Eigen::Affine3d> obj_tsrs;
            int minid = -1;
            double minvalue = 10000000;            
            for (int k = 0; k < goal_dist.size(); k++)
                if (goal_dist[k] < minvalue && 
                    k!=i &&
                    msg-> poses[k].poses.size() > 0 &&
                    msg-> poses[k].poses[0].position.x != -9999 )
                {
                    minvalue = goal_dist[k];
                    minid = k;
                }
            if (minid != -1)  
            {
                for (size_t j=0; j<msg->poses[minid].poses.size(); j++) 
                {
                    Eigen::Quaternion<double> quat ((double)msg-> poses[minid].poses[j].orientation.w, 
                                                    (double)msg-> poses[minid].poses[j].orientation.x, 
                                                    (double)msg-> poses[minid].poses[j].orientation.y, 
                                                    (double)msg-> poses[minid].poses[j].orientation.z);
                    
                    
                    Eigen::Matrix3d rotm = quat.matrix();
                    Eigen::Matrix4d rottm;
                    rottm <<    rotm(0,0), rotm(0,1), rotm(0,2), (double)msg-> poses[minid].poses[j].position.x,
                                rotm(1,0), rotm(1,1), rotm(1,2), (double)msg-> poses[minid].poses[j].position.y,
                                rotm(2,0), rotm(2,1), rotm(2,2), (double)msg-> poses[minid].poses[j].position.z,
                                0. ,0., 0., 1.;
                    Eigen::Affine3d eigen(rottm);
                    obj_tsrs.push_back(eigen);      
                } 
            }
            else
            {
                Eigen::Matrix4d rottm;
                rottm <<    1., 0., 0., -9999,
                            0., 1., 0., -9999,
                            0., 0., 1., -9999,
                            0. ,0., 0., 1.;
                Eigen::Affine3d eigen(rottm);
                obj_tsrs.push_back(eigen);
            }
            target_poses.push_back(obj_tsrs);
        }  
    }
    else
    {
        //  TO  EACH POSE ASSOCIATE THE TSR OF ALL THE REMAINING POSES        
        for (size_t i=0; i<msg->poses.size(); i++)   //PoseArray
        {
            std::vector<Eigen::Affine3d> obj_tsrs;
            for (size_t k=0; k<msg->poses.size(); k++) 
            {  
                if (i!=k)
                {
                    for (size_t j=0; j<msg->poses[k].poses.size(); j++) 
                    {
                        Eigen::Quaternion<double> quat ((double)msg-> poses[k].poses[j].orientation.w, 
                                                        (double)msg-> poses[k].poses[j].orientation.x, 
                                                        (double)msg-> poses[k].poses[j].orientation.y, 
                                                        (double)msg-> poses[k].poses[j].orientation.z);
                        
                        
                        Eigen::Matrix3d rotm = quat.matrix();
                        Eigen::Matrix4d rottm;
                        rottm <<    rotm(0,0), rotm(0,1), rotm(0,2), (double)msg-> poses[k].poses[j].position.x,
                                    rotm(1,0), rotm(1,1), rotm(1,2), (double)msg-> poses[k].poses[j].position.y,
                                    rotm(2,0), rotm(2,1), rotm(2,2), (double)msg-> poses[k].poses[j].position.z,
                                    0. ,0., 0., 1.;
                        Eigen::Affine3d eigen(rottm);
                        obj_tsrs.push_back(eigen);      
                    }
                }
            }
            target_poses.push_back(obj_tsrs);
        }   
    }
    tar_topic_r = true;
}


void rosmanager::robposCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    Eigen::Quaternion<double> quat ((double)msg-> orientation.w, (double)msg-> orientation.x, 
                                    (double)msg-> orientation.y, (double)msg-> orientation.z);
    Eigen::Matrix3d rotm = quat.matrix();
    Eigen::Matrix4d rottm;
    rottm <<    rotm(0,0), rotm(0,1), rotm(0,2), (double)msg-> position.x,
                rotm(1,0), rotm(1,1), rotm(1,2), (double)msg-> position.y,
                rotm(2,0), rotm(2,1), rotm(2,2), (double)msg-> position.z,
                0. ,0., 0., 1.;
    Eigen::Affine3d rp (rottm);            
    robot_pose = rp;
    
    rob_topic_r = true;
}

void rosmanager::restartCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)  //tsr change has to arrive toghter or before I enter this cycle
    {
        objectGrasped = 1;
    }
    
    restart_topic_r = true;
}

void rosmanager::updateRobotPose(Eigen::MatrixXd translation, Eigen::MatrixXd rotation)
{
    geometry_msgs::Pose rp;
    rp.position.x = (float)translation(0,0); 
    rp.position.y = (float)translation(1,0); 
    rp.position.z = (float)translation(2,0);   
    rp.orientation.w = (float)rotation(3,0); 
    rp.orientation.x = (float)rotation(0,0); 
    rp.orientation.y = (float)rotation(1,0); 
    rp.orientation.z = (float)rotation(2,0); 
    rob_pub.publish(rp); 
    ros::spinOnce();
}

void rosmanager::updateRobotTwist(Eigen::MatrixXd twist)
{
    std_msgs::Float32MultiArray rt;
    for (int i=0; i< twist.rows(); i++)    
        rt.data.push_back(twist(i,0));

    std::cout<<" rt "<< rt <<std::endl;
    
    rob_pub.publish(rt);    
    ros::spinOnce();
}

void rosmanager::updateMinTsrPose(Eigen::Matrix4d pose)
{
    std_msgs::Float32MultiArray mp;
    for (int i=0; i< pose.rows(); i++)
        for (int j=0; j< pose.cols(); j++)
            mp.data.push_back(pose(i,j));
    
    mintsr_pub.publish(mp);    
    ros::spinOnce();
}

