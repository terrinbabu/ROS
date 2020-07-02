#include <iostream>
#include <fstream>

#include <moveit_msgs/DisplayTrajectory.h>

#include <itia_futils/itia_futils.h> 

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

using namespace std;

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "hand_dis" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (10.0);
    
    std::vector<double> obj_pose_1 = { 0.4,-0.5,0.08}; 
    std::vector<double> obj_pose_2 = { 0.2,-0.5,0.08}; 
    std::vector<double> obj_pose_3 = { 0.6,-0.5,0.08}; 
    
    std::vector<double> tsr_orentation = { 0.49832,-0.5031,0.49989,-0.4986};
              
    itia::kinova_utils::move("1a",obj_pose_1[0],obj_pose_1[1],obj_pose_1[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("1b",obj_pose_1[0],obj_pose_1[1],obj_pose_1[2]+0.4-0.31,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("1c",obj_pose_1[0],obj_pose_1[1],obj_pose_1[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    
    itia::kinova_utils::move("2a",obj_pose_2[0],obj_pose_2[1],obj_pose_2[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("2b",obj_pose_2[0],obj_pose_2[1],obj_pose_2[2]+0.4-0.31,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("2c",obj_pose_2[0],obj_pose_2[1],obj_pose_2[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    
    itia::kinova_utils::move("3a",obj_pose_3[0],obj_pose_3[1],obj_pose_3[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("3b",obj_pose_3[0],obj_pose_3[1],obj_pose_3[2]+0.4-0.31,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    itia::kinova_utils::move("3c",obj_pose_3[0],obj_pose_3[1],obj_pose_3[2]+0.4,tsr_orentation[0],tsr_orentation[1],tsr_orentation[2],tsr_orentation[3]);
    sleep(10.0);
    
    itia::support::end();
    return 0;
}