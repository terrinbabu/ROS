#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <tsupport/moveit_utils.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Bool.h"
#include <numeric>
#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_ekf/rosdyn_ekf.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <subscription_notifier/subscription_notifier.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  
    init(argc, argv,"ident_trj_filtering");
    NodeHandle nh;

    // load model for kalman filtering
    
    urdf::Model model;
    model.initParam("robot_description");

    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;

    std::string base_frame = "base_link";
    std::string tool_frame = "tool0";

    rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);

    if (!chain)
        ROS_ERROR("chain creation fail");
    
    unsigned int dof=chain->getActiveJointsNumber();
    
    // loading binary file
    
    ROS_INFO("Loading a binary file, please wait a moment...");

    std::string m_file_name = "/home/terrin/.ros/ident_trj_1558118146_rep1_JointState__joint_states.bin";

    std::ifstream is;
    is.open (m_file_name, std::ios::in | std::ios::binary );

    is.seekg ( 0, std::ios::end );
    unsigned int length = is.tellg(); 
    is.seekg ( 0, std::ios::beg );

    int number_of_sample = length / sizeof(double) / ( 1+3*dof );
    
    Eigen::MatrixXd singleFileData( (1+3*dof), number_of_sample );
    
    singleFileData.setZero();

    is.read ( (char*)singleFileData.data(), length );
    is.close();
    
    // kalman filter design 
    
    std::vector<double> diff;
    
    for (int i=0;i<(number_of_sample-1);i++)
    {
        double diff_v = singleFileData(0,i+1) - singleFileData(0,i);
        diff.push_back(diff_v);
    }
    
    double sampling_period;
    tsupport::basic::vector_val_avg(sampling_period,diff);
  
    rosdyn::EKFilter ekf(chain,sampling_period);
    
    Eigen::VectorXd q(dof);
    Eigen::VectorXd Dq(dof);
    Eigen::VectorXd tau(dof);
    Eigen::VectorXd fq(dof);
    Eigen::VectorXd fDq(dof);
    Eigen::VectorXd fDDq(dof);
    Eigen::VectorXd ftau(dof);
    
    q.setZero();
    Dq.setZero();
    tau.setZero();
    
    Eigen::MatrixXd Q(dof*2,dof*2);
    Eigen::MatrixXd R(dof*2,dof*2);

    Q.setIdentity();
    R.setIdentity();
    
    double r_q,r_dq,r_tau;
    
    if (!nh.getParam("ekf/r_q",r_q))
    r_q=0.001;
    if (!nh.getParam("ekf/r_q0",r_dq))
    r_dq=0.01;

    R.block(0,0,dof,dof)*=r_q;
    R.block(dof,dof,dof,dof)*=r_dq;
    
    if (!ekf.initialize(q,Dq,Q,R))
        ROS_ERROR("initialization fail");

    // filter q,Dq and eff
    
    Eigen::MatrixXd q_full( singleFileData.cols(), dof ),
                    Dq_full( singleFileData.cols(), dof ), 
                    eff_full( singleFileData.cols(), dof );
    
    q_full.setZero(); Dq_full.setZero(); eff_full.setZero(); 

    for ( unsigned int idxJnt=0; idxJnt<dof; idxJnt++ )
    {      
        q_full.col(idxJnt) = singleFileData.row(   idxJnt + 1 ).transpose();
        Dq_full.col(idxJnt) = singleFileData.row(  idxJnt + 1 + dof  ).transpose();
        eff_full.col(idxJnt) = singleFileData.row( idxJnt + 1 + dof * 2 ).transpose();
    }
    
    int count = 1;
    double read_rate = 1/sampling_period;
    ros::Rate lp(read_rate);
    
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState filtered_joint_state;
    
    joint_state.name=chain->getMoveableJointNames();
    joint_state.position.resize(dof);
    joint_state.velocity.resize(dof);
    joint_state.effort.resize(dof);

    filtered_joint_state.name=chain->getMoveableJointNames();
    filtered_joint_state.position.resize(dof);
    filtered_joint_state.velocity.resize(dof);
    filtered_joint_state.effort.resize(dof);
    
    Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/ident_trj_joint_states", 1000);
    Publisher filtered_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/ident_trj_kalman_filtered_joint_states", 1000);
    
    for (unsigned int i=0;i<q_full.rows();i++)
    {
        q     = q_full.row(i);
        Dq    = Dq_full.row(i);
        tau   = eff_full.row(i);
        
        ekf.update(q,Dq,tau,fq,fDq,fDDq);
        
        for (unsigned int idof=0;idof<dof;idof++)
        {
            joint_state.position.at(idof)   = fq(idof);
            joint_state.velocity.at(idof)   = fDq(idof);
            joint_state.effort.at(idof)     = tau(idof);

            filtered_joint_state.position.at(idof)   = fq(idof);
            filtered_joint_state.velocity.at(idof)   = fDq(idof);
            filtered_joint_state.effort.at(idof)     = tau(idof);
        }
        
        joint_state.header.seq = count;
        joint_state.header.stamp = ros::Time::now();
        joint_states_pub.publish(joint_state);
            
        filtered_joint_state.header.seq = count;
        filtered_joint_state.header.stamp = joint_state.header.stamp;
        filtered_joint_states_pub.publish(filtered_joint_state);
            
        // store filtered value in bin file 
        
// //         std::ofstream m_file;
// //         m_file.open("/home/terrin/.ros/filtered/ident_trj_1558118146_rep1_JointState__joint_states.bin",std::ios::out | std::ios::binary);
// // 
// //         const size_t bufsize = 1024 * 1024;
// //         std::unique_ptr<char[]> m_buf;
// //         m_buf.reset(new char[bufsize]);
// //         m_file.rdbuf()->pubsetbuf(m_buf.get(), bufsize);
// //         
// //         std::vector<double> position = {0,0,0,0,0,0};
// //         std::vector<double> velocity = {0,0,0,0,0,0};
// //         std::vector<double> effort = {0,0,0,0,0,0};
// //         
// //         tsupport::basic::EigenVectoStdVec(q,position);
// //         tsupport::basic::EigenVectoStdVec(Dq,velocity);
// //         tsupport::basic::EigenVectoStdVec(tau,effort);
// //         
// //         double time = singleFileData(0,i);
// //         
// //         m_file.write((char*) &time, sizeof(double));
// //         m_file.write((char*) &position[0], position.size() * sizeof(double));
// //         m_file.write((char*) &velocity[0], velocity.size() * sizeof(double));
// //         m_file.write((char*) &effort[0],   effort.size() * sizeof(double));
        
        count +=1;
        cout << count << "/" << q_full.rows() << endl;
        lp.sleep();
    }
    ROS_INFO("Process completed");
    return 0;
    
}
