#include <iostream>
#include <string>
#include <numeric>
#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <tsupport/basic.h>
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
using namespace std::chrono;

class rosPS
{
private:
    
    NodeHandle&                                        nh;
    Subscriber                                         joint_states_sub;
    Publisher                                          filtered_joint_states_pub;

    double count = 0;
    double sampling_period, seq1,seq2;
    high_resolution_clock::time_point t1,t2;
    
    std::vector < std::shared_ptr<rosdyn::EKFilter> > ekf;
    sensor_msgs::JointState filtered_joint_state;

public:
  
    rosPS ( NodeHandle& nh,
            const std::string& joint_states)
    : nh  ( nh )
    {
        joint_states_sub = nh.subscribe(joint_states, 1000, &rosPS::joint_states_Callback,this);
        filtered_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/kalman_filtered_joint_states", 1000);
    }

    ~rosPS() 
    {};
    
    void joint_states_Callback(const sensor_msgs::JointState& joint_states_info)
    {
    
        unsigned int dof=joint_states_info.name.size();

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
        
        for (unsigned int i=0;i<dof;i++)
        {
            q(i)     = joint_states_info.position.at(i);
            Dq(i)    = joint_states_info.velocity.at(i);
            tau(i)   = joint_states_info.effort.at(i);
        }

        if (count == 0)
        {
            t1 = high_resolution_clock::now();
            seq1 = joint_states_info.header.seq;
        }

        if (count == 20)
        {
            t2 = high_resolution_clock::now();
            seq2 = joint_states_info.header.seq;
            
            sampling_period = ( duration_cast<duration<double>>(t2 - t1).count() ) / (seq2 - seq1);
            
            urdf::Model model;
            model.initParam("robot_description");

            Eigen::Vector3d grav;
            grav << 0, 0, -9.806;

            std::string base_frame = "base_link";
            std::string tool_frame = "tool0";

            rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
            
            if (!chain)
                ROS_ERROR("chain creation fail");

            std::shared_ptr <rosdyn::EKFilter> tmp_ekf(new rosdyn::EKFilter(chain,sampling_period));
            ekf.push_back(tmp_ekf);
            
            Eigen::MatrixXd Q(dof*2,dof*2);
            Eigen::MatrixXd R(dof*2,dof*2);

            Q.setIdentity();
            R.setIdentity();
            
            double r_q,r_dq; // higher the value higher amount of filtering
            
            if (!nh.getParam("/kalman_filter_example/r_q",r_q))
            r_q=0.001;
            if (!nh.getParam("/kalman_filter_example/r_dq",r_dq))
            r_dq=0.1;
            
            cout << "r_q :=" << r_q << endl;
            cout << "r_dq :=" << r_dq << endl;

            cout << "Q values : " << endl << Q << endl;
            cout << "R values : " << endl << R << endl;
            
            R.block(0,0,dof,dof)*=r_q;
            
            cout << "R considering r_q : " << endl << R << endl;
            
            R.block(dof,dof,dof,dof)*=r_dq;

            cout << "R considering r_q and r_dq : " << endl << R << endl;

            if (!ekf.at(0)->initialize(q,Dq,Q,R))
                ROS_ERROR("initialization fail");
  
            filtered_joint_state.name=chain->getMoveableJointNames();
            filtered_joint_state.position.resize(dof);
            filtered_joint_state.velocity.resize(dof);
            filtered_joint_state.effort.resize(2*dof);
            
        }
        
        if (count > 20)
        {
            ekf.at(0)->update(q,Dq,tau,fq,fDq,fDDq);

            for (unsigned int idof=0;idof<dof;idof++)
            {
                filtered_joint_state.position.at(idof)   = fq(idof);
                filtered_joint_state.velocity.at(idof)   = fDq(idof);
                filtered_joint_state.effort.at(idof)     = tau(idof);
                filtered_joint_state.effort.at(idof+dof) = fDDq(idof);
            }
            
            filtered_joint_state.header.seq = count-20;
            filtered_joint_state.header.stamp = joint_states_info.header.stamp;
            //filtered_joint_state.header.stamp = ros::Time::now();
            filtered_joint_states_pub.publish(filtered_joint_state);
        }
        count += 1;
        
    } // void
}; // class

int main ( int argc, char **argv )
{
    init ( argc, argv, "kalman_filter_example" );
    NodeHandle nh;
  
    rosPS topics ( nh,
                   "/joint_states");
    spin();
    waitForShutdown();
    return 0;
}
