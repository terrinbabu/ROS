#include <iostream>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <geometry_msgs/WrenchStamped.h>
#include <chrono>
#include <math.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

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
    double read_fz,natural_frequency,sampling_period, sampling_frequency, seq1,seq2;
    high_resolution_clock::time_point t1,t2;
        
    std::vector < std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> > lpfq;
    std::vector < std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> > lpfDq;
    std::vector < std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> > lpftau;

public:
  
    rosPS ( NodeHandle& nh,
            const std::string& joint_states)
    : nh  ( nh )
    {
        joint_states_sub = nh.subscribe(joint_states, 1000, &rosPS::joint_states_Callback,this);
        filtered_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/filtered_joint_states", 1000);
    }

    ~rosPS() 
    {};
    
    void joint_states_Callback(const sensor_msgs::JointState& joint_states_info)
    {
        
        if (count == 0)
        {
            t1 = high_resolution_clock::now();
            seq1 = joint_states_info.header.seq;
            bool read_feq =nh.getParam("/joint_state_filtering/hz",read_fz);
            natural_frequency = 2*M_PI*read_fz; // [rad/s]
            cout << "natural_frequency : " << read_fz << endl;
        }

        if (count == 20)
        {
            t2 = high_resolution_clock::now();
            seq2 = joint_states_info.header.seq;
            
            sampling_period = ( duration_cast<duration<double>>(t2 - t1).count() ) / (seq2 - seq1);
            
            sampling_frequency = 1/sampling_period ;
            cout << "sampling_frequency : " << sampling_frequency << endl;
            
            for (unsigned int i=0;i<joint_states_info.name.size();i++)
            {
                std::shared_ptr <eigen_control_toolbox::FirstOrderLowPass> tmp_lpfq(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period)); 
                lpfq.push_back(tmp_lpfq);
                lpfq.back()->setStateFromLastIO(joint_states_info.position.at(i), joint_states_info.position.at(i) );
                
                std::shared_ptr <eigen_control_toolbox::FirstOrderLowPass> tmp_lpfDq(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period)); 
                lpfDq.push_back(tmp_lpfDq);
                lpfDq.back()->setStateFromLastIO(joint_states_info.velocity.at(i), joint_states_info.velocity.at(i));
                
                std::shared_ptr <eigen_control_toolbox::FirstOrderLowPass> tmp_lpftau(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period)); 
                lpftau.push_back(tmp_lpftau);
                lpftau.back()->setStateFromLastIO(joint_states_info.effort.at(i), joint_states_info.effort.at(i));
            }
        }
        
        if (count > 20)
        {
            Eigen::VectorXd fq(joint_states_info.name.size());
            fq.setZero();
            Eigen::VectorXd fDq(joint_states_info.name.size());
            fDq.setZero();
            Eigen::VectorXd ftau(joint_states_info.name.size());
            ftau.setZero();
            
            for (unsigned int i=0;i<joint_states_info.name.size();i++)
            {
                fq(i)     = lpfq.at(i)->update( joint_states_info.position.at(i));
                fDq(i)    = lpfDq.at(i)->update(joint_states_info.velocity.at(i));
                ftau(i)   = lpftau.at(i)->update(joint_states_info.effort.at(i));
            }
                      
            sensor_msgs::JointState filtered_joint_state;
  
            filtered_joint_state.name.resize(6);
            filtered_joint_state.position.resize(6);
            filtered_joint_state.velocity.resize(6);
            filtered_joint_state.name.at(0) = "shoulder_pan_joint";
            filtered_joint_state.name.at(1) = "shoulder_lift_joint";
            filtered_joint_state.name.at(2) = "elbow_joint";
            filtered_joint_state.name.at(3) = "wrist_1_joint";
            filtered_joint_state.name.at(4) = "wrist_2_joint";
            filtered_joint_state.name.at(5) = "wrist_3_joint";

            filtered_joint_state.header.seq = count-20;
            filtered_joint_state.header.stamp = ros::Time::now();

            std::vector<double> fq_v   = {0,0,0,0,0,0};
            std::vector<double> fDq_v  = {0,0,0,0,0,0};
            std::vector<double> ftau_v = {0,0,0,0,0,0};

            fq_v[0] = fq(0); fDq_v[0] = fDq(0); ftau_v[0] = ftau(0);
            fq_v[1] = fq(1); fDq_v[1] = fDq(1); ftau_v[1] = ftau(1);
            fq_v[2] = fq(2); fDq_v[2] = fDq(2); ftau_v[2] = ftau(2);
            fq_v[3] = fq(3); fDq_v[3] = fDq(3); ftau_v[3] = ftau(3);
            fq_v[4] = fq(4); fDq_v[4] = fDq(4); ftau_v[4] = ftau(4);
            fq_v[5] = fq(5); fDq_v[5] = fDq(5); ftau_v[5] = ftau(5);

            filtered_joint_state.position = fq_v;
            filtered_joint_state.velocity = fDq_v;
            filtered_joint_state.effort   = ftau_v;

            filtered_joint_states_pub.publish(filtered_joint_state);

        }
        count += 1;
        
    } // void
}; // class

int main ( int argc, char **argv )
{
    init ( argc, argv, "joint_state_filtering" );
    NodeHandle nh;
    rosPS topics ( nh,
                   "/joint_states");
    spin();
    waitForShutdown();
    return 0;
}
