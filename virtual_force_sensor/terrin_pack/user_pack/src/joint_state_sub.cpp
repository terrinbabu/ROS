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

int main ( int argc, char **argv )
{
    init ( argc, argv, "joint_state_sub" );
    NodeHandle nh;
    ros::Rate rate(125);
    
    ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_rec(nh,"/joint_states",1);
    
    ROS_INFO("waiting for topic");
    if (!js_rec.waitForANewData(ros::Duration(1000)))
        return 0;
    
    Eigen::VectorXd q(js_rec.getData().name.size());
    Eigen::VectorXd Dq(js_rec.getData().name.size());
    Eigen::VectorXd tau(js_rec.getData().name.size());
    
    while (ros::ok())
    {
        for (unsigned int idx=0;idx<js_rec.getData().name.size();idx++)
            {
                q(idx)=js_rec.getData().position.at(idx);
                Dq(idx)=js_rec.getData().velocity.at(idx);
                tau(idx)=js_rec.getData().effort.at(idx);
            }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    waitForShutdown();
    return 0;
}
