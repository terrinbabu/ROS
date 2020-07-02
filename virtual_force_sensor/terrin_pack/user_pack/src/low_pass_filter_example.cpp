#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <complex>
#include <cmath>
#include <iterator>
#include <fstream>
#include <tsupport/basic.h>
#include <tsupport/moveit_utils.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <ctime>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv,"low_pass_filter_example");
    NodeHandle nh;
    AsyncSpinner spinner(1);
    spinner.start(); 
    sleep(3.0);
    
    ros::Publisher sine_pub = nh.advertise<std_msgs::Float64>("/sine_wave", 1000);
    ros::Publisher fsine_pub = nh.advertise<std_msgs::Float64>("/fsine_wave", 1000);
    
    double natural_frequency = 2*M_PI*50; // [rad/s]
    double sampling_period = 0.001; // s
    std::cout << "natural_frequency : " << natural_frequency << std::endl;
    eigen_control_toolbox::FirstOrderLowPass lpf(natural_frequency,sampling_period);
    
    double y=0;
    double yf=0;
    lpf.setStateFromLastIO(y, yf);
    
    ros::Rate lp(1./sampling_period);
    
    double a1 = 20;
    double a2 = 40;
    double a3 = 100;
    
    double f1 = 20;
    double f2 = 30;
    double f3 = 200;
    
    while (ros::ok())
    {
    
        double time_now =ros::Time::now().toSec();  // Sine Wave y = A +[a*sine(2*pi*f*t)]
        
        double y = (a1*sin(2*M_PI*f1*time_now)) + (a2*sin(2*M_PI*f2*time_now)) + (a3*sin(2*M_PI*f3*time_now));
        
        std_msgs::Float64 sine_value_;
        sine_value_.data = y;
        sine_pub.publish(sine_value_);
        
        yf=lpf.update(y);
        
        std_msgs::Float64 fsine_value_;
        fsine_value_.data = yf;
        fsine_pub.publish(fsine_value_);
        lp.sleep(); 
    }

    
}
