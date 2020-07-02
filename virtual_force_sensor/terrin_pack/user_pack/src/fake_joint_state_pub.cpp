#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosdyn_core/urdf_parser.h>
#include <subscription_notifier/subscription_notifier.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_joint_state_pub");
  ros::NodeHandle nh;
  
  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd tau(6);
  tau.setZero();
  
  double rate_hz = 125;
  
  ros::Rate rate(rate_hz);

  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_rec(nh,"/joint_states",1);
  
  ROS_INFO("waiting for topic");
  if (!js_rec.waitForANewData(ros::Duration(1000)))
    return 0;

  int js_size = js_rec.getData().name.size();
  
  sensor_msgs::JointState fake_js;
  ros::Publisher fake_js_pub=nh.advertise<sensor_msgs::JointState>("/fake_js",1);
  
  ROS_INFO("start");
  
  double y1_last = 0;
  
  while (ros::ok())
  {
    
    fake_js=js_rec.getData();
    
    double time_now =ros::Time::now().toSec();  // Sine Wave y = A +[a*sine(2*pi*f*t)]
    double A = 0; double a = M_PI; double f = 2.5;
    double y = A + (a*sin(2*M_PI*f*time_now));
    
    double Dqq = 1;
    double y1 = y1_last + (Dqq/rate_hz);

    y1_last = y1;
    
    if (y1 > 10)
        return 0;
    
    q(0) = 0; q(1) = -0.5*M_PI; q(2) = -0.5*M_PI; q(3) = -0.5*M_PI; q(4) =0; q(5) =0;
    Dq(0) = 0; Dq(1) = 0; Dq(2) = 0; Dq(3) = 0; Dq(4) =0; Dq(5) =0;
    
    for (unsigned int idx=0;idx<js_size;idx++)
    {
       fake_js.position.at(idx)=q(idx);
       fake_js.velocity.at(idx)=Dq(idx);
       
      if (idx == 0)
      {
            fake_js.position.at(idx)=y;
            fake_js.velocity.at(idx)=y1;
      }      
    }
    
    fake_js_pub.publish(fake_js);
    
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
