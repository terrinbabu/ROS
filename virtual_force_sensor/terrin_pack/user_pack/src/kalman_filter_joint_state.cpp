#include <eigen3/Eigen/StdVector>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_joint_state");
  ros::NodeHandle nh;
  
  double rate_hz = 125;
  double t = 1/rate_hz;
  
  ros::Rate rate(rate_hz);

  double P_gain,D_gain,I_gain,sd_q,sd_Dq,sd_DDq,sd_tau,error_zero_tolerance;
  double P_gain_tau,D_gain_tau,I_gain_tau,error_zero_tolerance_tau;
  
  nh.getParam("/std_dev/q",sd_q);
  nh.getParam("/std_dev/Dq",sd_Dq);
  nh.getParam("/std_dev/DDq",sd_DDq);
  nh.getParam("/std_dev/tau",sd_tau);
  
  nh.getParam("/motion_KF/error_zero_tolerance",error_zero_tolerance);
  nh.getParam("/motion_KF/P_gain",P_gain);
  nh.getParam("/motion_KF/I_gain",I_gain);
  nh.getParam("/motion_KF/D_gain",D_gain);
  
  nh.getParam("/tau_KF/error_zero_tolerance",error_zero_tolerance_tau);
  nh.getParam("/tau_KF/P_gain",P_gain_tau);
  nh.getParam("/tau_KF/I_gain",I_gain_tau);
  nh.getParam("/tau_KF/D_gain",D_gain_tau);
  
  cout << "/motion_KF/error_zero_tolerance : " << error_zero_tolerance << endl;
  cout << "/motion_KF/P_gain : " << P_gain << endl;
  cout << "/motion_KF/I_gain : " << I_gain << endl;
  cout << "/motion_KF/D_gain : " << D_gain << endl;
  
  cout << "/tau_KF/error_zero_tolerance : " << error_zero_tolerance_tau << endl;
  cout << "/tau_KF/P_gain : " << P_gain_tau << endl;
  cout << "/tau_KF/I_gain : " << I_gain_tau << endl;
  cout << "/tau_KF/D_gain : " << D_gain_tau << endl;
  
  double sd3_q = 3*sd_q;
  double sd3_Dq = 3*sd_Dq;
  double sd3_DDq = 3*sd_DDq;
  double sd3_tau = 3*sd_tau;
  
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_rec(nh,"/joint_states",1);
  
  ROS_INFO("waiting for joint state topic");
  if (!js_rec.waitForANewData(ros::Duration(1000)))
    return 0;
  
  int dof = js_rec.getData().name.size();
  
  Eigen::VectorXd q(dof),
                  Dq(dof),
                  tau(dof),
                  Dq_last(dof),
                  DDq(dof);
  Dq_last.setZero();
  DDq.setZero();
  
  
  // motion_KF Initialization 
  
  Eigen::Matrix3d A,P_k_minus1,R,P_kp, K,P_k,I;
  Eigen::Vector3d B((pow(t,3))/6,(pow(t,2))/2, t);
  Eigen::MatrixXd X_k_minus1(3,dof),Y_k(3,dof),X_k(3,dof),X_kp(3,dof);
  Eigen::VectorXd J(dof), EF(dof), diff_EF(dof), inter_EF(dof), EF_last(dof),P(dof),D(dof);
  
  J.setZero();
  EF.setZero();
  EF_last.setZero();
  diff_EF.setZero();
  inter_EF.setZero();
  
  A << 1, t, (pow(t,2))/2,
       0, 1, t,
       0, 0, 1;

  P_k_minus1 << sd3_q,    0,      0,
                0,        sd3_Dq, 0,
                0,        0,      sd3_DDq;
                
  R = P_k_minus1;
  
  I = I.Identity();
  
  
  // tau_KF Initialization 
  
  double P_k_minus1_tau = sd3_tau;  
  double R_tau = P_k_minus1_tau;
  
  Eigen::MatrixXd X_k_minus1_tau(1,dof),Y_k_tau(1,dof), X_k_tau(1,dof),X_kp_tau(1,dof);
  double P_kp_tau,K_tau,P_k_tau;
  
  Eigen::VectorXd delta(dof), EF_tau(dof), diff_EF_tau(dof), inter_EF_tau(dof), EF_last_tau(dof); 
  // delta = d(tau)/dt - control variable
  delta.setZero();
  EF_tau.setZero();
  diff_EF_tau.setZero();
  inter_EF_tau.setZero();
  EF_last_tau.setZero();
  
  
  int count = 1;

  sensor_msgs::JointState filtered_js;
  ros::Publisher filtered_js_pub = nh.advertise<sensor_msgs::JointState>("/kf_joint_states",1);
  sensor_msgs::JointState js_with_DDq;
  ros::Publisher js_with_DDq_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_with_acceleration",1);
  
  while (ros::ok())
  { 
    for (unsigned int idx=0;idx<dof;idx++)
        {
            q(idx)=js_rec.getData().position.at(idx);
            Dq(idx)=js_rec.getData().velocity.at(idx);
            tau(idx)=js_rec.getData().effort.at(idx);
        }
    
    filtered_js=js_rec.getData();
    js_with_DDq = js_rec.getData();
    filtered_js.effort.resize(12);
    js_with_DDq.effort.resize(12);
    
    DDq=(Dq-Dq_last)*rate_hz;
    Dq_last=Dq;

    // motion_KF 
    
    if (count == 1)
    {
        for (int i = 0;i<dof;i++)
        {
            X_k_minus1(0,i)=q(i);
            X_k_minus1(1,i) = Dq(i);
            X_k_minus1(2,i) = DDq(i);
        }
    }

    X_kp = A*X_k_minus1 + B*J.transpose();
    P_kp = A*P_k_minus1*A.transpose();
    
    P_kp(1,2)=0;    P_kp(1,3)=0;
    P_kp(2,1)=0;    P_kp(2,3)=0;
    P_kp(3,1)=0;    P_kp(3,2)=0;
    
    K = P_kp*((P_kp+R).inverse());
    
    for (int i = 0;i<dof;i++)
    {
        Y_k(0,i) =q(i);
        Y_k(1,i) = Dq(i);
        Y_k(2,i) = DDq(i);
    }

    X_k = X_kp + K*(Y_k-X_kp);
    P_k = (I-K)*P_kp; 
    
    X_k_minus1 = X_k;
    P_k_minus1 = P_k;

    // motion_KF - jerk estimation with PID controller
    
    for (int i = 0;i<dof;i++)
    {
        EF(i) = Y_k(1,i) - X_k(1,i);
        if (abs(EF(i)) < error_zero_tolerance)
            EF(i) = 0;
    }

    diff_EF = (EF - EF_last)/t;
    inter_EF = ((EF - EF_last)/2)*t;
    EF_last = EF;

    // gain sheduling

    I_gain = 0;
    
    for (unsigned int i=0;i<dof;i++)
    {
        if (abs(Dq(i))<0.01 )
        {
            P(i)=5;D(i)=5;    
        }
        else if(abs(Dq(i))<0.1)
        {
            P(i)=50;D(i)=20;
        }
        else if(abs(Dq(i))<0.3)
        {
            P(i)=100;D(i)=30;
        }
        else if(abs(Dq(i))<0.5)
        {
            P(i)=200;D(i)=45;
        }
        else if(abs(Dq(i))<0.8)
        {
            P(i)=400;D(i)=90;
        }
        else
        {
            P(i)=600;D(i)=100;
        }
        
        J(i) = (P(i)*EF(i)) + (D(i)*diff_EF(i));
        
    }

//    J = (P_gain*EF) + (I_gain*inter_EF) + (D_gain*diff_EF);
    
    // tau_KF
    
    if (count == 1)
    {
        for (int i = 0;i<dof;i++)
            X_k_minus1_tau(0,i)=tau(i);
    }
    
    X_kp_tau = X_k_minus1_tau + t*delta.transpose();
    P_kp_tau = P_k_minus1_tau;
    K_tau = P_kp_tau/(P_kp_tau+R_tau);
  
    for (int i = 0;i<dof;i++)
        Y_k_tau(0,i) =tau(i);
  
    X_k_tau = X_kp_tau + K_tau*(Y_k_tau-X_kp_tau);
    P_k_tau = (1-K_tau)*P_kp_tau; 
  
    X_k_minus1_tau = X_k_tau;
    P_k_minus1_tau = P_k_tau;

    // tau_KF - delta estimation with PID controller
    
    for (int i = 0;i<dof;i++)
    {
        EF_tau(i) = Y_k_tau(0,i) - X_k_tau(0,i);
        if (abs(EF_tau(i)) < error_zero_tolerance_tau)
            EF_tau(i) = 0;
    }
  
    diff_EF_tau = (EF_tau - EF_last_tau)/t;
    inter_EF_tau = ((EF_tau - EF_last_tau)/2)*t;
    EF_last_tau = EF_tau;

    delta = (P_gain_tau*EF_tau) + (I_gain_tau*inter_EF_tau) + (D_gain_tau*diff_EF_tau);

    
    // zero velocity to org
    
    for (unsigned int i=0;i<dof;i++)
        {
            if (abs(Dq(i))<0.02 )
            {
                X_k(1,i) = Dq(i);
                X_k(2,i) = 0;
            }
        }
        
    for (int i = 0;i<dof;i++)
    {
//         filtered_js.position.at(i)=X_k(0,i);
        filtered_js.velocity.at(i)=X_k(1,i);
        filtered_js.effort.at(i+dof)=X_k(2,i);
        filtered_js.effort.at(i)=X_k_tau(0,i);
        js_with_DDq.effort.at(i+dof)=DDq(i); 
    }
    
    filtered_js_pub.publish(filtered_js);
    js_with_DDq_pub.publish(js_with_DDq);
    
    rate.sleep();
    ros::spinOnce();

    count = count + 1;
}
    
  waitForShutdown();
  return 0;
}


// longer intergration/differation

// nh.getParam("/C_time_interval",C_time_interval);
//  int C_count = 1;

//   while (ros::ok())
//   {
//     if (C_count == C_time_interval)
//     {
//         diff_EF = (EF - EF_last)/t;
//         inter_EF = ((EF - EF_last)/2)*t;
//         EF_last = EF;
//         C_count = 0;
//     }
//     C_count = C_count+1;


// jerk estimation

// //   double t1 = 0;
// //   double time_limit = 0.005; // must be greater than 0.001
// //   std::vector<double> temp_time;
// //   std::vector<double> temp_estimate_Dq;
// //   std::vector<double> temp_Dq;
// //   double t_start =ros::Time::now().toSec();
  
  
// //     double t_now =ros::Time::now().toSec();
// //     cout << "t_now : " << t_now << endl;
// //     t1 = t_now - t_start;
// //     
// //     if (t1 < time_limit)
// //     {  
// //       cout << "t1 : " << t1 << endl;
// //       temp_time.push_back(t1);
// //       temp_estimate_Dq.push_back(X_k(1));
// //       temp_Dq.push_back(Y_k(1));      
// //     }
// //     else 
// //     {
// //        t_start =ros::Time::now().toSec();
// //        double t_size = temp_time.size();
// //        double eDq_size = temp_estimate_Dq.size();
// //        double Dq_size = temp_Dq.size();
// //        
// //        cout << "t_size : " << t_size << endl;
// //        cout << "eDq_size : " << eDq_size << endl;
// //        cout << "Dq_size : " << Dq_size << endl;
// //        
// //        double area_under_estimate_Dq = ((temp_estimate_Dq[eDq_size]-temp_estimate_Dq[0])/2)*(temp_time[t_size]-temp_time[0]);
// //        double area_under_Dq = ((temp_Dq[Dq_size]-temp_Dq[0])/2)*(temp_time[t_size]-temp_time[0]);
// //        
// //        EF = area_under_Dq - area_under_estimate_Dq;
// //        
// //        if (abs(EF) < error_zero_tolerance)
// //            EF = 0;
// //        
// //        cout << "EF : " << EF << endl;
// //        
// //        temp_time.clear();
// //        temp_estimate_Dq.clear();
// //        temp_Dq.clear();   
// //        
// //        double diff_EF = (EF - last_EF)/t1;
// //        last_EF = EF;
// //        J = P_gain*EF + D_gain*diff_EF;
// //     }
