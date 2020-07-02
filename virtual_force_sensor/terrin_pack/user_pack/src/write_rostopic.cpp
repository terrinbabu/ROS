#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <geometry_msgs/WrenchStamped.h>
#include <chrono>
#include <math.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <std_msgs/Float64.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;
using namespace ros;
using namespace std::chrono;

ofstream file_1,file_2,file_3,file_4,file_5,file_6;

// // std::time_t t = std::time(0);   // get time now
// // std::tm* now = std::localtime(&t);
// // ofstream file_1("/home/terrin/sine_wave_" 
// //                 + std::to_string(now->tm_mday)             + "-"
// //                 + std::to_string(now->tm_mon + 1)          + "-"
// //                 + std::to_string(now->tm_year + 1900)      + "_"
// //                 + std::to_string(now->tm_hour)             + ":"
// //                 + std::to_string(now->tm_min)              + ":"
// //                 + std::to_string(now->tm_sec)              + ".txt");

class rosPS
{
private:
    
    NodeHandle&                                        nh;
    Subscriber                                         topic1_sub;
    Subscriber                                         topic2_sub;
    Subscriber                                         topic3_sub;
    Subscriber                                         topic4_sub;
    Subscriber                                         topic5_sub;
    Subscriber                                         topic6_sub;
    
    int count_1 = 1;
    int count_2 = 1;
    int count_3 = 1;
    int count_4 = 1;
    int count_5 = 1;
    int count_6 = 1;

public:
  
    rosPS ( NodeHandle& nh,
            const std::string& topic1,
            const std::string& topic2,
            const std::string& topic3,
            const std::string& topic4,
            const std::string& topic5,
            const std::string& topic6)
    : nh  ( nh )
    {
        topic1_sub = nh.subscribe(topic1, 1000, &rosPS::topic1_Callback,this);
        topic2_sub = nh.subscribe(topic2, 1000, &rosPS::topic2_Callback,this);
        topic3_sub = nh.subscribe(topic3, 1000, &rosPS::topic3_Callback,this);
        topic4_sub = nh.subscribe(topic4, 1000, &rosPS::topic4_Callback,this);
        topic5_sub = nh.subscribe(topic5, 1000, &rosPS::topic5_Callback,this);
        topic6_sub = nh.subscribe(topic6, 1000, &rosPS::topic6_Callback,this);
    }

    ~rosPS() 
    {};
    
    // // // // // // // // // // // // // // // // // // // // // // // // //     
    // // // // //   if  ros topic : std_msgs::String       // // // // // //     
    // // // // // // // // // // // // // // // // // // // // // // // // //  
    
/*    
    void topic1_Callback(const std_msgs::String& topic_info)
    {
        std::string value= topic_info.data.c_str();
        
        std::time_t t = std::time(0);   // get time now
        std::tm* now = std::localtime(&t);
        
        if(file_1.is_open())
        {
            ros::Time time_now =ros::Time::now();
            
            if (count_1 == 1)
                file_1 << "TIME" << "\t \t" << "RECORDED COMMAND" << "\t" << "\n"; 
            count_1 += 1;
            
            file_1 << std::to_string(now->tm_hour) + ":" + std::to_string(now->tm_min)            + ":" + std::to_string(now->tm_sec) << "\t" << value << "\t" << "\n" ;
        }
    } // void
  */  
    // // // // // // // // // // // // // // // // // // // // // // // // //     
    // // // // //   if  ros topic : std_msgs::Float64       // // // // // //     
    // // // // // // // // // // // // // // // // // // // // // // // // //    
    
    
/*    
    void topic1_Callback(const std_msgs::Float64& topic_info)
    {
        double value;
        value = topic_info.data;

        if(file_1.is_open())
        {
            ros::Time time_now =ros::Time::now();
            
            if (count_1 == 1)
                file_1 << "time" << "\t" << "value" << "\t" << "\n"; 
            count_1 += 1;
            
            file_1 << time_now << "\t" << value << "\t" << "\n" ;
        }
    } // void
*/
    
 /*   
    void topic2_Callback(const std_msgs::Float64& topic_info)
    {
        double value;
        value = topic_info.data;

        if(file_2.is_open())
        {
            ros::Time time_now =ros::Time::now();
            
            if (count_2 == 1)
                file_2 << "time" << "\t" << "value" << "\t" << "\n"; 
            count_2 += 1;
            
            file_2 << time_now << "\t" << value << "\t" << "\n" ;
        }
    } // void
    */

    
    
    // // // // // // // // // // // // // // // // // // // // // // // // // // 
    // // // // //  if  ros topic : sensor_msgs::JointState + accelaration // //      
    // // // // // // // // // // // // // // // // // // // // // // // // // 


    void topic1_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5,
               DDq0,DDq1,DDq2,DDq3,DDq4,DDq5;
        
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);  DDq0 = topic_info.effort.at(6);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);  DDq1 = topic_info.effort.at(7);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);  DDq2 = topic_info.effort.at(8);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);  DDq3 = topic_info.effort.at(9);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);  DDq4 = topic_info.effort.at(10);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);  DDq5 = topic_info.effort.at(11);
        
        if(file_1.is_open())
        {
            if (count_1 == 1)
                file_1 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t"
                       << "DDq0" << "\t" << "DDq1" << "\t" << "DDq2" << "\t" << "DDq3" << "\t" << "DDq4" << "\t" << "DDq5" << "\t" << "\n"; 
            
            count_1 += 1;
            
            file_1 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" 
                   << DDq0  << "\t" << DDq1  << "\t" << DDq2 << "\t" << DDq3 << "\t" << DDq4 << "\t" << DDq5 << "\t" << "\n" ;
            
        }                                               
    } // void

    
/*
    void topic2_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5,
               DDq0,DDq1,DDq2,DDq3,DDq4,DDq5;
        
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);  DDq0 = topic_info.effort.at(6);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);  DDq1 = topic_info.effort.at(7);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);  DDq2 = topic_info.effort.at(8);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);  DDq3 = topic_info.effort.at(9);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);  DDq4 = topic_info.effort.at(10);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);  DDq5 = topic_info.effort.at(11);
        
        if(file_2.is_open())
        {
            if (count_2 == 1)
                file_2 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t"
                       << "DDq0" << "\t" << "DDq1" << "\t" << "DDq2" << "\t" << "DDq3" << "\t" << "DDq4" << "\t" << "DDq5" << "\t" << "\n"; 
            
            count_2 += 1;
            
            file_2 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" 
                   << DDq0  << "\t" << DDq1  << "\t" << DDq2 << "\t" << DDq3 << "\t" << DDq4 << "\t" << DDq5 << "\t" << "\n" ;
            
        }                                               
    } // void
*/

    
    // // // // // // // // // // // // // // // // // // // // //  
    // // // // //  if  ros topic : sensor_msgs::JointState  // //      
    // // // // // // // // // // // // // // // // // // // // //  
    
    void topic2_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5;
       
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);
        
        if(file_2.is_open())
        {
            if (count_2 == 1)
                file_2 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t" << "\n"; 
            
            count_2 += 1;
            
            file_2 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" << "\n" ;
        }                                               
    } // void
    
    
    void topic3_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5;
       
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);
        
        if(file_3.is_open())
        {
            if (count_3 == 1)
                file_3 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t" << "\n"; 
            
            count_3 += 1;
            
            file_3 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" << "\n" ;
        }                                               
    } // void
    
    
    void topic4_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5;
       
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);
        
        if(file_4.is_open())
        {
            if (count_4 == 1)
                file_4 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t" << "\n"; 
            
            count_4 += 1;
            
            file_4 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" << "\n" ;
        }                                               
    } // void
    
    
    void topic5_Callback(const sensor_msgs::JointState& topic_info)
    {
        double seq,
               q0,q1,q2,q3,q4,q5,
               Dq0,Dq1,Dq2,Dq3,Dq4,Dq5,
               eff0,eff1,eff2,eff3,eff4,eff5;
       
        ros::Time time = topic_info.header.stamp;
        seq = topic_info.header.seq;
        
        q0 = topic_info.position.at(0);    Dq0 = topic_info.velocity.at(0);    eff0 = topic_info.effort.at(0);
        q1 = topic_info.position.at(1);    Dq1 = topic_info.velocity.at(1);    eff1 = topic_info.effort.at(1);
        q2 = topic_info.position.at(2);    Dq2 = topic_info.velocity.at(2);    eff2 = topic_info.effort.at(2);
        q3 = topic_info.position.at(3);    Dq3 = topic_info.velocity.at(3);    eff3 = topic_info.effort.at(3);
        q4 = topic_info.position.at(4);    Dq4 = topic_info.velocity.at(4);    eff4 = topic_info.effort.at(4);
        q5 = topic_info.position.at(5);    Dq5 = topic_info.velocity.at(5);    eff5 = topic_info.effort.at(5);
        
        if(file_5.is_open())
        {
            if (count_5 == 1)
                file_5 << "seq" << "\t" << "time" << "\t" 
                       << "q0" << "\t" << "q1" << "\t" << "q2" << "\t" << "q3" << "\t" << "q4" << "\t" << "q5" << "\t" 
                       << "Dq0" << "\t" << "Dq1" << "\t" << "Dq2" << "\t" << "Dq3" << "\t" << "Dq4" << "\t" << "Dq5" << "\t" 
                       << "eff0" << "\t" << "eff1" << "\t" << "eff2" << "\t" << "eff3" << "\t" << "eff4" << "\t" << "eff5" << "\t" << "\n"; 
            
            count_5 += 1;
            
            file_5 << seq << "\t" << time << "\t" 
                   << q0    << "\t" << q1    << "\t" << q2   << "\t" << q3   << "\t" << q4   << "\t" << q5   << "\t" 
                   << Dq0   << "\t" << Dq1   << "\t" << Dq2  << "\t" << Dq3  << "\t" << Dq4  << "\t" << Dq5  << "\t"
                   << eff0  << "\t" << eff1  << "\t" << eff2 << "\t" << eff3 << "\t" << eff4 << "\t" << eff5 << "\t" << "\n" ;
        }                                               
    } // void
    

    // // // // // // // // // // // // // // // // // // // // // // // // //     
    // // // // // if  ros topic : geometry_msgs::WrenchStamped // // // // //      
    // // // // // // // // // // // // // // // // // // // // // // // // // 
    

    void topic6_Callback(const geometry_msgs::WrenchStamped& topic_info)
    {
        double seq,fx,fy,fz,tx,ty,tz;
        
        seq = topic_info.header.seq;
        ros::Time time = topic_info.header.stamp;
        fx = topic_info.wrench.force.x;
        fy = topic_info.wrench.force.y;
        fz = topic_info.wrench.force.z;
        tx = topic_info.wrench.torque.x;
        ty = topic_info.wrench.torque.y;
        tz = topic_info.wrench.torque.z;

        if(file_6.is_open())
        {
            if (count_6 == 1)
                file_6 << "seq" << "\t" << "time" << "\t" 
                       << "fx" << "\t" << "fy" << "\t" << "fz" << "\t" << "tx" << "\t" << "ty" << "\t" << "tz" << "\t" << "\n"; 
            
            count_6 += 1;
            
            file_6 << seq << "\t" << time << "\t" 
                   << fx << "\t" << fy << "\t" << fz << "\t" << tx << "\t" << ty << "\t" << tz << "\t" << "\n" ;
        }
    } // void


}; // class

int main ( int argc, char **argv )
{
    init ( argc, argv, "write_rostopic" );
    NodeHandle nh;

    string file_path;
    bool read_file_path =nh.getParam("/write_rostopic/file_path",file_path);
    string file_name;
    bool read_file_name =nh.getParam("/write_rostopic/file_name",file_name);
   
    
    ifstream ifile(file_path + "ijt_" + file_name + ".txt");
    if (ifile) 
    {
      ROS_ERROR("File already exist");
      return 0;
    }
    
    file_1.open(file_path + "ijt_" + file_name + ".txt");
    file_2.open(file_path + "fjt_" + file_name + ".txt");
    file_3.open(file_path + "ejt_" + file_name + ".txt");
    file_4.open(file_path + "mjt_" + file_name + ".txt");
    file_5.open(file_path + "rjt_" + file_name + ".txt");
    file_6.open(file_path + "vf_"  + file_name + ".txt");

    rosPS topics ( nh,
                   "/ur10/inertial_joint_torques",
                   "/ur10/frictional_joint_torques",
                   "/ur10/estimated_joint_torques",
                   "/joint_states",
                   "/ur10/residual_joint_torques",
                   "/ur10/virtual_wrench");
    
    spin();
    waitForShutdown();
    
    file_1.close();
    file_2.close();
    file_3.close();
    file_4.close();
    file_5.close();
    file_6.close();
    
    return 0;
}
