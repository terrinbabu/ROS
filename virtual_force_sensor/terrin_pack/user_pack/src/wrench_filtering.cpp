#include <iostream>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <geometry_msgs/WrenchStamped.h>
#include <chrono>
#include <math.h>
#include <eigen_state_space_systems/eigen_common_filters.h>

using namespace std;
using namespace ros;
using namespace std::chrono;

class rosPS
{
private:
    
    NodeHandle&                                        nh;
    Subscriber                                         virtual_wrench_sub;
    Publisher                                          filtered_virtual_wrench_pub;
    
    double fx,fy,fz,tx,ty,tz,ffx,ffy,ffz,ftx,fty,ftz;
    double count = 0;
    double read_fz,natural_frequency,sampling_period, sampling_frequency, seq1,seq2;
    high_resolution_clock::time_point t1,t2;
    
    std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> lpffx;
    std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> lpffy;
    std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> lpffz;

public:
  
    rosPS ( NodeHandle& nh,
            const std::string& virtual_wrench)
    : nh  ( nh )
    {
        virtual_wrench_sub = nh.subscribe(virtual_wrench, 1000, &rosPS::virtual_wrench_Callback,this);
        filtered_virtual_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ur10/filtered_virtual_wrench", 1000);
    }

    ~rosPS() 
    {};
    
    void virtual_wrench_Callback(const geometry_msgs::WrenchStamped& virtual_wrench_info)
    {
        if (count == 0)
        {
            t1 = high_resolution_clock::now();
            seq1 = virtual_wrench_info.header.seq;
            bool read_feq =nh.getParam("/wrench_filtering/hz",read_fz);
            natural_frequency = 2*M_PI*read_fz; // [rad/s]
            cout << "natural_frequency : " << read_fz << endl;
        }

        if (count == 20)
        {
            t2 = high_resolution_clock::now();
            seq2 = virtual_wrench_info.header.seq;
            
            sampling_period = ( duration_cast<duration<double>>(t2 - t1).count() ) / (seq2 - seq1);
            
            sampling_frequency = 1/sampling_period ;
            cout << "sampling_frequency : " << sampling_frequency << endl;
            
            lpffx.reset(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period));
            lpffy.reset(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period));
            lpffz.reset(new eigen_control_toolbox::FirstOrderLowPass(natural_frequency,sampling_period));
        }
        
        if (count > 20)
        {
            fx = virtual_wrench_info.wrench.force.x;
            fy = virtual_wrench_info.wrench.force.y;
            fz = virtual_wrench_info.wrench.force.z;
            tx = virtual_wrench_info.wrench.torque.x;
            ty = virtual_wrench_info.wrench.torque.y;
            tz = virtual_wrench_info.wrench.torque.z;
            
            if (count == 21)
            {
                lpffx->setStateFromLastIO(fx, fx);
                lpffy->setStateFromLastIO(fy, fy);
                lpffz->setStateFromLastIO(fz, fz);
            }
            
            ffx=lpffx->update(fx);
            ffy=lpffy->update(fy);
            ffz=lpffz->update(fz);
            
            geometry_msgs::WrenchStamped fwrench_msg;
            
            fwrench_msg.header.stamp=virtual_wrench_info.header.stamp;
            fwrench_msg.header.frame_id=virtual_wrench_info.header.frame_id;
            fwrench_msg.wrench.force.x= ffx;
            fwrench_msg.wrench.force.y= ffy;
            fwrench_msg.wrench.force.z= ffz;
            fwrench_msg.wrench.torque.x=tx;
            fwrench_msg.wrench.torque.y=ty;
            fwrench_msg.wrench.torque.z=tz;

            filtered_virtual_wrench_pub.publish(fwrench_msg);
            
        }
        count += 1;
        
    } // void
}; // class

int main ( int argc, char **argv )
{
    init ( argc, argv, "wrench_filtering" );
    NodeHandle nh;
    rosPS topics ( nh,
                   "/ur10/virtual_wrench");
    spin();
    waitForShutdown();
    return 0;
}
