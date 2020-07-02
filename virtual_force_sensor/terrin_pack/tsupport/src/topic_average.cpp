#include <iostream>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <tsupport/basic.h>
#include <tsupport/moveit_utils.h>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;
using namespace ros;

class rosPS
{
private:
    
    NodeHandle&                                        nh;
    Subscriber                                         virtual_wrench_sub;
    Publisher                                          max_min_avg_wrench_pub;
    Publisher                                          val_avg_wrench_pub;
    int count, time_start, time_now, loop_time;
    vector<double> vfx,vfy,vfz,vtx,vty,vtz;
    vector<double> mfx,mfy,mfz,mtx,mty,mtz;
    
public:
  
    rosPS ( NodeHandle& nh,
            const std::string& virtual_wrench)
    : nh  ( nh )
    {
        count = 0;
        virtual_wrench_sub = nh.subscribe(virtual_wrench, 1000, &rosPS::virtual_wrench_Callback,this);
        max_min_avg_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ur10/max_min_avg_wrench", 1000);
        val_avg_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ur10/val_avg_wrench", 1000);
    }

    ~rosPS() 
    {};
    
    void virtual_wrench_Callback(const geometry_msgs::WrenchStamped& virtual_wrench_info)
    {
//         cout << virtual_wrench_info.wrench.force.x << endl;
        if (count == 0)
            time_start = virtual_wrench_info.header.stamp.sec;
        
        time_now = virtual_wrench_info.header.stamp.sec;
        mfx.push_back(virtual_wrench_info.wrench.force.x);
        mfy.push_back(virtual_wrench_info.wrench.force.y);
        mfz.push_back(virtual_wrench_info.wrench.force.z);
        mtx.push_back(virtual_wrench_info.wrench.torque.x);
        mty.push_back(virtual_wrench_info.wrench.torque.y);
        mtz.push_back(virtual_wrench_info.wrench.torque.z);
        
        count = count + 1;
        loop_time = time_now - time_start;
        
        if (loop_time == 3)
        {  
            double avg_fx,avg_fy,avg_fz,avg_tx,avg_ty,avg_tz;

            tsupport::basic::vector_max_min_avg (avg_fx,mfx);
            tsupport::basic::vector_max_min_avg (avg_fy,mfy);
            tsupport::basic::vector_max_min_avg (avg_fz,mfz);
            tsupport::basic::vector_max_min_avg (avg_tx,mtx);
            tsupport::basic::vector_max_min_avg (avg_ty,mty);
            tsupport::basic::vector_max_min_avg (avg_tz,mtz);
            
            geometry_msgs::WrenchStamped mwrench_msg;
            
            mwrench_msg.header.stamp=virtual_wrench_info.header.stamp;
            mwrench_msg.header.frame_id=virtual_wrench_info.header.frame_id;
            mwrench_msg.wrench.force.x=avg_fx;
            mwrench_msg.wrench.force.y=avg_fy;
            mwrench_msg.wrench.force.z=avg_fz;
            mwrench_msg.wrench.torque.x=avg_tx;
            mwrench_msg.wrench.torque.y=avg_ty;
            mwrench_msg.wrench.torque.z=avg_tz;

            max_min_avg_wrench_pub.publish(mwrench_msg);

            mfx.clear();            mtx.clear();
            mfy.clear();            mty.clear();
            mfz.clear();            mtz.clear();

//             tsupport::basic::vector_val_avg (avg_fx,vfx);
//             tsupport::basic::vector_val_avg (avg_fy,vfy);
//             tsupport::basic::vector_val_avg (avg_fz,vfz);
//             tsupport::basic::vector_val_avg (avg_tx,vtx);
//             tsupport::basic::vector_val_avg (avg_ty,vty);
//             tsupport::basic::vector_val_avg (avg_tz,vtz);
    
            count = 0; time_start = 0;
            
        }
        
    } // void
}; // class

int main ( int argc, char **argv )
{
    init ( argc, argv, "topic_average" );
    NodeHandle nh;
    rosPS topics ( nh,
                   "/ur10/virtual_wrench");
    spin();
    waitForShutdown();
    return 0;
}



















// // // int main(int argc, char **argv)
// // // {
// // //   init(argc, argv,"topic_average");
// // //   NodeHandle nh;
// // //   MultiThreadedSpinner spinner(4);
// // //   spinner.spin();
// // // //   AsyncSpinner spinner(1);
// // // //   spinner.start();
// // //   sleep(1.0);
// // //   
// // //   ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped> wrench_msg(nh,"/ur10/virtual_wrench",1);
// // //   ros::Publisher wrench_avg_pub=nh.advertise<geometry_msgs::WrenchStamped>("/ur10/avg_wrench",1);
// // //     
// // //   ROS_INFO("waiting for topic");
// // //   if (!wrench_msg.waitForANewData(ros::Duration(1000)))
// // //     return 0;
// // //   
// // //   int count = 0;
// // // //  vector<double> fx = {0,0,0,0,0,0,0,0,0,0};
// // //   vector<double> fx,fy,fz,tx,ty,tz;
// // //     
// // //   while (ros::ok())
// // //   {
// // //     double fx = wrench_msg.getData().wrench.force.x;
// // //     cout << fx << endl;
// // // //     fx.push_back(wrench_msg.getData().wrench.force.x);
// // // //     fy.push_back(wrench_msg.getData().wrench.force.y);
// // // //     fz.push_back(wrench_msg.getData().wrench.force.z);
// // // //     tx.push_back(wrench_msg.getData().wrench.torque.x);
// // // //     ty.push_back(wrench_msg.getData().wrench.torque.y);
// // // //     tz.push_back(wrench_msg.getData().wrench.torque.z);
// // //     
// // //     count = count + 1;
// // //     
// // // // //     if (count == 9)
// // // // //     {  
// // // // //        tsupport::basic::printvectordouble ("fx",fx);
// // // // //        
// // // // //        double avg_fx,avg_fy,avg_fz,avg_tx,avg_ty,avg_tz;
// // // // // 
// // // // //        tsupport::basic::vector_max_min_avg (avg_fx,fx);
// // // // //        tsupport::basic::vector_max_min_avg (avg_fy,fy);
// // // // //        tsupport::basic::vector_max_min_avg (avg_fz,fz);
// // // // //        tsupport::basic::vector_max_min_avg (avg_tx,tx);
// // // // //        tsupport::basic::vector_max_min_avg (avg_ty,ty);
// // // // //        tsupport::basic::vector_max_min_avg (avg_tz,tz);
// // // // //        
// // // // //        cout << avg_fx << endl;
// // // // //        
// // // // //        fx.clear();
// // // // //        count = 0;
// // // // //     }
// // // //     spin();
// // //   }
// // // }
