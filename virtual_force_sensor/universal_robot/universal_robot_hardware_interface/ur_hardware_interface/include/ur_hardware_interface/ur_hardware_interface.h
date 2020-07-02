#ifndef __ITIA_UR_HARDWARE_INTERFACE__
#define __ITIA_UR_HARDWARE_INTERFACE__

#include <itia_basic_hardware_interface/itia_basic_hardware_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <itia_basic_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ur_hardware_interface/ur_driver.h>



namespace itia_hardware_interface
{
  
  class UrRobotHW: public itia_hardware_interface::BasicRobotHW
  {
  public:
    UrRobotHW(std::vector<std::string> joint_names);
    virtual ~UrRobotHW()
    {
      m_mutex.lock();
      m_mutex.unlock();
    };
    virtual void shutdown();
    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);
    
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) ;
    
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& stop_list);
    
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list);
    
    
    
  protected:
    virtual bool setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res);
    
    std::shared_ptr<ros::Subscriber> m_js_sub;
    std::shared_ptr<ros::Publisher>  m_js_pub;
    sensor_msgs::JointStatePtr m_msg;
    
    hardware_interface::JointStateInterface    m_js_jh; //interface for reading joint state
    hardware_interface::VelocityJointInterface m_v_jh; //interface for writing velocity target
    hardware_interface::PositionJointInterface m_p_jh; //interface for writing velocity target
    hardware_interface::PosVelJointInterface   m_pv_jh; //interface for writing position/velocity target
    std::vector<std::string> m_joint_names;
    
    
    std::vector<double> m_pos; // feedback position
    std::vector<double> m_vel; // feedback velocity
    std::vector<double> m_eff; // feedback effort
    std::vector<double> m_tcp_force;
    std::vector<double> m_curr2torque;
    
    std::vector<double> m_last_cmd_vel;
    std::vector<double> m_cmd_vel; //target velocity
    std::vector<double> m_cmd_pos; //target position
    
    
    // UrDriver Variables
    std::shared_ptr<itia_hardware_interface::UrDriver> m_driver;
    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;
    int m_max_velocity;
    
    std::list< hardware_interface::ControllerInfo > m_active_controllers;
    
    unsigned int m_nAx;
    unsigned int m_missing_messages;
    unsigned int m_max_missing_messages;
    bool m_topic_received;
    
    std::mutex m_mutex;
    double m_max_vel_change;
    
    ros::Publisher m_target_pub;
    ros::Publisher m_force_pub;
    bool m_velocity_mode;
    bool m_position_mode;
    bool m_posvel_mode;
    double m_max_accepted_deviation;
    int m_servogain;
    double m_lookahead;
    int m_acceleration_coeff;
    
  };
}

#endif
