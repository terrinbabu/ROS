#include <ur_hardware_interface/ur_hardware_interface.h>
#include <tsupport/basic.h>

namespace itia_hardware_interface
{
  
UrRobotHW::UrRobotHW(std::vector< std::string > joint_names)
{
  m_joint_names=joint_names;
  m_nAx=m_joint_names.size();
  
  if (m_nAx==0)
    ROS_WARN("[%s] no joints selected",m_robot_hw_nh.getNamespace().c_str());
  
}

bool UrRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!itia_hardware_interface::BasicRobotHW::init(root_nh, robot_hw_nh))
  {
    ROS_ERROR("[%s] UrRobotHW error",robot_hw_nh.getNamespace().c_str());
    return false;
  }
  
  if  (m_nAx!=6)
  {
    ROS_ERROR("UR has 6 Joints");
    return false;
  }

  
  //Create UrDriver;
  std::string host;
  int reverse_port;
  double max_payload=1;
  double max_acceleration=15;
  m_max_velocity=10;
  m_acceleration_coeff=2;
  m_curr2torque.resize(6);
  if (!m_robot_hw_nh.getParam("curr2torque",m_curr2torque))
  {
    ROS_WARN("curr2torque is not defined, using 1.0");
    std::fill(m_curr2torque.begin(),m_curr2torque.end(),1.0);
  }
  
  if (!m_robot_hw_nh.getParam("acceleration_coeff",m_acceleration_coeff))
  {
    ROS_WARN("acceleration_coeff is not defined, using 2.0");
    m_acceleration_coeff=2;
  }
  
  
  if (!m_robot_hw_nh.getParam("m_driveip_address",host))
  {
    ROS_ERROR("m_driveip_address is not defined");
    return false;
  }
  if (!m_robot_hw_nh.getParam("reverse_port",reverse_port))
  {
    ROS_WARN("reverse_port is not defined, set 50001");
    reverse_port=50001;
  }
  if (!m_robot_hw_nh.getParam("max_payload",max_payload))
  {
    ROS_WARN("max_payload is not defined, set 1 kg");
    max_payload=1.0;
  }
  if (!m_robot_hw_nh.getParam("max_acceleration",max_acceleration))
  {
    ROS_WARN("max_acceleration is not defined, set 15 rad/s^2");
    max_acceleration=15;
  }
  if (!m_robot_hw_nh.getParam("max_velocity",m_max_velocity))
  {
    ROS_WARN("max_velocity is not defined, set 15 rad/s^2");
    m_max_velocity=15;
  }
  if (!m_robot_hw_nh.getParam("servo_lookahead",m_lookahead))
  {
    ROS_WARN("servo_lookahead is not defined, set 0.1 sec");
    m_lookahead=0.1;
  }
  if (!m_robot_hw_nh.getParam("servo_gain",m_servogain))
  {
    ROS_WARN("servo_gain is not defined, set 300");
    m_servogain=300;
  }
  double speedj_timeout;
  if (!m_robot_hw_nh.getParam("speedj_timeout",speedj_timeout))
  {
    ROS_WARN("servo_gain is not defined, set 0.016");
    speedj_timeout=0.016;
  }
  
  m_max_accepted_deviation=0.1;
  
  m_max_vel_change=max_acceleration/125;
  
  m_last_cmd_vel.resize(m_nAx);
  m_cmd_vel.resize(m_nAx);
  m_cmd_pos.resize(m_nAx);
  m_pos.resize(m_nAx);
  m_vel.resize(m_nAx);
  m_eff.resize(m_nAx);
  
  std::fill(m_cmd_vel.begin(),m_cmd_vel.end(),0.0);
  std::fill(m_last_cmd_vel.begin(),m_last_cmd_vel.end(),0.0);
  
  
  
  m_driver.reset(new itia_hardware_interface::UrDriver(m_rt_msg_cond, m_msg_cond, host, reverse_port,12,0,max_payload,speedj_timeout));
  m_driver->setJointNames(m_joint_names);
  
  if (!m_driver->start())
  {
    ROS_ERROR("Error creating connection");
    return false;
  }
  
  // assign vectors directly to m_pos, m_vel, m_eff changes their addresses and the state_handle does not work.
  std::vector<double> pos, vel, eff, tcp;
  for (int iw=0;iw<100;iw++)
  {
    ros::WallDuration(0.008).sleep(); //wait 10 cycles;
    pos = m_driver->rt_interface_->robot_state_->getQActual();
    vel = m_driver->rt_interface_->robot_state_->getQdActual();
    eff = m_driver->rt_interface_->robot_state_->getIActual();
  }
  
  for (unsigned int idx=0;idx<m_nAx;idx++)
  {
    m_cmd_pos.at(idx)=m_pos.at(idx)=pos.at(idx);
    tsupport::basic::print_green ("Joint value of ");
    std::cout << idx ;
    tsupport::basic::print_green (" is : ");
    std::cout << m_cmd_pos.at(idx) << std::endl;
    m_vel.at(idx)=vel.at(idx);
    m_last_cmd_vel.at(idx)=m_cmd_vel.at(idx)=0;
  }

  for (std::string& joint_name: m_joint_names) 
  {
    auto i = &joint_name-&m_joint_names[0];
    
    hardware_interface::JointStateHandle state_handle(joint_name, 
                                                      &(m_pos.at(i)), 
                                                      &(m_vel.at(i)), 
                                                      &(m_eff.at(i)));
    
    m_js_jh.registerHandle(state_handle);
    
    m_v_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))) );
    m_p_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))) );
    m_pv_jh.registerHandle( hardware_interface::PosVelJointHandle(state_handle, &(m_cmd_pos.at(i)),&(m_cmd_vel.at(i))) );
  }
  
  
  m_velocity_mode=false;
  m_position_mode=false;
  m_posvel_mode=false;
  
  registerInterface(&m_js_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_pv_jh);
  
  m_target_pub=robot_hw_nh.advertise<sensor_msgs::JointState>("command",1);
  m_force_pub=robot_hw_nh.advertise<geometry_msgs::WrenchStamped>("wrench",1);
  m_status=itia_hardware_interface::initialized;
  return true;
}



void UrRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  std::vector<double> pos, vel, eff, tcp;
  
  // assign vectors directly to m_pos, m_vel, m_eff changes their addresses and the state_handle does not work.
  pos = m_driver->rt_interface_->robot_state_->getQActual();
  vel = m_driver->rt_interface_->robot_state_->getQdActual();
  eff = m_driver->rt_interface_->robot_state_->getIActual();
  m_tcp_force = m_driver->rt_interface_->robot_state_->getTcpForce();
  geometry_msgs::WrenchStamped wrench;
  wrench.header.stamp=ros::Time::now();
  wrench.wrench.force.x=m_tcp_force.at(0);
  wrench.wrench.force.y=m_tcp_force.at(1);
  wrench.wrench.force.z=m_tcp_force.at(2);
  wrench.wrench.torque.x=m_tcp_force.at(3);
  wrench.wrench.torque.y=m_tcp_force.at(4);
  wrench.wrench.torque.z=m_tcp_force.at(5);
  m_force_pub.publish(wrench);
  
  for (unsigned int idx=0;idx<m_nAx;idx++)
  {
    m_pos.at(idx)=pos.at(idx);
    m_vel.at(idx)=vel.at(idx);
    m_eff.at(idx)=eff.at(idx)*m_curr2torque.at(idx);
  }
}

void UrRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  sensor_msgs::JointState cmd_msg;
  cmd_msg.effort.resize(6,0);
  cmd_msg.velocity.resize(6,0);
  cmd_msg.position.resize(6,0);

  if (m_status==itia_hardware_interface::with_error)
  {
    m_driver->stop(m_max_vel_change*125);
  }
  else if (m_velocity_mode || m_posvel_mode)
  {
    if (m_posvel_mode)
    {
      for (unsigned int idx=0;idx<m_nAx;idx++)
      {
        if (std::abs(m_pos.at(idx)-m_cmd_pos.at(idx))>m_max_accepted_deviation)
        {
          m_status=itia_hardware_interface::with_error;
          diagnostic_msgs::DiagnosticStatus error;
          error.hardware_id="Universal Robot";
          error.level=diagnostic_msgs::DiagnosticStatus::ERROR;
          error.message="Joint " + m_joint_names.at(idx) + " tracking error. Actual position: " + std::to_string(m_pos.at(idx)) + "Target value: " + std::to_string(m_cmd_pos.at(idx)) + "maximum allowed deviation: " + std::to_string(m_max_accepted_deviation);
          error.name="Tracking error";
          m_diagnostic.status.push_back(error);
        }
      }
    }
    if (m_status==itia_hardware_interface::with_error)
    {
      m_driver->stop(m_max_vel_change*125);
    }
    else 
    {
      for (unsigned int i = 0; i < m_cmd_vel.size(); i++)
      {
        if ( m_cmd_vel.at(i) > (m_last_cmd_vel.at(i) + m_max_vel_change) )
          m_last_cmd_vel.at(i)+=m_max_vel_change;
        else if ( m_cmd_vel.at(i) < (m_last_cmd_vel.at(i) - m_max_vel_change) )
          m_last_cmd_vel.at(i)-=m_max_vel_change;
        else 
          m_last_cmd_vel.at(i)=m_cmd_vel.at(i);
        
        cmd_msg.velocity.at(i)=m_last_cmd_vel.at(i);
      }
      m_driver->setSpeed(m_last_cmd_vel.at(0), m_last_cmd_vel.at(1), m_last_cmd_vel.at(2), m_last_cmd_vel.at(3), m_last_cmd_vel.at(4), m_last_cmd_vel.at(5),  m_acceleration_coeff*m_max_vel_change*125);
    }      
    cmd_msg.header.stamp=ros::Time::now();
    m_target_pub.publish(cmd_msg);
  }
  else if (m_position_mode)
  {
    
    for (unsigned int idx=0;idx<m_nAx;idx++)
    {
      if (std::abs(m_pos.at(idx)-m_cmd_pos.at(idx))>m_max_accepted_deviation)
      {
        m_status=itia_hardware_interface::with_error;
        diagnostic_msgs::DiagnosticStatus error;
        error.hardware_id="Universal Robot";
        error.level=diagnostic_msgs::DiagnosticStatus::ERROR;
        error.message="Joint " + m_joint_names.at(idx) + " tracking error. Actual position: " + std::to_string(m_pos.at(idx)) + "Target value: " + std::to_string(m_cmd_pos.at(idx)) + "maximum allowed deviation: " + std::to_string(m_max_accepted_deviation);
        error.name="Tracking error";
        m_diagnostic.status.push_back(error);
      }
      cmd_msg.position.at(idx)=m_cmd_pos.at(idx);
      cmd_msg.effort.at(idx)=m_cmd_pos.at(idx)-m_pos.at(idx);
    }
      

    if (m_status!=itia_hardware_interface::with_error)
      m_driver->setPosition(m_cmd_pos.at(0),m_cmd_pos.at(1),m_cmd_pos.at(2),m_cmd_pos.at(3),m_cmd_pos.at(4),m_cmd_pos.at(5),0.008,m_lookahead,m_servogain);
      
      cmd_msg.header.stamp=ros::Time::now();
      m_target_pub.publish(cmd_msg);
  }
}

bool UrRobotHW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  
  for (const hardware_interface::ControllerInfo& ctrl: stop_list)
  {
    bool not_prensent=true;
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller;
    for (std::list<hardware_interface::ControllerInfo>::iterator it=m_active_controllers.begin();it!=m_active_controllers.end();it++)
    {
      if (!it->name.compare(ctrl.name))
      {
        stopped_controller=it;
        not_prensent=false;
        break;
      }
    }
    if (not_prensent)
    {
      ROS_ERROR("controller %s is not active, so I cannot stop it",ctrl.name.c_str());
      return false;
    }
    m_active_controllers.erase(stopped_controller);
  }
  for (const hardware_interface::ControllerInfo& ctrl: start_list)
  {
    bool already_prensent=false;
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller;
    for (std::list<hardware_interface::ControllerInfo>::iterator it=m_active_controllers.begin();it!=m_active_controllers.end();it++)
    {
      if (!it->name.compare(ctrl.name))
      {
        stopped_controller=it;
        already_prensent=true;
        break;
      }
    }
    if (already_prensent)
      ROS_WARN("controller %s is not active, so I cannot stop it",ctrl.name.c_str());
    else 
      m_active_controllers.push_back(ctrl);
  }
  
  m_velocity_mode=false;
  m_position_mode=false;
  m_posvel_mode=false;
  for (const hardware_interface::ControllerInfo& ctrl: m_active_controllers)
  {
    for (const hardware_interface::InterfaceResources& res: ctrl.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
      {
        if (m_position_mode)
        {
          ROS_ERROR("Asking to load a velocity controller while a position controller still active");
          return false;
        }
        m_velocity_mode=true;
      }
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
      {
        if (m_velocity_mode)
        {
          ROS_ERROR("Asking to load a position controller while a velocity controller still active");
          return false;
        }
        m_position_mode=true;
        
        for (unsigned int idx=0;idx<m_nAx;idx++)
        {
          m_cmd_pos.at(idx)=m_pos.at(idx);
          ROS_INFO("ACTUAL POSE Joint%d pos=%f",idx,m_cmd_pos.at(idx));
        }
      }
    }
  }
  return true;
}

void UrRobotHW::doSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
}

bool UrRobotHW::setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res)
{
  for (unsigned int idx=0;idx<req.config_par_names.size();idx++)
  {
    if (!req.config_par_names.at(idx).data.compare("servo_gain"))
    {
      if (req.config_par_double_values.at(idx).data>2000)
      {
        ROS_WARN("servo_gain should be less than 2000");
        res.result_str.data="servo_gain should be less than 2000";
        return true;
      }
      if (req.config_par_double_values.at(idx).data<100)
      {
        ROS_WARN("servo_gain should be greater than 100");
        res.result_str.data="servo_gain should be greater than 100";
        return true;
      }
      m_servogain=req.config_par_double_values.at(idx).data;
    }
    if (!req.config_par_names.at(idx).data.compare("servo_lookahead"))
    {
      if (req.config_par_double_values.at(idx).data>0.2)
      {
        ROS_WARN("servo_lookahead should be less than 0.2");
        res.result_str.data="servo_lookahead should be less than 0.2";
        return true;
      }
      if (req.config_par_double_values.at(idx).data<0.03)
      {
        ROS_WARN("servo_lookahead should be greater than 0.03");
        res.result_str.data="servo_lookahead should be greater than 0.03";
        return true;
      }
      m_lookahead=req.config_par_double_values.at(idx).data;
    }
    
  }
  res.result.data=true;
  return true;
}


void UrRobotHW::shutdown()
{
  ROS_INFO_NAMED("UR driver","stopping driver");
  m_driver->halt();
}


}

