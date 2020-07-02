#include <ur_hardware_interface/ur_hi_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::UrHwIfaceNodelet, nodelet::Nodelet) 

namespace itia
{
  namespace control
  {
    void UrHwIfaceNodelet::onInit()
    {
      
      m_console_name = getPrivateNodeHandle().getNamespace()+" type: UrHwIfaceNodelet";
      
      ROS_INFO("[%s] STARTING", m_console_name.c_str());
      m_stop = false;
      
      std::vector<std::string> joint_names;
      if (!getPrivateNodeHandle().getParam("joint_names", joint_names))
      {
        ROS_FATAL_STREAM(getPrivateNodeHandle().getNamespace()+"/joint_names' does not exist");
        ROS_FATAL("ERROR DURING STARTING HARDWARE INTERFACE '%s'", getPrivateNodeHandle().getNamespace().c_str());
        return;
      }
      
      m_hw.reset(new itia_hardware_interface::UrRobotHW(joint_names));
      m_main_thread = std::thread(&itia::control::UrHwIfaceNodelet::mainThread, this);
      
    };
    
    
  }
}