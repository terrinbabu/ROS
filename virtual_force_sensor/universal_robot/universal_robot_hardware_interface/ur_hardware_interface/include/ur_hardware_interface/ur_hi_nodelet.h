#ifndef __TOPIC_HARDWARE_INTERFACE__ON_NODELET__
#define __TOPIC_HARDWARE_INTERFACE__ON_NODELET__

# include <controller_manager/controller_manager.h>
# include <nodelet/nodelet.h>
# include <thread>
# include <ur_hardware_interface/ur_hardware_interface.h>
# include <itia_basic_hardware_interface/basic_hi_nodelet.h>
namespace itia
{
  namespace control
  {
    
    class UrHwIfaceNodelet : public BasicHwIfaceNodelet
    {
    public:
      virtual void onInit();
      
    protected:
    };
    
    
    
  }
}
# endif