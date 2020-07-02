#ifndef __ITIA_MSG_UTILS__
# define __ITIA_MSG_UTILS__

# include <itia_msgs/MotionStamped.h>

namespace itia
{
namespace msgs
{
  inline void initMotionStamped(const int& size, itia_msgs::MotionStamped* motion)
  {
    motion->lin.resize(size);
    motion->ang.resize(size);
    for (int idx=0;idx<size;idx++)
    {
      motion->lin.at(idx).x=0;
      motion->lin.at(idx).y=0;
      motion->lin.at(idx).z=0;
      motion->ang.at(idx).x=0;
      motion->ang.at(idx).y=0;
      motion->ang.at(idx).z=0;
    }
  };
}
}

#endif
