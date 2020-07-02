#ifndef __ITIA__TF__UTILS__
#define __ITIA__TF__UTILS__


#include <tf/transform_listener.h>


namespace itia 
{
namespace tfutils
{

inline std::string canonical(const std::string& ns, const std::string& ref )
{
  std::string ret;
  if( (ns.front() == '/') && (ns.back() != '/') )
  {
    ret = std::string( ns.begin()+1, ns.end());
  }
  else if( (ns.front() == '/') && (ns.back() == '/') )
  {
    ret = std::string( ns.begin()+1, ns.end()-1 );
  }
  else if( (ns.front() != '/') && (ns.back() == '/') )
  {
    ret = std::string( ns.begin(), ns.end()-1);
  }
  else if( (ns.front() != '/') && (ns.back() != '/') )
  {
    ret = ns;
  }
  
  if( ref.front() == '/' )
    ret += ref;
  else
    ret += "/" + ref;
  
  
  return ret;
}
  

inline bool tfTransformToVector ( const ros::NodeHandle& nh
                                , const std::string first_ref
                                , const std::string second_ref
                                , std::vector<double>& pos
                                , std::vector<double>& quat
                                , const std::string&   ns)
{
  pos =  std::vector<double>( { 0., 0., 0. } ); 
  quat = std::vector<double>( { 0., 0., 0., 0.} );
  tf::TransformListener listener;  
  
  ros::Rate rate(10.0);
  
  tf::StampedTransform transform;
  try
  {
    std::string _first_ref = canonical( ns , first_ref) ;
    std::string _second_ref = canonical( ns , second_ref) ;
    listener.lookupTransform(first_ref, second_ref, ros::Time(0), transform);      
    pos[0] = transform.getOrigin().x();
    pos[1] = transform.getOrigin().y();
    pos[2] = transform.getOrigin().z();
    quat[0] = transform.getRotation().w();
    quat[1] = transform.getRotation().x();
    quat[2] = transform.getRotation().y();
    quat[3] = transform.getRotation().z();     
    return true;
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN_ONCE("%s",ex.what());
  }
  return false;
}
  
  
}
}
  
#endif