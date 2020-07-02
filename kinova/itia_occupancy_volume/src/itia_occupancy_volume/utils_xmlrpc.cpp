
#include <itia_occupancy_volume/utils_xmlrpc.h>
#include <itia_rutils/itia_rutils_xmlrpc.h>

namespace itia_occupancy_volume
{
  
bool getParam( XmlRpc::XmlRpcValue& config, GridNode& gn )
{
    
  if ( config.getType() != XmlRpc::XmlRpcValue::TypeStruct  )
  {
    ROS_ERROR ( "The input value (XmlRpcValue) is not of type GridNode. %d/%d", int( config.getType()), int( XmlRpc::XmlRpcValue::TypeStruct ) );
    return false;
  }
  
  XmlRpc::XmlRpcValue& point = config["point"];
  if ( point.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR ( "The 'point' field is not of type Array. %d/%d", int( point.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
    return false;
  }
  if ( point.size() != 3 )
  {
    ROS_ERROR ( "The 'point' field is not a 3 element array. %d", int( point.size()) );
    return false;
  }
  
  for(size_t i=0; i<size_t( point.size() ); i++)
  {
    double _val;
    XmlRpc::XmlRpcValue& val = point[i];
    if( val.getType() == XmlRpc::XmlRpcValue::TypeDouble )
      _val = double( val );
    else if( val.getType() == XmlRpc::XmlRpcValue::TypeInt )
    {
      int v = val;
      _val = double( v );
    }
    else
      throw std::runtime_error("Type error!");
    
    gn.xyz[i] = _val ;
  }
  
   
  XmlRpc::XmlRpcValue& prob = config["prob"];
  if( prob.getType() == XmlRpc::XmlRpcValue::TypeDouble )
    gn.hval = double( prob );
  else if( prob.getType() == XmlRpc::XmlRpcValue::TypeInt )
  {
    int v = prob;
    gn.hval = double( v );
  }
  else
    throw std::runtime_error("Type error!");
  
  return true;

}

/*
 * ros::Nodehandle nh
 *  XmlRpc::XmlRpcValue generic_param
 * nh.getParam("/pippopippipiip/occupancy_grid/points")
 * 
 * std::vector<GridNode> gns;
 * getParam( gns, gns );
 * 
 */
bool getParam( XmlRpc::XmlRpcValue& config, std::vector<GridNode>& gns )
{
  
  gns.clear();
  
  if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray  )
  {
    ROS_ERROR ( "The input value (XmlRpcValue) is not of type Array of GridNode. %d/%d", int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
    return false;
  }

  for(size_t i=0; i<size_t( config.size() ); i++)
  {
    ROS_INFO("4+%zu",i);
    XmlRpc::XmlRpcValue point = config[i];
    
    GridNode gn;
    if( getParam( point, gn ) )
      gns.push_back( gn );
    else
      throw std::runtime_error("Error!");
  }
  return true;
}

bool getParam( XmlRpc::XmlRpcValue& config, GridNodes& gi )
{

  if (!itia::rutils::getParam< double >(config, "ds" , gi.ds) ) 
    return false;
    
  if (!itia::rutils::getParamArray<double,3>(config, "bottom_left_vertex" , gi.blv) ) 
    return false;
    
  if (!itia::rutils::getParamArray<double,3>(config, "top_right_vertex" , gi.trv) ) 
    return false;
  
  if (!itia::rutils::getParam(config, "nrays" , gi.nrays) ) 
    return false;
    
  std::vector<std::vector< double > > _grid_points;
  if( !itia::rutils::getParamMatrix( config, "points", _grid_points ) )
  {
    ROS_ERROR("Parameter/occupancy_grid/points does not exist");
    return false;
  }

  for( auto it = _grid_points.begin(); it != _grid_points.end(); it++ )
  {
    GridNode gn;
    gn.idx  = it->at(0);
    gn.hval = it->at(4);
    if( it->size() == 6 )
      gn.rval = it->at(5);
    else
      gn.rval = 0;
    gn.xyz = { it->at(1), it->at(2), it->at(3) };
    gi.points.push_back( gn );
  }
  
  if (!itia::rutils::getParamVector(config, "samples" , gi.samples) ) 
      return false;

  return true;
  
}



void setParam( XmlRpc::XmlRpcValue& config, const GridNodes& gi )
{
  
  config[ "ds"                  ] = gi.ds;
  
  XmlRpc::XmlRpcValue blv;
  blv.setSize( gi.blv.size( ) );
  for( size_t i=0; i < gi.blv.size(); i++) {
    blv[i] = gi.blv[i];
  }
  config[ "bottom_left_vertex"  ] = blv;
  
  XmlRpc::XmlRpcValue trv;
  trv.setSize( gi.trv.size( ) );
  for( size_t i=0; i < gi.trv.size(); i++) {
    trv[i] = gi.trv[i];
  }
  config[ "top_right_vertex"    ] = trv;
  config[ "nrays"               ] = int(gi.nrays);
  
  XmlRpc::XmlRpcValue samples;
  samples.setSize( gi.samples.size( ) );
  for( size_t i=0; i < gi.samples.size(); i++) {
    samples[i] = int( gi.samples[i] );
  }
  config[ "samples" ] = samples;
  
  
  XmlRpc::XmlRpcValue points;
  points.setSize( gi.points.size() );
  for( size_t i=0; i < gi.points.size(); i++) {
    XmlRpc::XmlRpcValue point;
    point.setSize( 3 + 2 );
    
    point[ 0 ] = double( gi.points[i].idx );
    for( size_t j=0; j < 3; j++) {
      point[ j + 1] = gi.points[i].xyz[j];
    }
    point[ 4 ] = gi.points[i].hval;
    points[ i ] = point;
  }
  config["points"] = points;
}

}




