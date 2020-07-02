#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <itia_futils/string.h>
#include <geometry_msgs/Point.h>
#include <itia_occupancy_volume/LoadOccupancyGrid.h>
#include <itia_occupancy_volume/GetGridNodes.h>
#include <itia_occupancy_volume/occupancy_grid.h>


bool parse_program_options(const int argc, const char *const argv[], std::string& ids, std::string& filenames )
{
  std::string file_name;
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
    ("help,h" , "Produce this help message.")
    ("id,i" ,   boost::program_options::value< std::string > (&ids)->required(),"Binary file name with storing the Grid.")
    ("file,f" , boost::program_options::value< std::string > (&filenames)->required(),"Binary file name with storing the Grid.");

  boost::program_options::variables_map vm;
  try
  {
    // Display help text when requested
    if (vm.count("help"))
    {
      std::cout << "–help specified" << std::endl;
      std::cout << desc << std::endl;
      return false;
    }
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    
//     if( ids.size() != filenames.size() ) 
//     {
//       ROS_ERROR_STREAM( " The --id options and the --file options has to be equal in number of elements." );
//       return false; 
//     }
//     
//     for( auto filename : filenames )
//     {
      boost::filesystem::path p( filenames );
      if( !boost::filesystem::exists( p ) ) 
      {
        ROS_ERROR_STREAM( " Input File '" << file_name << "' does not exist " );
        return false; 
      }
//     }
  } 
  catch (std::exception &e)
  {
    ROS_ERROR( "%s", e.what() );
    std::cerr << desc << std::endl;
    ROS_ERROR( "Exit." );
    return false;
  }
  catch (...)
  {
    ROS_ERROR( "Error" );
    std::cerr << desc << std::endl;
    return false;
  }
  
  return true;
}


class OccupancyVolumeQuery 
{
  std::map< std::string, itia_occupancy_volume::OccupancyGridBase* > og_;
public:
  OccupancyVolumeQuery( )
  {
  }
  
  bool loadOccupancyGrid( itia_occupancy_volume::LoadOccupancyGrid::Request& req, itia_occupancy_volume::LoadOccupancyGrid::Response& res )
  {
    if( req.id.size() != req.bin_files.size() )
    {
      res.success = false;
      return true;
    }
    
    for( size_t i=0; i< req.id.size(); i++ )
    {
      auto ig = og_.find( req.id[i] );
      if( ig != og_.end() )
      {
        delete og_[ req.id[i] ]; 
      }
      
      boost::shared_ptr<itia_occupancy_volume::GridNodes> grid_nodes_h_partial( new itia_occupancy_volume::GridNodes( ) );
      try 
      {

        ROS_INFO_STREAM( "<<<<< Load Bin " << req.bin_files[i] );
        ros::Time before = ros::Time::now();
        if( !itia_occupancy_volume::loadBin( req.bin_files[i], grid_nodes_h_partial ) )        
        {
          res.success = res.success && false;
          continue;
        }
        
        ros::Time after = ros::Time::now();
        ROS_INFO_STREAM( " >>>>>>> Load Bin " << req.bin_files[i] << " elasped time: "<< (after-before) );
        
        ROS_INFO("Create OG.. %zu %zu %d %zu %zu ", grid_nodes_h_partial->blv.size()
                                                  , grid_nodes_h_partial->trv.size()
                                                  , grid_nodes_h_partial->nrays
                                                  , grid_nodes_h_partial->points.size()
                                                  , grid_nodes_h_partial->samples.size()  );
        
        og_[ req.id[i] ] = new itia_occupancy_volume::OccupancyGridBase ( grid_nodes_h_partial );
    
        ROS_INFO("OG created! (%p)", (void*)og_[req.id[i]] );
      }
      catch( std::exception& e)
      {
        ROS_INFO_STREAM( "std::exception in parsing " << req.bin_files[i] << std::endl << e.what() );
      }
      catch( ... )
      {
        ROS_INFO_STREAM( "Unhandled Exception in parsing " << req.bin_files[i] << std::endl);
      }
    } 
    res.success = true;
    return true;
  };

  
  bool getGridNodes(itia_occupancy_volume::GetGridNodes::Request& req, itia_occupancy_volume::GetGridNodes::Response& resp ) 
  {
    ros::Time start = ros::Time::now();
    if( og_.find(req.id) == og_.end() )
    {
      ROS_ERROR("The id '%s' of the grid is not recognized", req.id.c_str() );
      ROS_ERROR("Allowed ids: '%s' ", itia::futils::to_string_keys( og_ ).c_str()  );
      
      resp.success = false;
      return false;
    }
    std::vector< itia_occupancy_volume::xyz_t > traj;
    for(auto p : req.path )
      traj.push_back( itia_occupancy_volume::xyz_t( { p.x, p.y, p.z} ) );
    
    ROS_INFO("Extracting the grid %s ", req.id.c_str() );
    (og_[ req.id ])->extractSubset(traj, req.clearance, req.envelope, &(resp.nodes) );    
    ROS_INFO("Extracted the grid %s ", req.id.c_str() );
    ros::Time end = ros::Time::now();
    resp.calc_time = (end-start).toSec();
    resp.success = true;
    return true;
  }
  
};

int main(int argc, char **argv)
{
  
  std::string ids;
  std::string file_names;
  if( !parse_program_options(argc, argv, ids, file_names) )
    return -1;
  
  ros::init(argc, argv, "occupancy_volume_query" ) ;
  ros::AsyncSpinner spinner(8); spinner.start(); 
  ros::NodeHandle nh("~");
  
  ROS_INFO("Node Configuration");
  OccupancyVolumeQuery ovq;

  ROS_INFO("Load File");
  itia_occupancy_volume::LoadOccupancyGrid::Request req;
  itia_occupancy_volume::LoadOccupancyGrid::Response res;
  req.bin_files = { file_names };
  req.id        = { ids };
  if( !ovq.loadOccupancyGrid( req, res ) )
  {
    ROS_ERROR("Important failure in loading th bin file. Exit.");
    return -1;
  }
  if( !res.success )
  {
    ROS_WARN("Attentio!, something was wrong in laoding the file.\n Try to continue. ");
  }
  

  
  ROS_INFO("Services Creation");
  ros::ServiceServer scene_setup_service    = nh.advertiseService("og_load",    &OccupancyVolumeQuery::loadOccupancyGrid, &ovq );
  ros::ServiceServer hrc_setup_service      = nh.advertiseService("og_extract", &OccupancyVolumeQuery::getGridNodes,      &ovq );
  
  ROS_INFO("Ready!");
  ros::waitForShutdown();
  
  return 0;
}

