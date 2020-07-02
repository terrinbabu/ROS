
#include <ros/ros.h>
#include <cstdlib>
#include <sys/syscall.h>
#include <limits>
#include <fstream>
#include <istream>
#include <algorithm>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/circular_buffer.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <itia_tfutils/itia_tfutils.h>

#include <itia_occupancy_volume/utils.h>
#include <itia_occupancy_volume/utils_xmlrpc.h>
#include <itia_occupancy_volume/utils_yaml.h>
#include <itia_rutils/itia_rutils.h>

namespace itia_occupancy_volume
{
  
  

int generateSTL( const std::vector< xyz_t >& points, const std::string& outfiledir, const double& range_perc, const double& ds )
{
  try {
    
    pcl::PointCloud<pcl::PointXYZ   >::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal     >::Ptr  normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    
    // LOAD SURFACE POINTS FROM THE SERVER PARAMETER  
    std::for_each(points.begin(), points.end(), [&](const xyz_t& point ) { cloud->push_back( pcl::PointXYZ(point[0],point[1],point[2]) ); } );
    ROS_INFO(" Size cloud: %zu", cloud->size() );
    
    // END OF LOADING POINTS FROM THE SERVER PARAMETER
    
    // Normal estimation*
    pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud( cloud );
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>  ne;
    ne.setInputCloud     (cloud);
    ne.setSearchMethod   (tree);
    ne.setRadiusSearch   (2 * ds);
    ne.compute           (*normals );

    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);      
    ROS_INFO(" Merge points and normals information ... DONE");
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (10 * ds);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors ( 12 );
    gp3.setMaximumSurfaceAngle(2*M_PI); // 45 degrees
    gp3.setMinimumAngle(0); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    ROS_INFO(" cloud_with_normals.size() %zu....", cloud_with_normals->size());
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    ROS_INFO(" Reconstruct ....");
    gp3.reconstruct (triangles);
    ROS_INFO(" Reconstruct DONE");
    // Additional vertex information
    ROS_INFO(" Get Parts ....");
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    ROS_INFO(" Get Parts DONE");
    
    boost::filesystem::path logdir("./");
    boost::filesystem::file_status s = boost::filesystem::status(logdir);
    if( !boost::filesystem::is_directory(s) )
    { 
      if (!boost::filesystem::create_directory("./") )
      {
        ROS_ERROR("Error in creating the directory %s", "./");
        return -1;
      }
    }
    
    ROS_INFO(" Creating STL ");
    pcl::io::savePolygonFileSTL ( outfiledir  + "/surf" + std::to_string(range_perc) + ".stl", triangles );
    ROS_INFO(" Creating STL DONE ");
    ROS_INFO(" Creating Binary STL ");
    int res = system( std::string( "rosrun itia_occupancy_volume justConvertSTL.rb " + outfiledir + "/surf" + std::to_string(range_perc) + ".stl").c_str() ) ;
    if(res <0 )
      ROS_ERROR("Creating Binary STL Failed ");
    
    ROS_INFO(" Creating Binary STL DONE");
  } 
  catch(std::exception& e)
  {
    ROS_ERROR(" Error in creating Binary STL ");
    ROS_ERROR(" %s ", e.what() );
  }
  return 0;
}




  
}

