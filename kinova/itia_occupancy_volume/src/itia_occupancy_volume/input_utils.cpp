
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
#include <boost/program_options.hpp>
#include <tf/transform_listener.h>

#include <itia_occupancy_volume/occupancy_grid.h>
#include <itia_occupancy_volume/input_utils.h>
#include <itia_rutils/itia_rutils.h>

namespace itia_occupancy_volume
{

DataSource::DataSource  ( const int                             size
              , const std::vector<std::string>       labels_used
              , std::vector< std::vector< double> >  R_kp
              , std::vector< double >                T_p_k ) 
  : m_queue       ( size )
  , m_labels_used ( labels_used )
  , m_R_kp        ( R_kp )
  , m_T_p_k       ( T_p_k )
  , m_stop        ( false )
{
  assert ( size > 0 );
  assert ( labels_used.size() > 0 );
  assert ( R_kp.size() == 3 );
  assert ( R_kp[0].size() == 3 );
  assert ( T_p_k.size() == 3 );
}
DataSource::~DataSource() 
{
};

bool DataSource::pop ( std::map<std::string, xyz_t >* frame )
{
  assert ( frame != NULL );
  if ( m_queue.size() == 0 )
    return false;
    
//   if ( m_queue.empty() )
//     return false;
//   
  *frame = m_queue.front();
  m_queue.pop_front();
  return true;
}

void DataSource::start()
{
  m_thread = boost::thread ( &DataSource::run, this );
}
void DataSource::stop()
{
  m_stop = true;
}
void DataSource::join()
{
  m_thread.join();
}



FileSource::FileSource  ( const int                           size
            , const std::vector<std::string>      labels_used
            , std::vector< std::vector< double> > R_kp
            , std::vector< double >               T_p_k
            , const std::string&                  filename )
: DataSource ( size, labels_used, R_kp, T_p_k )
{
  
  std::ifstream                         kfs ( filename );
  std::vector< std::string >            all_labels;
  std::vector< std::vector< double > >  all_data;

  // extract labes
  std::string _labels_line;
  std::getline ( kfs , _labels_line );
  boost::algorithm::split ( all_labels, _labels_line, boost::is_any_of ( "\t " ), boost::token_compress_on );
  assert ( all_labels[0] == "time" );

  std::string _data_line_str;
  while ( std::getline ( kfs, _data_line_str ) )
  {
    std::istringstream iss ( _data_line_str );
    std::vector< double > _data_line_val;
    double val = 0.0;
    while ( iss >> val )
    {
      _data_line_val.push_back ( val );
    }
    all_data.push_back ( _data_line_val );
  }
  kfs.close();

  std::vector< double > time_vector;
  m_queue.clear();
  for ( size_t iTime = 0; iTime < all_data.size(); iTime++ )
  {
    std::map< std::string, xyz_t > used_data;
    for ( size_t iLabel = 0; iLabel < m_labels_used.size() ; iLabel++ )
    {
      std::vector<std::string>::iterator it = std::find ( all_labels.begin(),all_labels.end(), m_labels_used[iLabel] );
      if ( it == all_labels.end() )
        throw std::runtime_error(std::string( "Error! Label '" + m_labels_used[iLabel] + "' not found in the file!").c_str() );
      
      if ( m_labels_used[iLabel] == "time" )
        time_vector.push_back ( all_data[ iTime ][ 0 ] );
      else
      {
        size_t idx = std::distance ( all_labels.begin() + 1, it ); // do not consider time

        xyz_t data_p;
        for ( size_t iRow=0; iRow<3; iRow++ )
        {
          for ( size_t iCol=0; iCol<3; iCol++ )
            data_p[iRow] += m_R_kp[iRow][iCol] * all_data[iTime][ 3*idx + iCol + 1 ]; // 3 elements more time

          data_p[iRow] += m_T_p_k[iRow];
        }
        used_data[ m_labels_used[iLabel] ] = data_p;
      }
    }
    m_queue.push_back ( used_data );
  }
  
};

void FileSource::run()
{
  while ( ros::ok() && !m_stop )
  {
    ros::Duration ( 2.0 ).sleep();
  }
}

void FileSource::boundaries ( xyz_t*  bottom_left_vertex, xyz_t*  top_right_vertex )
{
  std::cout << "boundaries " << std::endl;
  assert ( bottom_left_vertex != NULL );
  assert ( top_right_vertex != NULL );

  std::cout << "boundaries " << &m_queue << std::endl;
  
  std::vector< double > max_x, max_y, max_z;
  std::vector< double > min_x, min_y, min_z;
  std::cout << "m_queue.size() " << m_queue.size()<< std::endl;
  for ( auto it = m_queue.get().begin(); it != m_queue.get().end(); it++ )
  {
    max_x.push_back ( std::max_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[0] < v2.second[0]; } )->second[0] );
    max_y.push_back ( std::max_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[1] < v2.second[1]; } )->second[1] );
    max_z.push_back ( std::max_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[2] < v2.second[2]; } )->second[2] );

    min_x.push_back ( std::min_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[0] < v2.second[0]; } )->second[0] );
    min_y.push_back ( std::min_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[1] < v2.second[1]; } )->second[1] );
    min_z.push_back ( std::min_element ( it->begin(), it->end(), [&] ( frame_named_t v1, frame_named_t v2 ) { return v1.second[2] < v2.second[2]; } )->second[2] );
  }

  *bottom_left_vertex = { *std::min_element ( min_x.begin(), min_x.end() )  - .5
                        , *std::min_element ( min_y.begin(), min_y.end() )  - .5
                        , *std::min_element ( min_z.begin(), min_z.end() )  - .5 };

  *top_right_vertex   = { *std::max_element ( max_x.begin(), max_x.end() )  + .5
                        , *std::max_element ( max_y.begin(), max_y.end() )  + .5
                        , *std::max_element ( max_z.begin(), max_z.end() )  + .5 };
}

OpenNITracker::OpenNITracker( const int                           size
                            , const std::vector<std::string>      labels_used
                            , std::vector< std::vector< double> > R_kp
                            , std::vector< double >               T_p_k
                            , std::string                         world
                            , const std::string&                  outfiledir )
  : DataSource ( size, labels_used, R_kp, T_p_k )
  , m_outfiledir(outfiledir)
  , m_world ( world ) 
  
{ 
  ROS_INFO("Outfile dir #%sà", m_outfiledir.c_str() );
  if( m_outfiledir.size() > 0 )
  {
    ROS_INFO("Log file: %s", (m_outfiledir + "/opennitracker.m").c_str() );
    m_log.open(m_outfiledir + "/opennitracker.m");
  }
};

void OpenNITracker::run()
{
  ros::Rate rate ( 30.0 );
  ros::Time start_time; 
  do {
    start_time = ros::Time::now();
  
    ROS_INFO("***************************************** ");
    std::cout << "start_time " << start_time << std::endl;
    ROS_INFO("***************************************** ");
    rate.sleep();
  } while( start_time.toSec() <= 0.0001 );
  
  m_log << "opennitracker_data = { ";
  while ( ros::ok() && !m_stop )
  {
    std::map< std::string, xyz_t > used_data;
    for ( size_t iLabel = 0; iLabel < m_labels_used.size() ; iLabel++ )
    {
      try
      {
        tf::StampedTransform transform;
        m_listener.lookupTransform ( m_world, m_labels_used[iLabel], ros::Time ( 0 ), transform );
        tf::Vector3 point = transform.getOrigin();
        xyz_t data_p; std::fill( data_p.begin(), data_p.end(), 0 );
        for ( size_t iRow=0; iRow<3; iRow++ )
        {
          data_p[iRow] += m_R_kp[iRow][0] * point.x();
          data_p[iRow] += m_R_kp[iRow][1] * point.y();
          data_p[iRow] += m_R_kp[iRow][2] * point.z();
          data_p[iRow] += m_T_p_k[iRow];
        }
        used_data[ m_labels_used[iLabel] ] = { data_p[0], data_p[1], data_p[2] };
      }
      catch ( tf::TransformException ex )
      {
        ROS_ERROR ( "[ OpenNITracker::run() ] %s",ex.what() );
      }
    }
    if( used_data.size() != 0 )
    {
      m_queue.push_back ( used_data );
      if( m_outfiledir.size() > 0 )
      {
        m_log << "{ ";
        m_log << (ros::Time::now() - start_time).toSec() << ", { ";
        for( auto data : used_data )
        {
            m_log << "'" << data.first << "', "; 
        }
        m_log << "}, [ ";
        for( auto data : used_data )
        {
          m_log << "[ " << data.second[0] << ", " << data.second[1] << ", " << data.second[2] << ", " << "]; "; 
        }
        m_log << "] }," <<std::endl;;
      }
    }
    rate.sleep();
  }
  m_log << "};" << std::endl;
  return;
}



  
  

  
}

