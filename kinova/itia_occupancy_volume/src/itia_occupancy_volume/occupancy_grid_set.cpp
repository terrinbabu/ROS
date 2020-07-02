#include <string>
#include <iostream>
#include <cassert>
#include <climits>
#include <cmath>
#include <algorithm>
#include <functional>
#include <map>
#include <itia_occupancy_volume/occupancy_grid.h>
#include <itia_occupancy_volume/utils.h>



namespace itia_occupancy_volume 
{

OccupancyGridSet::OccupancyGridSet( const double        ds
                                  , const xyz_t&        bottom_left_vertex
                                  , const xyz_t&        top_right_vertex
                                  , const int           nrays
                                  , const uint64_t      max_point_per_gridm_max_point_per_grid )
: m_max_point_per_grid( max_point_per_gridm_max_point_per_grid )
{
  std::vector< double >  size = { std::fabs( top_right_vertex.at(0) - bottom_left_vertex.at(0) )
                                , std::fabs( top_right_vertex.at(1) - bottom_left_vertex.at(1) )
                                , std::fabs( top_right_vertex.at(2) - bottom_left_vertex.at(2) ) };

  size_t n_split = 0;
  do {
    
    double npoints = (size[0] / ds) * (size[1] / ds)  * ( size[2] / ds) ;
    std::cout << "Number of points: " << npoints << "/" << m_max_point_per_grid << " max (ds=" << ds<<" size: " << size[0] << " " << size[1]<< " " << size[2] << ")" << std::endl;
    if( npoints < m_max_point_per_grid )
    {
      break;
    }
    std::cout << "Split the parallelepiped" << std::endl;
    
    n_split++;
    size[0] /= 2.; size[1] /= 2.; size[2] /= 2.;
    
  } while( 1 );
  std::cout << "Number of split: " << n_split << std::endl;

//   std::vector< std::vector<double> > bottom_left_vertexes = { bottom_left_vertex };
//   std::vector< std::vector<double> > top_right_vertexes = { top_right_vertex };

  for( size_t i_split=0; i_split<n_split; i_split++ )
  {
    throw std::runtime_error("to do");
  }
//   std::cout << "Set of vertexes: " << bottom_left_vertexes.size() << std::endl;
  
//   for( size_t ii=0; ii<bottom_left_vertexes.size(); ii++ )
//   {
//     m_ogs.push_back( new OccupancyGridBase(ds, bottom_left_vertexes[ii], top_right_vertexes[ii], nrays ) );      
//   }
}


bool OccupancyGridSet::inRange( const xyz_t& xyz ) const
{
  bool ret = false;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    ret |= (*it)->inRange( xyz );
  return ret;
}
bool OccupancyGridSet::updateGridFromLine( const xyz_t& pA, const xyz_t& pB, const double radius_bb )
{
  assert( pA.size() == 3 );
  assert( pB.size() == 3 );
  bool ret = false;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    ret |= (*it)->updateGridFromLine( pA, pB, radius_bb );
  return ret;
}
bool OccupancyGridSet::updateGridFromTrapezoid( const xyz_t& pA
                            , const xyz_t& pB
                            , const xyz_t& pC
                            , const xyz_t& pD
                            , const double offset_bb )
{
  bool ret = false;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
  {
    grid_error_t err;
    ret |= (*it)->updateGridFromTrapezoid ( pA, pB, pC, pD, offset_bb, &err );
  }
  return ret;
}
void OccupancyGridSet::terminate( )
{
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    (*it)->terminate( );
}
int OccupancyGridSet::getMaxOccupancy( )
{
  std::vector< int > maxs;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    maxs.push_back( (*it)->getMaxOccupancy( ) );
  return *std::max_element( maxs.begin(), maxs.end() );
}
int OccupancyGridSet::getMinOccupancy( )
{
  std::vector< int > mins;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    mins.push_back( (*it)->getMinOccupancy( ) );
  return *std::min_element( mins.begin(), mins.end() );
}
void OccupancyGridSet::getPointsInRange( const double& minval, const double& maxval, std::vector< xyz_t >* ret, const std::string& what )
{
  assert(ret != NULL);
  ret->clear();
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
  {
    std::vector< xyz_t > tmp;
    if( (*it)->extractSubset( minval, maxval, &tmp, what ) );
      ret->insert(ret->end(), tmp.begin(), tmp.end() );
  }
}
void OccupancyGridSet::update( )
{
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    (*it)->update();
}
double OccupancyGridSet::getGridDimension()
{
  double sz = 0.0;
  for( auto it = m_ogs.begin(); it != m_ogs.end(); it++ )
    sz += (*it)->getGridDimension();
  return sz;
}


}
