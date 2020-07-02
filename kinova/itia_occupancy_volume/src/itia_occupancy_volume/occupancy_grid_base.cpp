#include <string>
#include <iostream>
#include <cassert>
#include <climits>
#include <cmath>
#include <algorithm>
#include <functional>
#include <map>
#include <itia_occupancy_volume/GridNodes.h>
#include <itia_occupancy_volume/occupancy_grid.h>
#include <itia_occupancy_volume/utils.h>



namespace itia_occupancy_volume 
{
  
OccupancyGridBase::OccupancyGridBase( const double  ds
                                    , const xyz_t&  bottom_left_vertex
                                    , const xyz_t&  top_right_vertex )
: OccupancyGridBase( newGridNodes( ds, bottom_left_vertex, top_right_vertex ) )
{

}
  

OccupancyGridBase::OccupancyGridBase  ( boost::shared_ptr<GridNodes> data )
: m_data( data )
{
  
  ROS_INFO("Create Occupancy Grid Base");
  if( data == NULL )
    throw std::runtime_error("Null Pointer as input!");
  
  m_data = sort( fix( data ) );
  m_ogrid_size = m_data->grid_size_x * m_data->grid_size_y * m_data->grid_size_z;
  m_data_packed = *(m_data.get() ) ;
  
  COUTV(  " [1] ds                  [m]: " << m_data->ds << std::endl, 0 );
  printv( " [1] BOTTOM_LEFT_VERTEX  [m]: ", m_data->blv, 0);
  printv( " [1] TOP_RIGHT_VERTEX    [m]: ", m_data->trv, 0);
  COUTV(  " [1] m_npoints           [-]: " << m_ogrid_size << std::endl, 0 );
  COUTV(  " [1] points              [-]: " << m_data->points.size() << std::endl, 0 );

  if( m_data->points.size() == 0 )
    return;
  
  auto m1 = std::max_element(m_data->points.begin(), m_data->points.end(), [&]( GridNode& n1, GridNode& n2) { return n1.hval < n2.hval; } ) ;
  COUTV(  " [1] max                 [-]: " << m1->hval<< std::endl, 0 );
  COUTV(  " [2] ds                  [m]:" << m_data_packed.ds << std::endl, 0 );
  printv( " [2] BOTTOM_LEFT_VERTEX  [m]: ", m_data_packed.blv, 0);
  printv( " [2] TOP_RIGHT_VERTEX    [m]: ", m_data_packed.trv, 0);
  COUTV(  " [2] m_npoints           [-]: " << m_data_packed.grid_size_x * m_data_packed.grid_size_y * m_data_packed.grid_size_z << std::endl, 0 );
  COUTV(  " [2] points              [-]: " << m_data_packed.points.size() << std::endl, 0 );
  auto m2 = std::max_element(m_data_packed.points.begin(), m_data_packed.points.end()
    , [&]( std::pair<const uint64_t, std::pair<boost::array<double,3>, double > >& n1
         , std::pair<const uint64_t, std::pair<boost::array<double,3>, double > >& n2) 
    { return double( n1.second.second ) < double( n2.second.second) ; } );
  COUTV(  " [2] max                 [-]: " << m2->second.second<< std::endl, 0 );
    
  for( auto it = m_data->points.begin(); it != m_data->points.end(); it++ )
  {
    if(it->idx > m_ogrid_size )
    {
      std::cerr <<" OccupancyGridBase::OccupancyGridBase() error in idx: " << it->idx << " ogrid size: " << m_ogrid_size << std::endl;
      throw std::runtime_error("OccupancyGridBase::OccupancyGridBase() Data inconsistency");
    }
    uint64_t idx_;
    bool ok = xyz2i( it->xyz , &idx_ );
    if( !ok )
    {
      std::cout << " m_blv: " << m_data->blv[0] << " " << m_data->blv[1] << " " << m_data->blv[2] << std::endl;
      std::cout << "     x: " << it->xyz[0]<< " " << it->xyz[1]<< " " << it->xyz[2]<< " val" << it->hval << ", " << it->idx  << std::endl;
      std::cout << " m_trv: " << m_data->trv[0] << " " << m_data->trv[1] << " "  << m_data->trv[2] << std::endl;
      std::cerr <<" OccupancyGridBase::OccupancyGridBase() error!! Position out of range "<< std::endl;
      throw std::runtime_error("OccupancyGridBase::OccupancyGridBase() Data inconsistency");
    }
    if( it->idx != idx_ )
    {
      std::cerr <<" OccupancyGridBase::OccupancyGridBase() error!! ????? stored "<< it->idx << " idx from calc: " << idx_ << std::endl;
      
      std::cout << "-------------------------------------------------------------------" << std::endl;
      xyz_t xyz1;
      xyz_t xyz2;
      i2xyz(it->idx , &xyz1 );
      i2xyz(idx_    , &xyz2 );
      std::cout << "IN                       " <<it->idx << "\t x" << it->xyz[0]<< "\t y" << it->xyz[1]<< "\t z" << it->xyz[2]<< "\t val" << it->hval << std::endl;
      std::cout << "CALC 1 (from idx to xyz )" <<it->idx << "\t x" << xyz1[0]<< "\t y" << xyz1[1]<< "\t z" << xyz1[2]<< std::endl;
      std::cout << "CALC 2 (from xyz to idx )" <<idx_ << "\t x" << xyz2[0]<< "\t y" << xyz2[1]<< "\t z" << xyz2[2]<< std::endl;
      
      
      xyz_t xyz11;
      std::cout << " TEST vertex -------------------------------------------------------------------" << std::endl;
      i2xyz(0, &xyz11 );                                              std::cout << "i2xyz " << 0  << "\t\t\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      i2xyz(m_data->grid_size_x-1, &xyz11 );                                 std::cout << "i2xyz " << m_data->grid_size_x << "\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      i2xyz(m_data->grid_size_x * m_data->grid_size_y, &xyz11 );                    std::cout << "i2xyz " << m_data->grid_size_x * m_data->grid_size_y << "\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      i2xyz(m_data->grid_size_x * m_data->grid_size_y * m_data->grid_size_z - 1, &xyz11 ); std::cout << "i2xyz " << m_data->grid_size_x * m_data->grid_size_y * m_data->grid_size_z - 1<< "\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      
      
      std::cout << " TEST i2xyz/xyz2t-------------------------------------------------------------------" << std::endl;
      uint64_t i22;
      i2xyz(it->idx , &xyz11 );       std::cout << "i2xyz " <<it->idx<< "\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      xyz2i(xyz11, &i22 );            std::cout << "xyz2i " <<i22    << "\t x" << xyz11[0]<< "\t y" << xyz11[1]<< "\t z" << xyz11[2]<< std::endl;
      
      double vvvv=std::max( std::max( std::abs( xyz1[0] - xyz2[0]), std::abs(xyz1[1]-xyz2[1])), std::abs(xyz1[2]-xyz2[2]));
      if( std::max( std::max( std::abs( xyz1[0] - xyz2[0]), std::abs(xyz1[1]-xyz2[1])), std::abs(xyz1[2]-xyz2[2])) > 1.1 * m_data->ds )
      {
        std::cout <<vvvv << "---" << m_data->ds << std::endl;
        throw std::runtime_error("OccupancyGridBase::OccupancyGridBase() Data inconsistency");
      } 
    }
  }
}

bool OccupancyGridBase::inRange( const xyz_t& xyz ) const
{
  bool ret = true; 
  for( size_t i=0;i<3;i++ ) 
  {
    ret &= ( xyz[i] >= m_data->blv[i] ) && (xyz[i] <= m_data->trv[i] );
  }
  return ret;
}

bool OccupancyGridBase::xyz2i   ( const std::vector< xyz_t >& xyz, std::vector<uint64_t>*     idx ) const
{
  assert( idx != NULL );
  for( auto it = xyz.begin(); it != xyz.end(); it++ )
  {
    uint64_t v;
    xyz2i( *it, &v );
    idx->push_back( v );
  }
  return true;
};

bool OccupancyGridBase::xyz2uvt ( const std::vector< xyz_t >& xyz, std::vector< uvt_t >* uvt ) const
{
  assert( uvt != NULL );
  for( auto it = xyz.begin(); it != xyz.end(); it++ )
  {
    uvt_t v;
    xyz2uvt( *it, &v );
    uvt->push_back( v );
  }
  return true;
}
bool OccupancyGridBase::i2uvt( const std::vector< uint64_t >& idx, std::vector< uvt_t >*  uvt ) const
{
  assert( uvt != NULL );
  for( auto it = idx.begin(); it != idx.end(); it++ )
  {
    uvt_t v;
    i2uvt( *it, &v );
    uvt->push_back( v );
  }
  return true;
}

bool OccupancyGridBase::i2uvt ( const uint64_t& idx, uvt_t* uvt ) const 
{
      
  assert( uvt != NULL );
  if( idx > m_ogrid_size )
  {
    std::cerr << "index out of boundaries"  << std::endl;
    return false; 
  }

  static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
  static uint64_t points_per_line  = m_data->grid_size_z;
  
  uvt->at(0) =  idx / points_per_plane;
  uint64_t point_in_the_last_plane =  idx % points_per_plane; 
  uvt->at(1) = point_in_the_last_plane / points_per_line;
  uvt->at(2) = point_in_the_last_plane % points_per_line ;
  
  return true;
}

bool OccupancyGridBase::uvt2i (const uvt_t& uvt, uint64_t* idx) const 
{
  if( ( int( uvt[0] ) > m_data->grid_size_x ) || ( int( uvt[1] ) > m_data->grid_size_y ) || ( int( uvt[2] ) > m_data->grid_size_z ) )
    return false;
    
  static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
  static uint64_t points_per_line  = m_data->grid_size_z;
  *idx =  ( uvt[0] * points_per_plane ) 
        + ( uvt[1] * points_per_line ) 
        + ( uvt[2] );
        
  return true;
}

bool OccupancyGridBase::i2xyz ( const uint64_t& idx, xyz_t* xyz ) const 
{
  if( idx > m_ogrid_size )
    return false;
    
  uvt_t uvt;
  i2uvt( idx, &uvt );
  return uvt2xyz( uvt, xyz );
}
bool OccupancyGridBase::xyz2i (const xyz_t& xyz, uint64_t* idx) const
{
  if( !inRange( xyz ) )
    return false; 
  
  xyz_t xyz_translated(xyz);
  std::transform( xyz_translated.begin()
                , xyz_translated.end()
                , m_data->blv.begin()
                , xyz_translated.begin()
                ,  std::minus<double>( ) );
  
  if( xyz_translated[0] < 0 || xyz_translated[1] <0 || xyz_translated[2] < 0 ) 
  {
    printv(" ERROR INPUT:         ", xyz, 5 );
    printv(" ERROR LEFT BOTTOM:   ", m_data->blv, 5 );
    printv(" ERROR TOP RIGHT  :   ", m_data->trv, 5 );
    printv(" ERROR traslated  :   ", xyz_translated, 5 );
    return false;
  }
  
  std::transform( xyz_translated.begin(), xyz_translated.end(), xyz_translated.begin(), std::bind1st( std::multiplies<double>(), 1.0 / m_data->ds ) );
  uvt_t uvt;
  std::transform( xyz_translated.begin(), xyz_translated.end(), uvt.begin(), [](double v){ assert( v > 0 ); return std::round( v ); } );
  return uvt2i( uvt, idx );
  
}

bool OccupancyGridBase::uvt2xyz ( const uvt_t& uvt, xyz_t* xyz ) const
{
  assert( xyz != NULL );
  
  if( ( int( uvt[0] ) > m_data->grid_size_x ) || ( int( uvt[1] ) > m_data->grid_size_y ) || ( int( uvt[2] ) > m_data->grid_size_z ) )
    return false;
  

  xyz_t _xyz = { double(uvt[0]), double(uvt[1]), double(uvt[2]) } ;
  std::transform( _xyz.begin()                     // iterator begin
                , _xyz.end()                       // iterator end
                , _xyz.begin()                     // iterator begin
                , std::bind1st( std::multiplies<double>(), m_data->ds ) ); 
  std::transform( m_data->blv.begin()    // first iterator begin
                , m_data->blv.end()      // first iterator end
                , _xyz.begin()                    // second iterator begin
                , xyz->begin()                    // output itreator begin
                , std::plus<double>( ) );
  return true;
}

bool OccupancyGridBase::xyz2uvt( const xyz_t& xyz, uvt_t* uvt ) const 
{
  if( !inRange( xyz ) )
    return false; 

  static double dsi = 1.0 / m_data->ds;
  
  xyz_t xyz_translated = xyz;
  std::transform( xyz.begin()                     // first iterator begin
                , xyz.end()                       // first iterator end
                , m_data->blv.begin()    // second iterator begin
                , xyz_translated.begin()          // output iterator
                , std::minus<double>( ) );
  
  if( xyz_translated[0] < 0 || xyz_translated[1] <0 || xyz_translated[2] < 0 ) 
  {
    return false;
  }

  std::transform (xyz_translated.begin (), xyz_translated.end (), xyz_translated.begin (),
                std::bind1st (std::multiplies <double> () , dsi)) ;
  
  uvt->at(0) = std::round( xyz_translated.at(0) );
  uvt->at(1) = std::round( xyz_translated.at(1) );
  uvt->at(2) = std::round( xyz_translated.at(2) );
  
  return true;
}

void OccupancyGridBase::getOccupancyVector( const std::vector< uint64_t >& indexes_pnts, std::vector< GridNode >* ret  ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return;
  }
  
  ret->clear();
  for( size_t i = 0; i < indexes_pnts.size(); i++) 
  {
    GridNode gn;
    gn.idx = indexes_pnts[i];
    i2xyz( gn.idx, &(gn.xyz) );
    auto it = std::find_if( m_data->points.begin(), m_data->points.end(), [&]( GridNode& gk ){ return gn.idx == gk.idx; } ); 
    if( it != m_data->points.end() )
    {
      gn.hval = it->hval;
      gn.rval = it->rval;
    }
    else
    {
      gn.hval = 0;
      gn.rval = 0;
    }
    ret->push_back( gn );
  }
}
  
void OccupancyGridBase::extractSubset     ( const std::vector< xyz_t >&         traj
                                          , const double&                       evaluation_grid_dimension
                                          , const double&                       robot_envelop_dimension
                                          , std::vector< GridNode >*            human_grid ) const
#define PROFILER( S, X )\
{\
ros::Time start = ros::Time::now();\
X;\
ros::Time end = ros::Time::now();\
std::cout << S << " ds: " << (end-start).toSec() << std::endl;\
}
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return;
  }
  assert( human_grid != NULL );

  std::vector< std::pair< uint64_t, bool > > evaluation_grid_indexes;
  
PROFILER( "Check TIME ******* Envelope Grid Extraction ******", 
  envelope( traj, evaluation_grid_dimension, robot_envelop_dimension, &evaluation_grid_indexes );
)

PROFILER( "Check TIME ******* Swap  ******", 
  envelope( traj, evaluation_grid_dimension, robot_envelop_dimension, &evaluation_grid_indexes );
)
  human_grid->clear();
//   human_grid->resize( evaluation_grid_indexes.size() );
  
  for( size_t i = 0; i < evaluation_grid_indexes.size(); i++) 
  {

    GridNode gn;
    gn.idx = evaluation_grid_indexes[i].first;
    
    //-------------------------------------
    auto ip = m_data_packed.points.find( gn.idx );
    if( ( ip != m_data_packed.points.end() )
    &&  ( evaluation_grid_indexes[i].second != 0 ) )
    {
      gn.xyz = ip->second.first;
      gn.hval = ip->second.second;
      gn.rval = evaluation_grid_indexes[i].second;
      human_grid->push_back( gn );
    }
//     else
//     {
//       i2xyz( gn.idx, &( gn.xyz ) );
//       gn.hval = 0.0;
//       gn.rval = 0;
//     }

  }
  
  return;
}

bool OccupancyGridBase::extractSubset ( const double&            minval
                                      , const double&            maxval
                                      , std::vector< xyz_t >*    ret
                                      , const std::string&       what ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return false;
  }
  COUTV( "extractSubset min: "<< minval << " max: " << maxval << std::endl, 999); 
  assert( ret != NULL );
  assert( what == "surface" || what == "volume" );
  ret->clear();

  
  COUTV( "extractSubset EXTRACT" << std::endl, 1 );
  auto _minval = std::find_if( m_data->points.begin(), m_data->points.end(), [&](const GridNode& v){ return v.hval > minval; } );
  auto _maxval = std::find_if( m_data->points.begin(), m_data->points.end(), [&](const GridNode& v){ return v.hval > maxval; } );

  std::vector< xyz_t > points( _maxval - _minval );
  
  uint64_t _iminval = std::distance( m_data->points.begin(), _minval  );
  uint64_t _imaxval = std::distance( m_data->points.begin(), _maxval  );
  for(size_t i = _iminval; i < _imaxval; i++)
  {
    i2xyz( m_data->points[ i ].idx, &points[i-_iminval] );
  }
  COUTV( "getPointsInRange size"<< points.size() << std::endl, 1 );
  
  std::cout << "front: " << points.front()[0] << " " << points.front()[1] << " " << points.front()[2] << std::endl;
  std::cout << "back : " << points.back()[0]  << " " << points.back()[1]  << " " << points.back()[2] << std::endl;
  
  std::vector< xyz_t > volume = points;
  std::sort( volume.begin(), volume.end(), [](const xyz_t& v1, const xyz_t& v2){ 
    return  v1[0] <  v2[0] ? true 
          : v1[0] == v2[0] ? ( v1[1] <  v2[1] ? true 
                            : v1[1] == v2[1] ? ( v1[2] < v2[2] ? true : false )
                            : false )
          : false; } 
  );
  
  if( what == "volume" )
  {
    *ret = volume;
  }
  else 
  {
    COUTV( "getPointsInRange PURGING" << std::endl, 1 );
    std::vector< xyz_t > surface;
    purgeInsideElements( volume, &surface );
    *ret = surface;
    COUTV( "getPointsInRange DONE" << std::endl, 1 ); 
  }
  return true;
}

double OccupancyGridBase::getMaxOccupancy( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return -1;
  }
  std::vector<GridNode>::iterator it;
  it = std::max_element( m_data->points.begin(), m_data->points.end(), [](GridNode g1, GridNode g2) { return g1.hval < g2.hval; } );
  
  return it->hval;
}
double  OccupancyGridBase::getMinOccupancy( ) const 
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return -1;
  }
  std::vector<GridNode>::iterator it;
  it = std::min_element( m_data->points.begin(), m_data->points.end(), [](GridNode g1, GridNode g2) { return g1.hval < g2.hval; } );
  
  return it->hval;
}

uint64_t OccupancyGridBase::getGridDimension(const std::string& what) const
{
  uint64_t ret = 0;
  if( what == "non-null" )
    ret = m_data->points.size();
  else
    ret = m_ogrid_size;
  
  return ret;
}

GridNodesConstPtr OccupancyGridBase::getGrid              ( ) const
{
  return m_data;
}

const std::vector<int>& OccupancyGridBase::getTrackedPoints ( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return m_data->samples;
  }
  return m_data->samples;
}
int OccupancyGridBase::getFramesNumber( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return 0;
  }
  return m_data->samples.size();
}
int OccupancyGridBase::getMeanTrackedPoints ( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return 0;
  }
  return m_data->mean_samples;
}
int OccupancyGridBase::getMaxTrackedPoints  ( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return 0;
  }
  return m_data->max_samples;
}

int OccupancyGridBase::getMinTrackedPoints  ( ) const
{
  if( m_data->points.size() == 0 )
  {
    ROS_ERROR("Data not present. Called fix()?");
    return 0;
  }
  return m_data->min_samples;
}



void OccupancyGridBase::piecewiseLine( const xyz_t& pA, const xyz_t& pB, std::vector< uvt_t >* line_uvt ) const 
{

  assert( line_uvt != NULL );
  assert( pA.size() == 3 );
  assert( pB.size() == 3 );
  assert( dist( pA, pB ) > m_data->ds * sqrt(3)/3 );
  
  line_uvt->clear();
  
  printv( "[<<--] A [m]", pA, 1);
  printv( "[<<--] B [m]", pB, 1);
  
  // 2# Start point of the line from A to B expressed in units
  uvt_t start_uvt;  
  uvt_t end_uvt  ;  
  xyz2uvt( pA, &start_uvt );
  xyz2uvt( pB, &end_uvt );
  printv( "[-->>] A      [u] ", start_uvt, 1);
  printv( "[-->>] B      [u] ", end_uvt, 1);
    
  // 3# calc of the unit step from A to B along the axes u,v,t
  std::vector<int>    vAB     ( pA.size() );
  std::vector< int >  nSteps  ( pA.size() ); // number of units from A to B, positive values!!!!
  std::vector< int >  dirSteps( pA.size() ); // direction of the steps from A to B, positive values!!!!

  std::transform( end_uvt.begin(), end_uvt.end(), start_uvt.begin(), vAB.begin(), [&]( uint64_t e, uint64_t s) { return int(e)-int(s);} );
  std::transform( vAB.begin(), vAB.end(), nSteps.begin(), [&]( int v ) { return abs( v ); }  );
  std::transform( vAB.begin(), vAB.end(), dirSteps.begin(), [&]( int v ) { return v > 0 ? 1 : -1;   }  );        
  printv( "[-->>] (B-A) [u]", vAB, 1);
  printv( "[-->>] (B-A) [u]  ", nSteps   , 1);
  printv( "[-->>] (B-A) [dir]", dirSteps , 1);

  // 4# Maximum number of steps along u,v,t
  int np = *(std::max_element( nSteps.begin(), nSteps.end() ));
  if( np < 1 )
  {
    COUTV("[****] Points too close", 1);
    return;
  }

  // 5# create a line (piecewise, like a "stair")
  for( int ip = 0; ip <= np; ip++ )
  {

    line_uvt->push_back({ uint64_t( int(start_uvt[0]) + ip * dirSteps[0] * nSteps[0] / np )
                        , uint64_t( int(start_uvt[1]) + ip * dirSteps[1] * nSteps[1] / np )
                        , uint64_t( int(start_uvt[2]) + ip * dirSteps[2] * nSteps[2] / np )} );
    
  }
}

void OccupancyGridBase::envelope ( const std::vector< xyz_t >&   traj, const double& envelop_radius, std::vector< uint64_t >*      envelope_indexes ) const
{
  assert( envelope_indexes != NULL );
  envelope_indexes->clear();
  std::vector< uint64_t >  traj_indexes;    
  xyz2i( traj, &traj_indexes );
  std::sort( traj_indexes.begin(), traj_indexes.end() );

  for( auto it = traj_indexes.begin();  it != traj_indexes.end(); it++) 
  {
    uvt_t uvt;
    i2uvt( *it, &uvt );
  
    uint64_t uMax = int(uvt[0] + envelop_radius) <  m_data->grid_size_x      ? (uvt[0] + envelop_radius)  : m_data->grid_size_x;
    uint64_t uMin = uvt[0] > envelop_radius                                  ? (uvt[0] - envelop_radius)  : 0;

    uint64_t vMax = int(uvt[1] + envelop_radius) <  m_data->grid_size_y      ? (uvt[1] + envelop_radius)  : m_data->grid_size_y;
    uint64_t vMin = uvt[1] > envelop_radius                                  ? (uvt[1] - envelop_radius)  : 0;
    
    uint64_t tMax = int(uvt[2] + envelop_radius) <  m_data->grid_size_z      ? (uvt[2] + envelop_radius)  : m_data->grid_size_z;
    uint64_t tMin = uvt[2] > envelop_radius                                  ? (uvt[2] - envelop_radius)  : 0;

    static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
    static uint64_t points_per_line  = m_data->grid_size_z;
    
    for( uint64_t u = uMin; u<= uMax; u++) 
      for( uint64_t v = vMin; v<= vMax; v++)
        for( uint64_t t = tMin; t<= tMax; t++)
            envelope_indexes->push_back( u * points_per_plane + v * points_per_line + t );
  }
  std::vector< uint64_t >::iterator it = std::unique( envelope_indexes->begin(), envelope_indexes->end() );
  envelope_indexes->resize( std::distance( envelope_indexes->begin(),it) ); 
  std::sort( envelope_indexes->begin(), envelope_indexes->end() );
  
  std::cout << "envelope_indexes->size()" << envelope_indexes->size() << std::endl;
}
  
  
void OccupancyGridBase::envelope( const std::vector< xyz_t >&                   traj
                                , const double&                                 external_envelop_radius
                                , const double&                                 internal_envelop_radius
                                , std::vector< std::pair<uint64_t,bool> >*      envelope_indexes ) const
  {
    assert( envelope_indexes != NULL );
    envelope_indexes->clear();

    uint64_t external_d = uint64_t( std::round( (external_envelop_radius > internal_envelop_radius ? external_envelop_radius : internal_envelop_radius) / m_data->ds ) );
    uint64_t internal_d = uint64_t( std::round( (external_envelop_radius < internal_envelop_radius ? external_envelop_radius : internal_envelop_radius) / m_data->ds ) );
    
    std::vector< uint64_t >  traj_indexes;    
    xyz2i( traj, &traj_indexes );
    std::sort( traj_indexes.begin(), traj_indexes.end() );
    
  
    for( auto it = traj_indexes.begin();  it != traj_indexes.end(); it++) 
    {
      uvt_t uvt;
      i2uvt( *it, &uvt );
    
      uint64_t uEMax = int(uvt[0] + external_d) <  m_data->grid_size_x      ? (uvt[0] + external_d)  : m_data->grid_size_x;
      uint64_t uEMin = uvt[0] > external_d                                  ? (uvt[0] - external_d)  : 0;
  
      uint64_t vEMax = int(uvt[1] + external_d) <  m_data->grid_size_y      ? (uvt[1] + external_d)  : m_data->grid_size_y;
      uint64_t vEMin = uvt[1] > external_d                                  ? (uvt[1] - external_d)  : 0;
      
      uint64_t tEMax = int(uvt[2] + external_d) <  m_data->grid_size_z      ? (uvt[2] + external_d)  : m_data->grid_size_z;
      uint64_t tEMin = uvt[2] > external_d                                  ? (uvt[2] - external_d)  : 0;
      
      uint64_t uIMax = int(uvt[0] + internal_d) <  m_data->grid_size_x      ? (uvt[0] + internal_d)  : m_data->grid_size_x;
      uint64_t uIMin = uvt[0] > internal_d                                  ? (uvt[0] - internal_d)  : 0;

      uint64_t vIMax = int(uvt[1] + internal_d) <  m_data->grid_size_y      ? (uvt[1] + internal_d)  : m_data->grid_size_y;
      uint64_t vIMin = uvt[1] > internal_d                                  ? (uvt[1] - internal_d)  : 0;

      uint64_t tIMax = int(uvt[2] + internal_d) <  m_data->grid_size_z      ? (uvt[2] + internal_d)  : m_data->grid_size_z;
      uint64_t tIMin = uvt[2] > internal_d                                  ? (uvt[2] - internal_d)  : 0;
    

      static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
      static uint64_t points_per_line  = m_data->grid_size_z;
      
      
      
      for( uint64_t u = uEMin; u<= uEMax; u++) 
        for( uint64_t v = vEMin; v<= vEMax; v++)
          for( uint64_t t = tEMin; t<= tEMax; t++)
          {
            envelope_indexes->push_back( 
              std::make_pair( u * points_per_plane + v * points_per_line + t 
                            , (u> uIMin && u < uIMax) &&(v> vIMin && v < vIMax) &&(t> tIMin && t < tIMax) )
                                        );
          }
    }
    auto it = std::unique( envelope_indexes->begin(), envelope_indexes->end(), [&]( std::pair<uint64_t,bool> i1, std::pair<uint64_t,bool> i2 ) {return i1.first == i2.first; }  );
    envelope_indexes->resize( std::distance( envelope_indexes->begin(),it) ); 
    std::sort( envelope_indexes->begin(), envelope_indexes->end(), []( std::pair<uint64_t,bool> i1, std::pair<uint64_t,bool> i2 ) {return i1.first < i2.first; } );
    
    std::cout << "memory allocated [MB]: " << double(envelope_indexes->size()) * double(sizeof(uint64_t)+ sizeof(bool)) / 1024.0 / 1024.0 << std::endl;
    
  }



 

}
