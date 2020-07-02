
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


#include <itia_tfutils/itia_tfutils.h>

#include <itia_occupancy_volume/common.h>
#include <itia_occupancy_volume/utils.h>
#include <itia_occupancy_volume/utils_xmlrpc.h>
#include <itia_occupancy_volume/utils_yaml.h>
#include <itia_rutils/itia_rutils.h>

namespace itia_occupancy_volume
{

void unit( const xyz_t& v, xyz_t* uv ) 
{
  assert( v.size() == 3 );
  assert( uv !=  NULL );

  double n = sqrt( pow( v[0],2) + pow( v[1],2) + pow( v[2],2) );
  std::transform( v.begin(), v.end(), uv->begin(), std::bind1st( std::multiplies<double>(), 1.0 / n ) );
}
void axis( const xyz_t& pA, const xyz_t& pB, xyz_t* uv, int force_direction ) 
{
  assert( uv != NULL );
  assert( pA.size() == 3 );
  assert( pA.size() == pB.size() );
  assert( force_direction >= -1 );
  assert( force_direction < int(pA.size()) );
  static xyz_t diff;
  if( force_direction < 0 )
    std::transform( pA.begin(), pA.end(), pB.begin(), diff.begin(),  std::minus<double>( ) );
  else
  {
    if( pA[force_direction] > pB[force_direction] )
      std::transform( pA.begin(), pA.end(), pB.begin(), diff.begin(),  std::minus<double>( ) );  
    else 
      std::transform( pB.begin(), pB.end(), pA.begin(), diff.begin(),  std::minus<double>( ) );  
  }
  unit(diff,uv);
}
double dist( const xyz_t& pA, const xyz_t& pB ) 
{
  assert( pA.size() == pB.size() );
  double ret=0;
  for( size_t i= 0; i<pA.size(); i++)
    ret += pow( pA[i] - pB[i], 2 );
  ret = sqrt(ret);
  return ret;
}
uint64_t dist( const uvt_t& pA, const uvt_t& pB ) 
{
  assert( pA.size() == pB.size() );
  size_t ret=0;
  for( size_t i= 0; i<pA.size(); i++)
    ret += pA[i] > pB[i] ? pA[i]-pB[i] : pB[i]-pA[i];

  return ret;
}
void mean( const xyz_t& pA, const xyz_t& pB, xyz_t* mean ) 
{
  assert( pA.size() == pB.size() );
  assert( mean != NULL );
  std::transform( pA.begin(), pA.end(), pB.begin(), mean->begin(), std::plus<double>( ) );
  std::transform( mean->begin(), mean->end(), mean->begin(), std::bind1st( std::multiplies<double>(), 0.5 ) );
}
bool equal( const xyz_t& pA, const xyz_t& pB ) 
{
  assert( pA.size() == pB.size() );
  bool ret = true;
  for(size_t i=0; i< pA.size(); i++) { ret &= pA[i] == pB[i] ; } 
  return ret;
}
bool inside( const xyz_t& pA, const xyz_t& pB, const xyz_t& pC  ) 
{
  assert( pA.size() == pB.size() );
  assert( pA.size() == pC.size() );
    
  xyz_t axAB;
  xyz_t axAC;
  axis(pA,pB,&axAB);
  axis(pA,pC,&axAC);
  
  //double cos_ = axAB[0] * axAC[0] + axAB[1] * axAC[1] + axAB[2] * axAC[2];
  if( axAB[0] * axAC[0] + axAB[1] * axAC[1] + axAB[2] * axAC[2] > 0.99 )
  {      
    return dist( pA, pB ) > dist( pA, pC );
  }
  else 
  {
    COUTV(" outside " << std::endl, 1 );
    return false;
  }
}

  
void piecewiseLine( const uvt_t& pA, const uvt_t& pB, std::vector< uvt_t >* line_uvt )
{
  assert( line_uvt != NULL );

  std::vector< int > nSteps  ( pA.size() ); 
  std::vector< int > dirSteps( pA.size() ); 
  std::transform( pA.begin(), pA.end(), pB.begin(), nSteps.begin(), [&]( uint64_t a, uint64_t b ) { return (b > a ? b - a : a - b); }  );
  std::transform( pA.begin(), pA.end(), pB.begin(), dirSteps.begin(), [&]( uint64_t a, uint64_t b ) { return ( b > a ? 1 : -1); }  );
  
  std::for_each(nSteps.begin(), nSteps.end(), [&](int t){ assert( t>= 0) ; } ) ;
  printv( "--> pA    [u]  ", pA , 1);
  printv( "--> (B-A) [u]  ", nSteps  , 1);
  printv( "--> (B-A) [dir]", dirSteps, 1);

  // 4# Maximum number of steps along u,v,t
  int np = *(std::max_element( nSteps.begin(), nSteps.end() ));
  assert( np > 0 );
  COUTV(  "--> np: " << np << std::endl, 1);
  // 5# create a line (piecewise, like a "stair")
  line_uvt->clear();
  for( int ip = 0; ip <= np; ip++ )
  {
    int iU = int(pA[0]) + ip * dirSteps[0] * nSteps[0] / np;
    int iV = int(pA[1]) + ip * dirSteps[1] * nSteps[1] / np;
    int iT = int(pA[2]) + ip * dirSteps[2] * nSteps[2] / np;
    if( iU < 0 || iV <0 || iT < 0 )
    {
      printv( "[ERROR][INPUT]--> pA    [u]  ", pA, 1);
      printv( "[ERROR][INPUT]--> pB    [u]  ", pB, 1);
      printv( "[ERROR][CALC] --> (B-A) [u]  ", nSteps, 1);
      printv( "[ERROR][CALC] --> (B-A) [dir]", dirSteps , 1);
      
      COUTV( "[ERROR]ip: " << ip << std::endl , 1);
      COUTV( "[ERROR]ip * dirSteps[0] * nSteps[0] / np: " << ip * dirSteps[0] * nSteps[0] / np << std::endl , 1);
      COUTV( "[ERROR]ip * dirSteps[1] * nSteps[1] / np: " << ip * dirSteps[1] * nSteps[1] / np << std::endl , 1);
      COUTV( "[ERROR]ip * dirSteps[2] * nSteps[2] / np: " << ip * dirSteps[2] * nSteps[2] / np << std::endl , 1);
      throw std::runtime_error("Something Strange is happen in 'piecewiseLine' ");
    }
    uvt_t _uvt; _uvt[0] = iU; _uvt[1] = iV; _uvt[2] = iT;
    line_uvt->push_back( _uvt );
  }
}


void piecewiseShpere  ( const uvt_t& pC, const uint64_t& radius, std::vector< uvt_t >* sphere_uvt )
{
  assert( sphere_uvt != NULL );
  sphere_uvt->clear();
  sphere_uvt->push_back( pC );
  for( uint64_t iU = -radius; iU <= radius; iU++ )
  {
    for( uint64_t iV = -radius; iV <= radius; iV++ )
    {
      for( uint64_t iT = -radius; iT <= radius; iT++ )
      {
        if( sqrt( pow( iU, 2) + pow(iV,2) + pow(iT,2) ) <= radius )
        {
          sphere_uvt->push_back( { pC[0]+iU, pC[1]+iV, pC[2]+iT });
        }
      }
    } 
  }  
}

void offsetLine( const std::vector< uvt_t >& line, const std::vector<int>& offset, std::vector< uvt_t >* line_offset )
{
  assert(line_offset != NULL );
  assert(offset.size() == 3 );
  line_offset->clear();
  *line_offset = line;
  for( auto it = line_offset->begin(); it != line_offset->end(); it++ )
  {
      std::transform( it->begin(), it->end(), offset.begin(), it->begin()
                    , [&](uint64_t l, int o ){ return  (int(l) + o) > 0 ?  uint64_t( int(l) + o)  : 0; }  );
  }
}



void fillSet( const std::vector< uvt_t >& setA, const std::vector< uvt_t >& setB, std::vector< uvt_t >* grid )
{
  assert( grid != NULL );
  assert( setA.size() ==  setB.size() );
  printvv("setA", setA, 1);
  printvv("setB", setB, 1);
  grid->clear();
  for( auto itA = setA.begin(), itB = setB.begin()
      ; ( (itA != setA.end())  && (itB != setB.end()) )
      ; itA++, itB++ )
  {
    if( dist( *itA, *itB ) == 1  )
    {
      grid->push_back(*itA);
      grid->push_back(*itB);
    } 
    else if( dist( *itA, *itB ) > 1  )
    {
      std::vector< uvt_t > line;
      piecewiseLine(*itA, *itB, &line);
      for( size_t i=0; i< line.size(); i++) 
        grid->push_back(line[i]);
    }
  }
}


void gridFromLines( const std::vector< uvt_t >& lineA, const std::vector< uvt_t >& lineB, std::vector< uvt_t >*  grid )
{
  printvv("lineA:", lineA, 1);
  printvv("lineA:", lineB, 1);
  assert( grid != NULL );
  if( lineA.size() == lineB.size() )
    return fillSet( lineA, lineB, grid );
  
  std::vector< std::vector< uvt_t > > lines;
  lineSetFromLines ( lineA, lineB, &lines );
  grid->clear();
  for( size_t i1=0; i1 < lines.size() / 2; i1++) 
  {
    for( size_t i2=0; i2< lines[i1].size(); i2++) 
      grid->push_back( lines[i1][i2] );
  }
}
void lineSetFromLines ( const std::vector< uvt_t >& lineA, const std::vector< uvt_t >& lineB, std::vector< std::vector< uvt_t > >* lines )
{
  assert( lines != NULL );
  printvv("lineA:",lineA, 1);
  printvv("lineB:",lineB, 1);
  std::vector< uvt_t > line1, line2;
  if( lineA.size() > lineB.size())
  {
    line1 = lineA;
    line2 = lineB;
  }
  else
  {
    line1 = lineB;
    line2 = lineA;
  }
  
  double r = double( line2.size() ) / double( line1.size() );
  lines->clear();
  for( size_t i1=0; i1 < line1.size(); i1++) 
  {
    if( dist( line1[i1], line2[ r * i1 ] ) == 1  )
    {
      lines->push_back( { line1[i1], line2[ r * i1 ] });
    } 
    else if( dist( line1[i1], line2[ r * i1 ] ) > 1  )
    {
      std::vector< uvt_t > line;
      piecewiseLine( line1[i1], line2[ r * i1 ], &line);
      if( line.size() > 0 )
        lines->push_back( line );
    }
  }
  //std::cout << "--------------------------------- lines " <<  lines->size() << std::endl;
}

  
/**
 * 
 */
void splitParallelepiped( const xyz_t& bl, const xyz_t& tr, std::vector< xyz_t >* bls, std::vector< xyz_t >* trs )
{
  assert( bls != NULL );
  assert( trs != NULL );
  
  const xyz_t bl_ = bl;
  const xyz_t tr_ = tr;
  
  bls->clear( );
  trs->clear( );
  
  bls->push_back({ bl_[0]                 ,  bl_[1]                 ,  bl_[2] } ); //1
  bls->push_back({(bl_[0] + tr_[0]) / 2.0 ,  bl_[1]                 ,  bl_[2] } ); //2
  bls->push_back({ bl_[0]                 , (bl_[1] + tr_[1]) / 2.0 ,  bl_[2] } ); //3
  bls->push_back({(bl_[0] + tr_[0]) / 2.0 , (bl_[1] + tr_[1]) / 2.0 ,  bl_[2] } ); //4
  bls->push_back({ bl_[0]                 ,  bl_[1]                 , (bl_[2] + tr_[2])/2.0   } ); //4
  bls->push_back({(bl_[0] + tr_[0]) / 2.0 ,  bl_[1]                 , (bl_[2] + tr_[2])/2.0   } ); //6
  bls->push_back({ bl_[0]                 , (bl_[1] + tr_[1]) / 2.0 , (bl_[2] + tr_[2])/2.0   } ); //7
  bls->push_back({(bl_[0] + tr_[0]) / 2.0 , (bl_[1] + tr_[1]) / 2.0 , (bl_[2] + tr_[2])/2.0   } ); //8
    
  trs->push_back({(bl_[0] + tr_[0]) / 2.0 , (bl_[1] + tr_[1]) / 2.0 , (bl_[2] + tr_[2]) / 2.0  } ); //1
  trs->push_back({ tr_[0]                 , (bl_[1] + tr_[1]) / 2.0 , (bl_[2] + tr_[2]) / 2.0  } ); //2
  trs->push_back({(bl_[0] + tr_[0]) / 2.0 ,  tr_[1]                 , (bl_[2] + tr_[2]) / 2.0  } ); //3
  trs->push_back({ tr_[0]                 ,  tr_[1]                 , (bl_[2] + tr_[2]) / 2.0  } ); //4
  trs->push_back({(bl_[0] + tr_[0]) / 2.0 , (bl_[1] + tr_[1]) / 2.0 ,  tr_[2] } ); //5
  trs->push_back({ tr_[0]                 , (bl_[1] + tr_[1]) / 2.0 ,  tr_[2] } ); //6
  trs->push_back({(bl_[0] + tr_[0]) / 2.0 ,  tr_[1]                 ,  tr_[2] } ); //7
  trs->push_back({ tr_[0]                 ,  tr_[1]                 ,  tr_[2] } ); //8  
}


void sortPoints( const std::vector< xyz_t >& ipoints, std::vector< xyz_t >* opoints, std::vector< std::string > criteria )
{
  assert( ipoints.size() > 0 );
  assert( ipoints[0].size() == 3 );
  assert( opoints != NULL );
  assert( criteria.size() == 3 );
  static std::map<std::string, std::pair<size_t, bool> > criteria_map = 
  { { "x+", { 0, true   } }
  , { "x-", { 0, false  } }
  , { "y+", { 1, true   } }
  , { "y-", { 1, false  } }
  , { "z+", { 2, true   } }
  , { "z-", { 2, false  } } };

  std::vector<size_t> idx(3,0);
  std::vector<bool>   eq (3,false);
  for( size_t i=0; i < 3; i++ )
  {
    if( criteria_map.find( criteria[i] ) != criteria_map.end() )
    {
      idx[i]  = criteria_map[ criteria[i] ].first;
      eq[i]   = criteria_map[ criteria[i] ].second;
    }
    else
        throw std::runtime_error("Error, criteria not allowed");
  }
  assert( std::unique( idx.begin(), idx.end() ) == idx.end() );
  
  *opoints = ipoints;
  std::sort( opoints->begin(), opoints->end(), [&](const xyz_t& v1, const xyz_t& v2){ 
    bool eq1  = eq[0] ? v1[ idx[0] ] < v2[ idx[0] ] : v1[ idx[0] ] > v2[ idx[0] ];
    bool eq2  = eq[1] ? v1[ idx[1] ] < v2[ idx[1] ] : v1[ idx[1] ] > v2[ idx[1] ];
    bool eq3  = eq[2] ? v1[ idx[2] ] < v2[ idx[2] ] : v1[ idx[2] ] > v2[ idx[2] ];
    bool eq11 = v1[ idx[0] ] == v2[ idx[0] ];
    bool eq22 = v1[ idx[1] ] == v2[ idx[1] ];

      return  eq1   ? true 
            : eq11  ? ( eq2 ? true 
                      : eq22 ? ( eq3 ? true : false )
                      : false )
            : false; } 
    );
}
void purgeInsideElements( const std::vector< xyz_t >& volume, std::vector< xyz_t >* surface )
{
  assert( surface != NULL );
  surface->clear();
  
  std::vector< std::vector<std::string> > criteria ={ { "y+", "z+", "x+" }, { "y+", "z+", "x-" }
                                                    , { "z+", "x+", "y+" }, { "z+", "x+", "y-" }
                                                    , { "x+", "y+", "z+" }, { "x+", "y+", "z-" }};

  std::vector< std::vector< int > > idx = { {1,2}, {1,2}, {2,0}, {2,0}, {0,1}, {0,1} };
  
  std::vector< xyz_t > _surface;
  for( size_t i=0; i< criteria.size(); i++ )
  {
    std::vector< xyz_t > points;  
    sortPoints( volume, &points, criteria[i] );

    auto it = std::unique( points.begin(), points.end(), [&](xyz_t v1, xyz_t v2) { return ((v1[idx[i][0]] == v2[idx[i][0]]) 
                                                                                                                        && (v1[idx[i][1]] == v2[idx[i][1]]) ); });
    points.resize( std::distance(points.begin(), it ) );
  
    _surface.insert( _surface.end(), points.begin(), points.end() );
  }

  std::cout << "before purging  " << _surface.size() << std::endl;  
  sortPoints( _surface, surface, { "x+", "y+", "z+" } );
  
  auto it = std::unique( surface->begin(), surface->end(), [&](xyz_t v1, xyz_t v2) { return( (v1[0] == v2[0]) 
                                                                                          && (v1[1] == v2[1]) 
                                                                                          && (v1[2] == v2[2]) ); });
  
  surface->resize( std::distance( surface->begin(), it ) );
  
  std::cout << "final elements: " << surface->size() << std::endl;  
  
}

void boundig_box( const std::vector<GridNode>& g, xyz_t* blv, xyz_t* trv )
{
  auto xM = std::max_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[0] < j.xyz[0]; } );
  auto yM = std::max_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[1] < j.xyz[1]; } );
  auto zM = std::max_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[2] < j.xyz[2]; } );
  
  auto xm = std::min_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[0] < j.xyz[0]; } );
  auto ym = std::min_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[1] < j.xyz[1]; } );
  auto zm = std::min_element( g.begin(), g.end(), []( GridNode i, GridNode j ){ return i.xyz[2] < j.xyz[2]; } );
  
  blv->at(0) = xm->xyz[0]; blv->at(1) = ym->xyz[1]; blv->at(2) = zm->xyz[2];
  trv->at(0) = xM->xyz[0]; trv->at(1) = yM->xyz[1]; trv->at(2) = zM->xyz[2];
}

size_t  non_null_elements( const std::vector<GridNode> & grid )
{
  size_t ret=0;
  for( auto it =grid.begin(); it!= grid.end(); it++)
    ret+= ( it->hval>0) ? 1 : 0;
  return ret;
}

  

  
  


void find( const std::vector< xyz_t >& points, const std::string& criteria, xyz_t* point )
{
  assert( point != NULL );
  assert( point->size() == 3 );
  assert( (criteria == "TR") || (criteria == "TL") || (criteria == "BR") || (criteria == "BL") );

  //sort along z
  size_t nz=0;
  std::vector<size_t> idz( points.size() );
  std::transform(idz.begin(), idz.end(), idz.begin(), [&](size_t k){ return nz++; });
  std::sort(idz.begin(), idz.end(), [&](size_t i1, size_t i2) {return points[i1][2] < points[i2][2] ;} );
  
  if( criteria == "TR" )
  {
    size_t i1 = idz[ idz.size() - 1 ];
    size_t i2 = idz[ idz.size() - 2 ];
    if( points[ i1 ][1] > points[ i2 ][1] )
      *point = points[ i1 ];
    else if( points[ i1 ][1] == points[ i2 ][1] )
      *point = points[ i1 ][0] > points[ i2 ][0] ? points[ i1 ] : points[ i2 ];
    else
      *point = points[ i2 ];
  }
  else if( criteria == "TL" )
  {
    size_t i1 = idz[ idz.size() - 1 ];
    size_t i2 = idz[ idz.size() - 2 ];
    if( points[ i1 ][1] < points[ i2 ][1] )
      *point = points[ i1 ];
    else if( points[ i1 ][1] == points[ i2 ][1] )
      *point = points[ i1 ][0] < points[ i2 ][0] ? points[ i1 ] : points[ i2 ];
    else
      *point = points[ i2 ];
  }
  else if( criteria == "BR" )
  {
    size_t i1 = idz[ 0 ];
    size_t i2 = idz[ 1 ];
    if( points[ i1 ][1] > points[ i2 ][1] )
      *point = points[ i1 ];
    else if( points[ i1 ][1] == points[ i2 ][1] )
      *point = points[ i1 ][0] > points[ i2 ][0] ? points[ i1 ] : points[ i2 ];
    else
      *point = points[ i2 ];
  }
  else if( criteria == "BL" )
  {
    size_t i1 = idz[ 0 ];
    size_t i2 = idz[ 1 ];
    if( points[ i1 ][1] < points[ i2 ][1] )
      *point = points[ i1 ];
    else if( points[ i1 ][1] == points[ i2 ][1] )
      *point = points[ i1 ][0] < points[ i2 ][0] ? points[ i1 ] : points[ i2 ];
    else
      *point = points[ i2 ];
  }
}

bool    getParam( ros::NodeHandle& nh, const std::string& key, GridNodes& gi )
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //LOAD SCENE (valuatre se mettere prima parte in una funzione apposita in quanto in comune )
  //   xyz_t obs_pos =  xyz_t( { 0., 0., 0. } ); 
  //   if( !get_ros_param_pos(nh, "/obstacles/position", obs_pos ))
  //     return -1;
  //   *base_translation_world = obs_pos;
  //   
  //   xyz_t obs_or = xyz_t( { 0., 0., 0., 0.} );
  //   get_ros_param_orient(nh, "/obstacles/orientation", obs_or);
  //   *rotation_world_base = obs_or;    
  
  //the data coming from the kinect are in the base system, while the env is in teh world sys
/*  xyz_t pos; 
  xyz_t quat;
  if (!itia::tfutils::tfTransformToVector(nh, "/world", "/base", pos, quat, ""))
    return false;    
  gi.base_translation_world = pos;
  gi.rotation_world_base = quat;  */  
  
  XmlRpc::XmlRpcValue node;
  if( nh.getParam( key, node ) )
    return itia_occupancy_volume::getParam(node, gi);
  
  return false;
}


bool getParam( const boost::filesystem::path& filename, GridNodes& gi )
{
 
  if (!boost::filesystem::exists(filename))   
  {
    ROS_ERROR("File %s not loaded", filename.string().c_str());
    return false;
  }
  
  YAML::Node node = YAML::LoadFile( filename.string() );
  
  return getParam(node,gi);
}

void setParam( ros::NodeHandle& nh, const std::string& key, const GridNodes& gi )
{
  XmlRpc::XmlRpcValue config;
  setParam( config, gi );
  nh.setParam( key, config );
}



bool saveYaml ( const std::string& path
              , const std::string& filename
              , const GridNodes& gi ) 
{
  boost::filesystem::path p( path );
  if( !boost::filesystem::is_directory( p ) )
  {
    ROS_ERROR("Path %s does not exist", p.string().c_str() );
    return false;
  }
  
  // Save YAML
  std::ofstream fd( boost::filesystem::canonical(path).string()+ "/" + filename );
  
  fd << "occupancy_grid:" << std::endl;
  fd << "  ds: " << gi.ds << std::endl;
  fd << "  bottom_left_vertex: [" << gi.blv[0] <<", "<<gi.blv[1]<<", "<<gi.blv[2]<<"]" << std::endl;
  fd << "  top_right_vertex: [" << gi.trv[0] <<", "<<gi.trv[1]<<", "<<gi.trv[2]<<"]" << std::endl;
  fd << "  nrays: " << gi.nrays << std::endl;
  fd << "  points:" << std::endl; 
  
  int cnt = 0;
  int cnt2 = 0;
  for(auto it = gi.points.begin(); it != gi.points.end(); it++)
  {
    cnt++;
    if ( it->hval != 0.0)
    {
      cnt2++;
      fd << "    - ["<< it->idx<<", " << it->xyz[0]<< ", " << it->xyz[1] << ", " << it->xyz[2] <<", "<< it->hval << "] "<< std::endl;
    }
  }
  std::cout << "filename: " <<filename << " cnt: " << cnt << " cnt2: " << cnt2 << " input points: " << gi.points.size() << std::endl;;
  
  fd << "  samples:" << std::endl;

  for(auto it = gi.samples.begin(); it != gi.samples.end(); it++)
  {
    fd << "    - "<< *it << std::endl;
  }
  fd << "  mean_samples: " << gi.mean_samples << std::endl;
  fd << "  min_samples: "  << gi.min_samples  << std::endl;
  fd << "  max_samples: "  << gi.max_samples  << std::endl;
  fd.close();
  return true;
}

bool saveBin( const std::string& path
            , const std::string& filename
            , const GridNodes& gi
            , std::string&       complete_filename) 
{
  boost::filesystem::path p( path );
  if( !boost::filesystem::is_directory( p ) )
  {
    ROS_ERROR("Path %s does not exist", p.string().c_str() );
    return false;
  }
 
  boost::filesystem::path pp( path / boost::filesystem::path( filename ) );
  
  complete_filename = pp.stem().string()
                     + "_p" + std::to_string( gi.points.size() ) 
                     + "_s" + std::to_string( gi.samples.size() ) 
                     + pp.extension().string();
  
  // Save BIN
  std::ofstream fd( p.string() + "/" + complete_filename, std::ios::out | std::ios::binary);
  if( !fd )
  {
    ROS_ERROR( "Cannot open file." );
    return 1;
  }
  
  char* buffer = new char[ grid_info_size( &gi ) ];
  
  to_buffer( &gi, buffer  );
  
  fd.write( buffer, grid_info_size( &gi ) );
  fd.close();

  return true;
}

bool yaml2bin ( const std::string& path
              , const std::string& yaml_filename
              , const std::string& bin_filename ) 
{
  boost::filesystem::path p( path );
  boost::filesystem::path pp( p / yaml_filename );
  
  GridNodes gi;
  if( !getParam( pp, gi ) )
  {
    ROS_ERROR("Erro in reading YAML file %s", pp.string().c_str() );
    return false;
  }
  
  std::string nu;
  return saveBin ( path, bin_filename, gi, nu );
  
}


bool loadBin  ( const std::string&              fullpath
              , boost::shared_ptr<GridNodes>    gi ) 
{
  boost::filesystem::path pp( fullpath );
  if( !boost::filesystem::exists( pp ) )
  {
    ROS_ERROR("[ Occupancy Volume ] File %s does not exist", pp.string().c_str() );
    return false;
  }
  
  std::string filename_ = pp.stem().string();
  
  std::vector< std::string > tt;
  boost::split(tt,filename_,boost::is_any_of("_"));      
  
  int nSamples = std::atoi( std::string( (tt.end()-1)->begin()+1,  (tt.end()-1)->end() ).c_str() );
  int nPoints  = std::atoi( std::string( (tt.end()-2)->begin()+1,  (tt.end()-2)->end() ).c_str() );
  
  std::ifstream fd( boost::filesystem::canonical(pp).string(), std::ios::in | std::ios::binary);
  if(!fd) 
  {
    ROS_ERROR( "Cannot open file %s.", boost::filesystem::canonical(pp).string().c_str()  );
    return 1;
  }
  
  gi->points.resize( nPoints );
  gi->samples.resize( nSamples );
  char* buffer = new char[ grid_info_size( gi.get() ) ];
  
  fd.read(buffer, grid_info_size( gi.get() ));
  fd.close();
  
  from_buffer( buffer, gi.get()  );
  
  return true;
}



bool loadBin  ( const std::string&              path
              , const std::string&              filename 
              , boost::shared_ptr<GridNodes>    gi  ) 
{
  boost::filesystem::path p( path );
  if( !boost::filesystem::is_directory( p ) )
  {
    ROS_ERROR("Path %s does not exist", p.string().c_str() );
    return false;
  }
  
  return loadBin( ( p / filename ).string(), gi );
}



  
}

