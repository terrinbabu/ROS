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
  
OccupancyGrid::OccupancyGrid( const double                   ds
                            , const xyz_t&                   bottom_left_vertex
                            , const xyz_t&                   top_right_vertex
                            , const int                      nrays )
  : OccupancyGridBase   ( ds, bottom_left_vertex, top_right_vertex ) 
  , m_nrays             ( nrays   )
  , m_idx_ogrid         (  )
  , m_ogrid             ( m_ogrid_size, 0 )
  , m_idx               ( m_ogrid_size, 0 )
  , m_tracked_points    ( 35 * 3600  ) 
  , m_angles            ( m_nrays )
  , m_total_occurrences ( 0 )
  , m_terminate         ( false   )
{
  printv( " BOTTOM_LEFT_VERTEX: ", bottom_left_vertex, 0);
  printv( " TOP_RIGHT_VERTEX  : ", top_right_vertex, 0);
  COUTV(  " npoints:            " << m_ogrid_size << std::endl, 0 );
  assert( m_nrays != 0 );
  for( size_t i=0; i< m_angles.size(); i++ )
    m_angles[i] = i * 2.0 * M_PI / m_angles.size();
  
  // ax + by +cz + d = 0;
  // u = [a,b,c]
  m_boundaries_planes.push_back({1,0,0,-bottom_left_vertex[0]});
  m_boundaries_planes.push_back({0,1,0,-bottom_left_vertex[1]});
  m_boundaries_planes.push_back({0,0,1,-bottom_left_vertex[2]});
  m_boundaries_planes.push_back({1,0,0,-top_right_vertex[0]});
  m_boundaries_planes.push_back({0,1,0,-top_right_vertex[1]});
  m_boundaries_planes.push_back({0,0,1,-top_right_vertex[2]});
  
  uint16_t n(0);
  std::transform(m_idx.begin(), m_idx.end(), m_idx.begin(), [&](size_t k){ return n++; });
}

bool OccupancyGrid::updateGridFromPoint( const xyz_t& pA, const double radius_bb )
{

  assert( pA.size() == 3 );
  
  if( !inRange(pA) )
    return false;
          
  if( m_terminate ) 
  {
    std::cerr << "Error, already called the terminate() method"  << std::endl;
    std::cerr << "The vector has already been sorted (terminate() has been called), thus no operation is allowed" << std::endl;
    return false;
  }
  
  uvt_t uvt; 
  xyz2uvt( pA, &uvt );
  
  static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
  static uint64_t points_per_line  = m_data->grid_size_z;

  int iU = uvt.at(0);
  int iV = uvt.at(1);
  int iT = uvt.at(2);

  if( iU  <0 || iV < 0 || iT < 0 )  // if the start point is close to the boundary, it could be that part the boundig box is out of the boundaries
    return false;
      
  if( (iU * points_per_plane + iV * points_per_line + iT) >= m_ogrid_size )  // if the start point is close to the boundary, it could be that part the boundig box is out of the boundaries
    return false;
      
  m_idx_ogrid.push_back( iU * points_per_plane + iV * points_per_line + iT );
  
  return true; 
}

bool OccupancyGrid::updateGridFromLine( const xyz_t& pA, const xyz_t& pB, const double radius_bb )
{
  
  xyz_t _pA = pA, _pB = pB;

  if( !inRange(pA) && !inRange(pB)  )
    return false;
  
  if( dist( pA, pB ) <= m_data->ds )
    return false;
  
  if( !inRange(pA) || !inRange(pB)  )
  {
    COUTV("inRange(pA)? "<< inRange(pA) << ", inRange(pB)? " << inRange(pB) << std::endl, 1 );
    enforceBoundaries(pA, pB, &_pA, &_pB);
  }
  
  if( dist( _pA, _pB ) <= m_data->ds )
    return false;

  printv("BL: ", m_data->blv, 5);    
  printv("TR: ", m_data->trv, 5);    
  printv("pA : ", pA, 5);
  printv("pB : ", pB, 5);
  printv("_pA: ", _pA, 5);
  printv("_pB: ", _pB, 5);
  
  if( m_terminate ) 
  {
    std::cerr << "Error, already called the terminate() method"  << std::endl;
    std::cerr << "The vector has already been sorted (terminate() has been called), thus no operation is allowed" << std::endl;
    return false;
  }
  
  std::vector< uvt_t > line_uvt; 
  piecewiseLine( _pA, _pB, &line_uvt );
  if( line_uvt.size() == 0 )
    return false;

  // 6# create the grid of points inside the bounding box 
  // 6.1# the radius projected along a coordinated plane (xy, xz, or yz )
  xyz_t ax;
  axis( _pA, _pB, &ax );
  printv( "--> axis   [m] ", ax, 1);
  size_t projection_direction = ( fabs( ax[0] ) > fabs( ax[1] ) )  && ( fabs( ax[0] ) > fabs( ax[2] ) ) ? 0
                              : ( fabs( ax[1] ) > fabs( ax[0] ) )  && ( fabs( ax[1] ) > fabs( ax[2] ) ) ? 1
                              : 2;
  double radius_m = radius_bb /fabs( ax[projection_direction]  ); 
  uint16_t radius_u = radius_m / m_data->ds; 
  COUTV ( "--> radius [m] " << radius_m << std::endl, 1 );
  COUTV ( "--> radius [u] " << radius_u << std::endl, 1 );
  
  // 6.2# create the grid of points (centered in 0) inside the bb projected in the plane xdy, or x, or yz
  std::vector< std::vector< int > >  delta;
  delta.push_back( {0,0,0} );
  for( size_t iRay = 0;iRay <m_angles.size(); iRay++)
  {
    for( size_t iStep = 0; iStep < radius_u; iStep++ )
    {
      int u=0, v=0, t=0;
      if( projection_direction == 0 )
      {
        u = 0;
        v = iStep * cos( m_angles[iRay] );
        t = iStep * sin( m_angles[iRay] ) ;
        delta.push_back( {u, v , t  } );
      }
      else if( projection_direction == 1 )
      {
        u = iStep * cos( m_angles[iRay] );
        v = 0;
        t = iStep * sin( m_angles[iRay] );
        delta.push_back( {u, v, t } );
      }
      else if( projection_direction == 2 )
      {
        u = iStep * cos( m_angles[iRay] );
        v = iStep * sin( m_angles[iRay] );
        t = 0;
      }
      delta.push_back( {u, v, t  } );
    }
  }
  //updateCylindricBB( radius, projection_direction, &m_delta );

  static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
  static uint64_t points_per_line  = m_data->grid_size_z;

  COUTV ( "--> update the grid " << std::endl, 1 );
  for( size_t ip = 0; ip < line_uvt.size(); ip++ )
  {
    for( size_t id = 0; id < delta.size() ; id++ )
    {
      int iU = line_uvt[ip].at(0) + delta[id].at(0);
      int iV = line_uvt[ip].at(1) + delta[id].at(1);
      int iT = line_uvt[ip].at(2) + delta[id].at(2);

      if( iU  <0 || iV < 0 || iT < 0 )  // if the start point is close to the boundary, it could be that part the boundig box is out of the boundaries
        continue;
      
      if( (iU * points_per_plane + iV * points_per_line + iT) >= m_ogrid_size )  // if the start point is close to the boundary, it could be that part the boundig box is out of the boundaries
        continue;
      
      m_idx_ogrid.push_back( iU * points_per_plane + iV * points_per_line + iT );
    }
  }
  
  return true; 
}


bool OccupancyGrid::updateGridFromTrapezoid( const xyz_t& pA, const xyz_t& pB, const xyz_t& pC, const xyz_t& pD, const double offset_bb, grid_error_t* error_code )
{
  assert( pA.size() == 3 );
  assert( pB.size() == 3 );
  assert( pC.size() == 3 );
  assert( pD.size() == 3 );

      
  if( !inRange(pA) && !inRange(pB)  && !inRange(pC) && !inRange(pD)  )
  {
    *error_code = OUT_OF_RANGE;
    return false;
  }
  if( dist( pA, pB ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }
  if( dist( pA, pC ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }
  if( dist( pA, pD ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }
  if( dist( pB, pC ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }
  if( dist( pB, pD ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }
  if( dist( pC, pD ) <= m_data->ds ) { std::cout << " updateGridFromTrapezoid exit at " << __LINE__ << std::endl; return false; }

  if( m_terminate ) 
  {
    *error_code = NOT_ALLOWED_OPERATION;
    std::cerr << "Error, already called the terminate() method"  << std::endl;
    std::cerr << "The vector has already been sorted (terminate() has been called), thus no operation is allowed" << std::endl;
    return false;
  }
  /////////////////////////////////////////////////////////////////
  // Input sorting
  std::vector< xyz_t > vertexes_xyz;
  // 1# vector from A to B, that is (B-A)
  printv( "[INPUT] A [m] (inside? "  + std::to_string( inRange(pA)  ) + ") ", pA, 1); 
  printv( "[INPUT] B [m] (inside? "  + std::to_string( inRange(pB)  ) + ") ", pB, 1); 
  printv( "[INPUT] C [m] (inside? "  + std::to_string( inRange(pC)  ) + ") ", pC, 1); 
  printv( "[INPUT] D [m] (inside? "  + std::to_string( inRange(pD)  ) + ") ", pD, 1); 
  
  vertexes_xyz.push_back(pA); 
  vertexes_xyz.push_back(pB);
  vertexes_xyz.push_back(pC);
  vertexes_xyz.push_back(pD);

  // Ordering Vertexes Top Right (TR), Top Left (TL), Bottom Right (BR), Bottom Left (BL)
  xyz_t TR, TL, BR, BL;
  
  find( vertexes_xyz, "TR", &TR ); 
  find( vertexes_xyz, "TL", &TL ); 
  find( vertexes_xyz, "BR", &BR ); 
  find( vertexes_xyz, "BL", &BL ); 
  // End input sorting
  /////////////////////////////////////////////////////////////////
  
  printv("--> TR [m] (inside? "  + std::to_string( inRange(TR)  ) + ") ", TR , 1); 
  printv("--> TL [m] (inside? "  + std::to_string( inRange(TL)  ) + ") ", TL , 1); 
  printv("--> BR [m] (inside? "  + std::to_string( inRange(BR)  ) + ") ", BR , 1); 
  printv("--> BL [m] (inside? "  + std::to_string( inRange(BL)  ) + ") ", BL , 1); 
  
  while( !inRange( TR ) || !inRange( TL ) || !inRange( BR ) || !inRange( BL ) )
  {
    xyz_t TR_=TR;
    xyz_t TL_=TL;
    xyz_t BR_=BR;
    xyz_t BL_=BL;;

    if( inRange( TR ) ) 
    {
      if( !inRange( TL )  ) { enforceBoundaries(TR, TL, &TR_, &TL_); TL = TL_; }
      if( !inRange( BR )  ) { enforceBoundaries(TR, BR, &TR_, &BR_); BR = BR_; }
    }
    if( inRange( TL ) ) 
    {
      if( !inRange( TR )  ) { enforceBoundaries(TL, TR, &TL_, &TR_); TR = TR_; }
      if( !inRange( BL )  ) { enforceBoundaries(TL, BL, &TL_, &BL_); BL = BL_; }
    }
    if( inRange( BR ) ) 
    {
      if( !inRange( TR )  ) { enforceBoundaries(BR, TR, &BR_, &TR_); TR = TR_; }
      if( !inRange( BL )  ) { enforceBoundaries(BR, BL, &BR_, &BL_); BL = BL_; }
    }
    if( inRange( BL ) ) 
    {
      if( !inRange( BR )  ) { enforceBoundaries(BL, BR, &BL_, &BR_); BR = BR_; }
      if( !inRange( TL )  ) { enforceBoundaries(BL, TL, &BL_, &TL_); TL = TL_; }
    }
  }

  if( dist( TR, TL ) <= m_data->ds ) { *error_code = POINTS_OVERLAP; return false;}
  if( dist( TR, BR ) <= m_data->ds ) { *error_code = POINTS_OVERLAP; return false;}
  if( dist( TL, BL ) <= m_data->ds ) { *error_code = POINTS_OVERLAP; return false;}
  if( dist( BL, BR ) <= m_data->ds ) { *error_code = POINTS_OVERLAP; return false;}
  
  printv("--> TR [m] (inside? "  + std::to_string( inRange(TR)  ) + ") ", TR , 1); 
  printv("--> TL [m] (inside? "  + std::to_string( inRange(TL)  ) + ") ", TL , 1); 
  printv("--> BR [m] (inside? "  + std::to_string( inRange(BR)  ) + ") ", BR , 1); 
  printv("--> BL [m] (inside? "  + std::to_string( inRange(BL)  ) + ") ", BL , 1); 

  /////////////////////////////////////////////////////////////////
  // Define the top and bottom lines
  COUTV("TOP LINE CALCULUS"<<std::endl, 5); 
  std::vector< uvt_t > line_top, line_bottom;
  piecewiseLine(TL, TR, &line_top);
  if( line_top.size() == 0 )
  {
    *error_code = WEIRD; 
    return false;
  }
  
  printvv("linetop", line_top, 5);
  
  COUTV("BOTTOM LINE CALCULUS"<<std::endl, 5); 
  piecewiseLine(BL, BR, &line_bottom);
  if( line_bottom.size() == 0 )
  {
    *error_code = WEIRD;
    return false;
  }

  
  printvv("linebottom", line_bottom, 1);
  
  xyz_t dir_top, dir_bottom;
  axis(TL, TR, &dir_top   , 1 ); // unit vector with projection on y > 0
  axis(TL, TR, &dir_bottom, 1 ); // unit vector with projection on y > 0
  
  std::vector<int> offset_top_xp    = { int(  (offset_bb * dir_top[1]) / m_data->ds     ),int( -( offset_bb * dir_top[0] ) / m_data->ds), 0 };
  std::vector<int> offset_top_xn    = { int( -(offset_bb * dir_top[1]) / m_data->ds     ),int(  ( offset_bb * dir_top[0] ) / m_data->ds), 0 };
  std::vector<int> offset_bottom_xp = { int(  (offset_bb * dir_bottom[1]) / m_data->ds  ),int( -( offset_bb * dir_bottom[0] ) / m_data->ds), 0 };      
  std::vector<int> offset_bottom_xn = { int( -(offset_bb * dir_bottom[1]) / m_data->ds  ),int(  ( offset_bb * dir_bottom[0] ) / m_data->ds), 0 };      

  std::vector< uvt_t > line_top_xp, line_top_xn;
  COUTV("OFFSET TOP P LINE CALCULUS"<<std::endl, 5); 
  offsetLine(line_top, offset_top_xp, &line_top_xp);
  printvv("line_top_xp", line_top_xp, 5); 
  COUTV("OFFSET TOP N LINE CALCULUS "<<std::endl, 5); 
  offsetLine(line_top, offset_top_xn, &line_top_xn);
  printvv("line_top_xn", line_top_xn, 5); 
  
  COUTV("OFFSET BOTTOM P LINE CALCULUS"<<std::endl, 5); 
  std::vector< uvt_t > line_bottom_xp, line_bottom_xn;
  offsetLine(line_bottom, offset_bottom_xp, &line_bottom_xp);
  printv("offset_bottom_xp", offset_bottom_xp, 5); 
  printvv("line_bottom_xp", line_bottom_xp, 5); 
  COUTV("OFFSET BOTTOM N LINE CALCULUS"<<std::endl, 5); 
  offsetLine(line_bottom, offset_bottom_xn, &line_bottom_xn);
  printvv("line_bottom_xn", line_bottom_xn, 5); 
  // End - Define the top and bottom lines
  /////////////////////////////////////////////////////////////////
  
  ////////////////////////////////////////////////////////////////
  // create a set of lines from bottom to top (for each point of the bottom lines a line is connecte with one point in the top line)
  std::vector< std::vector< uvt_t > > lines_from_bottom_to_top_xp;
  std::vector< std::vector< uvt_t > > lines_from_bottom_to_top_xn;
  COUTV("SET OF LINES P"<<std::endl, 5); 
  lineSetFromLines ( line_bottom_xp, line_top_xp, &lines_from_bottom_to_top_xp );
  if( lines_from_bottom_to_top_xp.size() == 0 )
  {
    *error_code = WEIRD;
    return false;
  }
  
  COUTV("SET OF LINES N"<<std::endl, 5); 
  lineSetFromLines ( line_bottom_xn, line_top_xn, &lines_from_bottom_to_top_xn );

  if( lines_from_bottom_to_top_xn.size() == 0 )
  {
    *error_code = WEIRD;
    return false;
  }
  
  assert( lines_from_bottom_to_top_xp.size() == lines_from_bottom_to_top_xn.size() );

  static uint64_t points_per_plane = m_data->grid_size_y*m_data->grid_size_z;
  static uint64_t points_per_line  = m_data->grid_size_z;
  
  std::vector< uvt_t > grid;
  for( size_t i=0; i< lines_from_bottom_to_top_xp.size(); i++ )
  {
    COUTV("GRID OF LINES"<<std::endl, 5); 
    gridFromLines( lines_from_bottom_to_top_xp[i], lines_from_bottom_to_top_xn[i], &grid);
    for( std::vector< uvt_t >::const_iterator it = grid.begin(); it != grid.end(); it++ )   
    {
      if( (it->at(0) * points_per_plane + it->at(1) * points_per_line + it->at(2) ) >= m_ogrid_size )  // if the start point is close to the boundary, it could be that part the boundig box is out of the boundaries
        continue;
      m_idx_ogrid.push_back( it->at(0) * points_per_plane + it->at(1) * points_per_line + it->at(2) );
    }
  }
  
  return true;

}  


void OccupancyGrid::update( )
{
  if( m_idx_ogrid.size() == 0 )
    return;
  
  if( m_total_occurrences == 0 )
    m_start = ros::Time::now();
  
  // in m_idx_ogrid ci sono gli indici dei punti toccati in un ciclo di campionamento
  std::sort( m_idx_ogrid.begin(), m_idx_ogrid.end() );
  std::vector< uint64_t >::iterator it = std::unique( m_idx_ogrid.begin(), m_idx_ogrid.end() );
  m_idx_ogrid.resize( std::distance(m_idx_ogrid.begin(),it) ); 
  
  // in m_tracked_points ci sono il numero di occrorenze che sono state identificate in un dato ciclo di campionamento
  m_tracked_points.push_back( m_idx_ogrid.size( ) );
  
  // in m_idx_ogrid ci sono gli indici dei punti toccati in un ciclo di campionamento
  m_total_occurrences += m_idx_ogrid.size( );
  
  // in m_ogrid metto l'integrale sul tempo delle occorrenze nei differenti punti toccati
  std::for_each( m_idx_ogrid.begin(), it, [&]( uint64_t idx ){ assert( idx < m_ogrid_size ); m_ogrid[idx] += 1.0; } );
  
  m_idx_ogrid.clear();
  
} 

void OccupancyGrid::terminate( )
{
  if( m_terminate ) 
  {
    std::cerr << "Error, already called the terminate() method"  << std::endl;
    throw std::runtime_error("The vector has already been sorted (terminate() has been called), thus no operation is allowed");
  }
  
  m_end = ros::Time::now();
  
  uint64_t n(0);
  std::transform(m_idx.begin(), m_idx.end(), m_idx.begin(), [&](uint64_t k){ return n++; });

  // sort indexes based on comparing values in v
  std::sort(m_idx.begin(), m_idx.end(), [&](uint64_t i1, uint64_t i2) {return m_ogrid.at(i1) < m_ogrid.at(i2);} );
  std::sort(m_ogrid.begin(), m_ogrid.end() );
  
  m_terminate = true;
  
  std::cout << "[ OccupancyGrid::terminate() ] n. of points " << m_ogrid.size() << std::endl;
  m_data->points.clear();
  for( uint64_t i = 0; i < m_ogrid.size(); i++ )
  {
    if( m_ogrid[i] == 0 )
      continue;
      
    itia_occupancy_volume::GridNode gn;
    gn.idx = m_idx[i];
    i2xyz(gn.idx, &gn.xyz );
    gn.hval = m_ogrid[i];
    gn.rval = 0;
    m_data->points.push_back( gn );
  }

  std::cout << "[ OccupancyGrid::terminate() ] n. of tracked points " << m_tracked_points.get().size() << std::endl;
  for( size_t i = 0; i < m_tracked_points.get().size(); i++ )
    m_data->samples.push_back( m_tracked_points.get().at(i) );
  
  m_data->mean_samples  = std::accumulate (m_data->samples.begin(), m_data->samples.end(), 0 ) / m_data->samples.size();
  m_data->min_samples   = *(std::min_element(m_data->samples.begin(), m_data->samples.end() ) );
  m_data->max_samples   = *(std::max_element(m_data->samples.begin(), m_data->samples.end() ) );

}

double OccupancyGrid::getAcquisitionTime( ) const
{
  return ( m_end - m_start ).toSec();
}

void OccupancyGrid::enforceBoundaries( const xyz_t& pA, const xyz_t& pB, xyz_t* pIn,  xyz_t*  pInt )
{
  assert( pIn != NULL );
  assert( pInt != NULL );
  *pIn = pA;
  *pInt = pB;
  if( inRange(pA) && inRange( pB ) )
    return;
  
  *pIn = inRange(pA) ? pA : pB;
  *pInt = *pIn;
  xyz_t pOut = inRange(pA) ? pB : pA;
  
  // line: pIn + lambda axis(pIn,pOut)
  // Plane: ax + by +cz + d = 0;
  // ----> intersection:  
  // a pIn[0] + b pIn[1] + c pIn[2] + d + lambda ( a * axis(pIn,pOut)[0] ..... ) = 0
  // lambda = -( a pIn[0] + b pIn[1] + c pIn[2] + d ) / ( a * axis(pIn,pOut)[0] ..... )
  // pInt = pIn -( a pIn[0] + b pIn[1] + c pIn[2] + d ) / ( a * axis(pIn,pOut)[0] ..... ) * asix()
  xyz_t ax;
  axis( *pIn, pOut, &ax );
  
  for( size_t i_plane = 0; i_plane < m_boundaries_planes.size(); i_plane++ )
  {
    COUTV("Check plane " << i_plane << "/" << m_boundaries_planes.size() << std::endl, 1 );

    double den = 0;
    double num = m_boundaries_planes[i_plane][3];
    for( size_t i_element=0; i_element<3;i_element++)
    {
      num += pIn->at(i_element) * m_boundaries_planes[i_plane][i_element];
      den += ax[i_element]      * m_boundaries_planes[i_plane][i_element];
    }
    if( den == 0 )
      continue;
    
    double lambda = -num / den;
    *pInt = xyz_t ( { pIn->at(0) + lambda*ax[0], pIn->at(1)+lambda*ax[1], pIn->at(2)+lambda*ax[2] } );

    if( inside( *pIn, pOut, *pInt ) )
    {
      COUTV("Inside! plane: " << i_plane << std::endl, 1 );
      return;
    }
  }
}


}
