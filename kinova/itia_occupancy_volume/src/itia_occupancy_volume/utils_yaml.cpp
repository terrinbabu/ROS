
#include <itia_occupancy_volume/GridNodes.h>
#include <itia_occupancy_volume/common.h>
#include <itia_occupancy_volume/utils_yaml.h>
#include <itia_rutils/itia_rutils_yaml.h>

namespace itia_occupancy_volume
{
  
bool getParam( const YAML::Node& config, GridNodes& gi )
{
  std::vector<std::vector< double > > _grid_points;
  
  bool ret = true;
  ret &= itia::rutils::getParam                (config["occupancy_grid"], "ds", gi.ds );
  ret &= itia::rutils::getParamArray<double,3> (config["occupancy_grid"], "bottom_left_vertex", gi.blv );
  ret &= itia::rutils::getParamArray<double,3> (config["occupancy_grid"], "top_right_vertex", gi.trv );
  ret &= itia::rutils::getParam                (config["occupancy_grid"], "nrays", gi.nrays );
  ret &= itia::rutils::getParam                (config["occupancy_grid"], "points", _grid_points );
  ret &= itia::rutils::getParamVector          (config["occupancy_grid"], "samples", gi.samples );
  for( auto it = _grid_points.begin(); it != _grid_points.end(); it++ )
  {
    gi.points.push_back( cast( *it ) );
  }
  ret &= itia::rutils::getParam         (config["occupancy_grid" ], "min_samples" , gi.min_samples );
  ret &= itia::rutils::getParam         (config["occupancy_grid" ], "max_samples" , gi.max_samples );
  ret &= itia::rutils::getParam         (config["occupancy_grid" ], "mean_samples", gi.mean_samples);
  
  return ret;
}

}
