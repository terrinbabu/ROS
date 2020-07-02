#include "UtilFunctions.h"

namespace assistance_policy {

double LogSumExp(const std::vector<double>& vals)
{
  //double log_normalization_val = 0;
  double max_exp = *std::max_element(vals.begin(), vals.end());

  double exp_sum = 0.0;
  for(size_t i = 0; i < vals.size(); i++) {
    exp_sum += exp(vals[i] - max_exp);
  }

  return (log(exp_sum) + max_exp);
}


double SoftMax(const std::vector<double>& vals)
{
  return LogSumExp(vals);
}

double SoftMin(const std::vector<double>& vals)
{
  std::vector<double> vals_neg(vals.size());
  for (size_t i=0; i < vals.size(); i++)
  {
    vals_neg[i] = -vals[i];
  }

  return -LogSumExp(vals_neg);
}

double XYDistance(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2)
{
  double x_diff = pt2[0]-pt1[0];
  double y_diff = pt2[1]-pt1[1];
  return sqrt(x_diff*x_diff + y_diff*y_diff);
}

//bool ObjRayIntersectionInfo::IntersectWithRay(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point) const
//{
//  Eigen::Vector3d direction = end_point-start_point;
//  double dist_between_points = direction.norm();
//  direction /= dist_between_points;
//
//  //line intersects sphere if this is >= 0
//  Eigen::Vector3d cen_to_start(center_ - start_point);
//
//  //first, make sure the object is forward
//  double dot_val = direction.dot(cen_to_start);
//  if (dot_val < 0)
//  {
//    return false;
//  }
//
//  double sqr_val = dot_val*dot_val - cen_to_start.squaredNorm() + radius_*radius_;
//  if (sqr_val <= 0) 
//  {
//    return false;
//  }
//  sqr_val = sqrt(sqr_val);
//
//  //find the points along direction with intersection
//  double d1 = dot_val + sqr_val;
//  double d2 = dot_val - sqr_val;
//
//
//  if (d1 < 0) //if d1 is behind, both are behind. no intersection)
//  {
//    return false;
//  } else if (d2 > dist_between_points) {//if d2 is too far, both are too far. no intersection.
//    return false;
//  } 
//  //otherwise, one of them is between, and we have intersection
//
//  return true;
//}

} //namespace
