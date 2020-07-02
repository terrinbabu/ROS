#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

namespace assistance_policy {

//constants
const double LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF = 0.01; //from looking at data  0.01




inline double AngleFromQuaternionW(double w)
{
  w = std::min(0.999999, std::max(-0.999999, w));
  double phi = 2.*acos(w);
  return std::min(phi, 2.*M_PI - phi);  //acos goes from 0 to pi, so this is always positive
}

inline double QuaternionDistance(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2)
{
  Eigen::Quaterniond quat_between = quat2*quat1.inverse();
  return AngleFromQuaternionW(quat_between.w());  //acos goes from 0 to pi, so this is always positive
}


inline double QuaternionDot(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2)
{
  return quat1.vec().dot(quat2.vec()) + quat1.w()*quat2.w();
}

//copied from arm MathUtils in case code base is later switched
inline double rand(const double& A, const double& B)
{  
    double s = std::rand() / static_cast<double>(RAND_MAX);
    return (B-A) * s + A;
}

double LogSumExp(const std::vector<double>& vals);
double SoftMax(const std::vector<double>& vals);
double SoftMin(const std::vector<double>& vals);
double XYDistance(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);

//class ObjRayIntersectionInfo
//{
//  public:
//    ObjRayIntersectionInfo() { }
//    ObjRayIntersectionInfo(double radius, const Eigen::Vector3d& center, double height)
//      : radius_(radius), center_(center), height_(height) { };
//
//
//    //computes if ray intersects this object
//    //assumed that direction is normalized
//    //see wikipedia line-sphere_intersection for formula
//    bool IntersectWithRay(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point) const;
//
//    const double& GetHeight() const {return height_;}
//    const Eigen::Vector3d& GetCenter() const {return center_;}
//
//    double radius_;
//    Eigen::Vector3d center_;
//    double height_; //height we must go over this object to clear it
//};

class Point2d
{
  public:
    Point2d (double x, double y) : x_(x), y_(y) { }

    static bool XLessThan(const Point2d& pt1, const Point2d& pt2) {return pt1.x_ < pt2.x_;}
    static bool YLessThan(const Point2d& pt1, const Point2d& pt2) {return pt1.y_ < pt2.y_;}


  double x_;
  double y_;
};


} //namespace
#endif
