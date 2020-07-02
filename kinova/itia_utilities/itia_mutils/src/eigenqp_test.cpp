#include <itia_mutils/eiquadprog.hpp>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <itia_mutils/frame_distance.h>
#include <itia_mutils/object_on_plane_estimation.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_mutils/quadprog_pc.h>

#include <Eigen/Geometry>
int main(int argc, char **argv){
  ros::init(argc, argv, "itia_mutils_test");
  ros::NodeHandle nh;
  
  
  unsigned int nu;
  unsigned int nc = 2;
  
  Eigen::MatrixXd H;
  itia::rutils::getParam (nh,  "H", H);
  nu = H.rows();
  
  Eigen::VectorXd h;
  Eigen::MatrixXd aux;
  itia::rutils::getParam (nh,  "h", aux);
  h = aux.col(0);
  
  Eigen::MatrixXd A;
  itia::rutils::getParam (nh,  "A", A);
  nc = A.cols();
  
  Eigen::VectorXd bm(nc);
  Eigen::VectorXd bp(nc);
  
  
  itia::rutils::getParam (nh,  "bm", aux);
  bm = aux.col(0);
  
  itia::rutils::getParam (nh,  "bp", aux);
  bp = aux.col(0);
  
  Eigen::VectorXd u(nu);
  Eigen::VectorXd s(nc);
  Eigen::VectorXd lm(nc);
  Eigen::VectorXd lp(nc);
  
  u.setZero();
  
  double mu = 1e-4;
  double toll = 1e-6;
  double nc_inv = 1.0/(nc);
  
  s.setConstant(0.0);
  lm.setConstant(1e0);
  lp.setConstant(1e0);
  
 
  solver::QuadProgPc qp(H, A);
  Eigen::MatrixXd At = A.transpose();
  ros::Time t0 = ros::Time::now();
  int iter = qp.solve(h, bm, bp, u, s, lm, lp);
  //int iter = solver::quadprog_pc(H, h, A, At, bm, bp, u, s, lm, lp, nu, nc, nc_inv, toll, mu);
  double dt = (ros::Time::now()-t0).toSec();
  ROS_INFO_STREAM("iter = " <<  iter);
  ROS_INFO_STREAM("dt = " <<  dt*1e3);
  ROS_INFO_STREAM("u  = " <<  u.transpose());
  ROS_INFO_STREAM("s  = " <<  s.transpose());
  ROS_INFO_STREAM("lm = " <<  lm.transpose());
  ROS_INFO_STREAM("lp = " <<  lp.transpose());
  
  t0 = ros::Time::now();
  iter = qp.solve(h, bm, bp, u, s, lm, lp);
  dt = (ros::Time::now()-t0).toSec();
  ROS_INFO_STREAM("iter = " <<  iter);
  ROS_INFO_STREAM("dt = " <<  dt*1e3);
  
  return 0;  
}