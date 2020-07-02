#ifndef __ITIA_MUTILS__QUATERNIONS__UTILS__
#define __ITIA_MUTILS__QUATERNIONS__UTILS__

# include <Eigen/Geometry>
# include <boost/math/special_functions/binomial.hpp>
# include <itia_mutils/norm_derivative.h>

namespace itia
{
namespace mutils
{

inline Eigen::Affine3d VectorToAffine(const Eigen::VectorXd& vec)
{
  Eigen::Affine3d T;
  T.translation() =vec.block(0, 0, 3, 1);
  Eigen::Quaterniond q;
  q.coeffs() = vec.block(3, 0, 4, 1);
  T.linear() = q.toRotationMatrix();
  return T;
};

inline Eigen::Affine3d VectorToAffine(const Eigen::MatrixXd& motion, Eigen::MatrixXd* twists)
{
  int n_derivative = motion.cols()-1;
  
  twists->resize(6, n_derivative);
  twists->setZero();
  Eigen::Affine3d T;
  T.translation() =motion.block(0, 0, 3, 1);
  
  std::vector<Eigen::Quaterniond> qvec(n_derivative+1);
  
  qvec.at(0).coeffs() = motion.block(3, 0, 4, 1);
  Eigen::Quaterniond qinv = qvec.at(0).inverse();
  T.linear() = qvec.at(0).toRotationMatrix();
  
  // Dq    = 0.5*w*q;
  // DDq   = 0.5*(Dw*q+w*Dq)
  // DDDq  = 0.5*(DDw*q+2*Dw*Dq+w*DDq)
  // DDDDq = 0.5*(DDDw*q+3*DDw*Dq+3*Dw*DDq+w*DDDq)
  
  // w      = (2*Dq )*qinv
  // Dw     = (2*DDq-w*Dq)*qinv = 2* DDq*qinv - w * (Dq*qinv) = 2* DDq*qinv - 0.5* w * w
  // DDw    = (2*DDDq-2*Dw*Dq-w*DDq)*qinv
  // DDDw   = (2*DDDDq-3*DDw*Dq-3*Dw*DDq-w*DDDq)*qinv

  std::vector<Eigen::Quaterniond> derw(n_derivative);

  for (unsigned int idx_der = 1;idx_der <= n_derivative;idx_der++)
  {
    derw.at(idx_der-1).coeffs() =2*(qvec.at(idx_der) *qinv).coeffs();
    for (unsigned int idx = 1;idx<(idx_der);idx++)
    {
      double coeff = boost::math::binomial_coefficient<double>(idx_der-1, idx);
      int idx_1 = idx_der-idx-1;
      derw.at(idx_der-1).coeffs()-= coeff * (derw.at(idx_1) * qvec.at(idx) *qinv).coeffs();
    }
    
    twists->block(0, idx_der-1, 3, 1) = motion.block(0, idx_der, 3, 1);
    twists->block(3, idx_der-1, 3, 1) = derw.at(idx_der-1).coeffs().block(0, 0, 3, 1);
    ROS_INFO_STREAM("T = " << qvec.at(idx_der).coeffs());
  }
  
  return T;
};

inline Eigen::VectorXd AffineToVector(const Eigen::Affine3d& T)
{
  Eigen::VectorXd pose(7);
  pose.block(0, 0, 3, 1) =T.translation();
  pose.block(3, 0, 4, 1) =Eigen::Quaterniond(T.linear()).coeffs();
  return pose;
}

inline Eigen::MatrixXd AffineToVector(const Eigen::Affine3d& T, const Eigen::MatrixXd& twists)
{
  ROS_INFO("qui");
  int n_derivative = twists.cols();
  Eigen::MatrixXd motion(7, n_derivative+1);
  motion.col(0) = AffineToVector(T);
  
  std::vector<Eigen::Quaterniond> qvec(n_derivative+1);
  std::vector<Eigen::Quaterniond> derw(n_derivative);
  qvec.at(0).coeffs() = motion.block(3, 0, 4, 1);
  
  // Dq    = 0.5*w*q;
  // DDq   = 0.5*(Dw*q+w*Dq)
  // DDDq  = 0.5*(DDw*q+2*Dw*Dq+w*DDq)
  // DDDDq = 0.5*(DDDw*q+3*DDw*Dq+3*Dw*DDq+w*DDDq)
  
  for (unsigned int idx_der = 1;idx_der <= n_derivative;idx_der++)
  {
    derw.at(idx_der-1) = Eigen::Quaterniond( 0, twists(3, idx_der-1), twists(4, idx_der-1), twists(5, idx_der-1) );
    qvec.at(idx_der) =Eigen::Quaterniond( 0, 0, 0, 0);
    for (unsigned int idx = 0;idx<(idx_der);idx++)
    {
      double coeff = boost::math::binomial_coefficient<double>(idx_der-1, idx);
      int idx_1 = idx_der-idx-1;
      qvec.at(idx_der).coeffs()+= coeff * (derw.at(idx) * qvec.at(idx_1)).coeffs();
    }
    qvec.at(idx_der).coeffs() *=0.5;
    motion.block(0, idx_der, 3, 1) =twists.block(0, idx_der-1, 3, 1);
    motion.block(3, idx_der, 4, 1) =qvec.at(idx_der).coeffs();
  }
  
  
  return motion;
}

inline Eigen::MatrixXd PolyVectorToQuatVector(const Eigen::MatrixXd& motion, const double& toll = 1e-2)
{
  
  // q    =    p * 1/norm(p) 
  // Dq   =   Dp * 1/norm(p) +     p * D(1/norm(p))
  // DDq  =  DDp * 1/norm(p) +  2*Dp * D(1/norm(p)) +    p * DD(1/norm(p))
  // DDDq = DDDp * 1/norm(p) + 3*DDp * D(1/norm(p)) + 3*Dp * DD(1/norm(p)) + p * DDD(1/norm(p))
  
  // (p)^c   = (p'*p)^c
  // D(p)^c  = c*(p'*p)^(c-1) + (p'*p)^c * 2*p'*Dp
  // DD(p)^c = c*(p'*p)^(c-2) + D(p'*p)^c * 2*p'*Dp + (p'*p)^c * 2*Dp'*Dp + (p'*p)^c * 2*p'*DDp

  Eigen::MatrixXd quat_motion = motion;
  int nder = motion.cols()-1;
  
  Eigen::VectorXd p = motion.block(3, 0, 4, 1);
  
  double norm=p.norm();
  if (norm<toll){
    printf("[PolynomialInterpolator::normalizePolyValuve] quaternion norm too low!\n");
    norm=toll;
  }
  
  // q=p/norm;
  quat_motion.block(3, 0, 4, 1) = p/norm;
  if (nder>0)
  {
    Eigen::VectorXd dp = motion.block(3, 1, 4, 1);
    double dnorm=p.dot(dp)/norm;
    // Dq=( Dp*norm - p*Dnorm ) /norm^2
    quat_motion.block(3, 1, 4, 1) = (dp*norm-p*dnorm)/pow(norm, 2);
    if (nder>1)
    {
      Eigen::VectorXd ddp = motion.block(3, 2, 4, 1);
      double ddnorm=( (p.dot(ddp)+dp.dot(dp))*norm - (p.dot(dp))*dnorm )/pow(norm,2.0);
      // DDq=( (DDp*norm-p*DDnorm )*norm^2 - (Dp*norm - p*Dnorm)*2*norm*Dnorm)/norm^4
      quat_motion.block(3, 2, 4, 1) = ((ddp*norm-p*ddnorm) *pow(norm, 2) - (dp*norm - p*dnorm)*2*norm*dnorm)/pow(norm, 4);
      if (nder>2)
      {
        printf("[itia_mutils/quaternion.h] Not implemented yet\n");
        for (int idx = 2;idx<nder;idx++)
          quat_motion.block(3, idx+1, 4, 1).setZero();
      }
    }
    return quat_motion;
  }
  
  

    
};



}
}


#endif