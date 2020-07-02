#ifndef rosdyn_ekf_201906025
#define rosdyn_ekf_201906025
#include <rosdyn_core/primitives.h>
namespace rosdyn
{

class EKFilter
{
protected:
  rosdyn::ChainPtr m_chain;

  double m_sampling_period;
  Eigen::VectorXd delta_q;
  Eigen::VectorXd delta_dq;
  unsigned int m_dof;

  Eigen::MatrixXd m_F_continuos;
  Eigen::MatrixXd m_F;
  Eigen::MatrixXd m_H;

  Eigen::VectorXd m_x;
  Eigen::VectorXd m_next_x;

  Eigen::VectorXd m_ddq;
  Eigen::VectorXd m_tau_f;

  Eigen::VectorXd m_z;
  Eigen::VectorXd m_dz;
  Eigen::VectorXd m_next_z;

  Eigen::VectorXd m_output;
  Eigen::VectorXd m_measure;

  Eigen::MatrixXd m_P;

  Eigen::MatrixXd m_Q;
  Eigen::MatrixXd m_R;
  Eigen::MatrixXd m_K;

  Eigen::MatrixXd m_I;
  Eigen::MatrixXd m_big_I;


  // friction coefficients
  Eigen::VectorXd m_c0; // Coloumb coefficients
  Eigen::VectorXd m_c1; // Viscous coefficients

  Eigen::VectorXd m_sigma0; //LuGre bristle stifness
  Eigen::VectorXd m_sigma1; //LuGre bristle microdamping

  Eigen::VectorXd m_sigma0_over_c0; //sigma0/c0
  void linearize(const Eigen::VectorXd& tau);
  void stateUpdate(const double sampling_period,
                   const Eigen::VectorXd& tau);
  void computeOutput();

  Eigen::VectorXd computeAcceleration(const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& dq,
                                      const Eigen::VectorXd& tau);

  void prioriEstimation(const Eigen::VectorXd& tau);
  void posterioriEstimation(const Eigen::VectorXd& measure);

public:
  EKFilter(const rosdyn::ChainPtr& chain, const double& sampling_period);
  bool initialize(const Eigen::VectorXd& q,
                  const Eigen::VectorXd& dq,
                  const Eigen::MatrixXd& Q,
                  const Eigen::MatrixXd& R);


  void update(const Eigen::VectorXd& q_meas,
              const Eigen::VectorXd& dq_meas,
              const Eigen::VectorXd& tau_meas,
              Eigen::VectorXd& q_estim,
              Eigen::VectorXd& dq_estim,
              Eigen::VectorXd& ddq_estim);

  void setQR(const Eigen::MatrixXd& Q,
             const Eigen::MatrixXd& R);

};
}

#endif
