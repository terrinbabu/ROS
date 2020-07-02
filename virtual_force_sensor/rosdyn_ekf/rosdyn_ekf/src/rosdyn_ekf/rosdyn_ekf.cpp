#include <rosdyn_ekf/rosdyn_ekf.h>

namespace rosdyn {

EKFilter::EKFilter(const ChainPtr &chain, const double& sampling_period):
  m_chain(chain),
  m_sampling_period(sampling_period)
{
  m_dof=m_chain->getActiveJointsNumber();

  delta_dq=0.01*m_chain->getDQMax();
  delta_q=Eigen::VectorXd::Constant(m_dof,0.001);//delta_dq*m_sampling_period;

  m_x.resize(m_dof*2);
  m_x.setZero();

  m_next_x.resize(m_dof*2);
  m_next_x.setZero();

  m_ddq.resize(m_dof);
  m_ddq.setZero();

  m_z.resize(m_dof);
  m_dz.resize(m_dof);

  m_F.resize(m_dof*2,m_dof*2);
  m_F.setZero();

  m_F_continuos.resize(m_dof*2,m_dof*2);
  m_F_continuos.setZero();

  m_I=Eigen::MatrixXd::Identity(m_dof,m_dof);
  m_big_I=Eigen::MatrixXd::Identity(2*m_dof,2*m_dof);

  m_F_continuos.block(0,0,m_dof,m_dof)=m_I;

  m_H=m_big_I;

  m_P.resize(m_x.rows(),m_x.rows());
  m_P.setIdentity();

  m_Q.resize(m_x.rows(),m_x.rows());
  m_Q.setIdentity();

  m_measure.resize(2*m_dof);
  m_output.resize(2*m_dof);

  m_c0.resize(m_dof);
  m_c1.resize(m_dof);
  m_sigma0.resize(m_dof);
  m_sigma1.resize(m_dof);
  m_sigma0_over_c0.resize(m_dof);

  //   m_c0.setConstant(25.79); 
  //   m_c1.setConstant(18.10);
  
  m_c0(0) = 15.51; m_c0(1) = 13.92; m_c0(2) = 8.12;
  m_c0(3) = 2.55 ; m_c0(4) = 2.58 ; m_c0(5) = 2.26;
  m_c1(0) = 25.79; m_c1(1) = 18.10; m_c1(2) = 2.73;
  m_c1(3) = 3.37 ; m_c1(4) = 3.59 ; m_c1(5) = 2.64;

  ros::NodeHandle nh;
  double sigma0,sigma1;
  if (!nh.getParam("ekf/sigma0",sigma0))
    sigma0=5000; // was s0 = 500 literature sigma0 is 5000 (starting from 2000-10000)
  if (!nh.getParam("ekf/sigma1",sigma1)) 
    sigma1=1000; // was s0 = 1000 literature sigma1 is 20-30 (starting from 20-30)

  m_sigma0.setConstant(sigma0);
  m_sigma1.setConstant(sigma1);
  m_sigma0_over_c0=m_sigma0.cwiseQuotient(m_c0);
}

bool EKFilter::initialize(const Eigen::VectorXd &q,
                          const Eigen::VectorXd &dq,
                          const Eigen::MatrixXd &Q,
                          const Eigen::MatrixXd &R)
{
  if (q.rows()!=m_dof)
  {
    ROS_ERROR("q size is %zu instead of %u",q.rows(),m_dof);
    return false;
  }

  if (dq.rows()!=m_dof)
  {
    ROS_ERROR("dq size is %zu instead of %u",q.rows(),m_dof);
    return false;
  }

  if (Q.rows()!=2*m_dof)
  {
    ROS_ERROR("Q rows is %zu instead of %u",Q.rows(),2*m_dof);
    return false;
  }

  if (Q.cols()!=2*m_dof)
  {
    ROS_ERROR("Q cols is %zu instead of %u",Q.cols(),2*m_dof);
    return false;
  }

  if (R.rows()!=2*m_dof)
  {
    ROS_ERROR("P rows is %zu instead of %u",R.rows(),2*m_dof);
    return false;
  }

  if (R.cols()!=2*m_dof)
  {
    ROS_ERROR("R cols is %zu instead of %u",R.cols(),2*m_dof);
    return false;
  }


  m_Q=Q;
  m_P=Q;
  m_R=R;

  m_x.segment(0,m_dof)=q;
  m_x.segment(m_dof,m_dof)=dq;
  m_z=dq.cwiseSign();

  return true;
}


void EKFilter::update(const Eigen::VectorXd &q_meas,
                      const Eigen::VectorXd &dq_meas,
                      const Eigen::VectorXd &tau_meas,
                      Eigen::VectorXd &q_estim,
                      Eigen::VectorXd &dq_estim,
                      Eigen::VectorXd &ddq_estim)
{
  m_measure.segment(0,m_dof)=q_meas;
  m_measure.segment(m_dof,m_dof)=dq_meas;

  prioriEstimation(tau_meas); // prediction
  posterioriEstimation(m_measure); // after measurement

  computeOutput();
  q_estim   = m_x.segment(0,m_dof);
  dq_estim  = m_x.segment(m_dof,m_dof);
  ddq_estim = m_ddq = computeAcceleration(m_x.segment(0,m_dof),m_x.segment(m_dof,m_dof),tau_meas);

}

void EKFilter::prioriEstimation(const Eigen::VectorXd& tau)
{

  m_P= m_F*m_P*m_F.transpose()+m_Q; // P(k|k-1)

  stateUpdate(m_sampling_period,tau);

//  for (unsigned int idx=0;idx<100;idx++)
//    stateUpdate(0.01*m_sampling_period);
  linearize(tau);
  computeOutput();
}

void EKFilter::posterioriEstimation(const Eigen::VectorXd& measure)
{

  m_K=m_P*m_H.transpose()*((m_H*m_P*m_H.transpose()+m_R).inverse());
  m_x=m_x+m_K*(measure-m_output);
  m_P=m_P-m_K*m_H*m_P;
}

Eigen::VectorXd EKFilter::computeAcceleration(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd &tau)
{
  m_dz=dq-m_sigma0_over_c0.cwiseProduct(dq.cwiseAbs()).cwiseProduct(m_z);

//   m_tau_f=m_sigma0.cwiseProduct(m_z)+
//           m_sigma1.cwiseProduct(m_dz)+
//           m_c1.cwiseProduct(dq);
  m_tau_f=m_c0*dq.cwiseAbs()+m_c1*dq;
  
  return m_chain->getJointInertia(q).inverse()*(tau-m_chain->getJointTorqueNonLinearPart(q,dq)-m_tau_f);

}

void EKFilter::stateUpdate(const double sampling_period, const Eigen::VectorXd& tau)
{
  Eigen::VectorBlock<Eigen::VectorXd> q   = m_x.segment(      0,m_dof);
  Eigen::VectorBlock<Eigen::VectorXd> dq  = m_x.segment(  m_dof,m_dof);

  Eigen::VectorBlock<Eigen::VectorXd> next_q   = m_next_x.segment(      0,m_dof);
  Eigen::VectorBlock<Eigen::VectorXd> next_dq  = m_next_x.segment(  m_dof,m_dof);

  m_ddq=computeAcceleration(q,dq,tau);

  next_q=q+
         dq*sampling_period+
         m_ddq*std::pow(sampling_period,2)*0.5;

  next_dq=dq+m_ddq*sampling_period;

  m_next_z=m_z+m_dz*sampling_period;

  m_x=m_next_x;
  m_z=m_next_z;


}

void EKFilter::computeOutput()
{
  m_output=m_x;
}

void EKFilter::linearize(const Eigen::VectorXd& tau)
{
  Eigen::VectorBlock<Eigen::VectorXd> q   = m_x.segment(      0,m_dof);
  Eigen::VectorBlock<Eigen::VectorXd> dq  = m_x.segment(  m_dof,m_dof);

  // d(ddq)/d(q)
  // numerical derivative of ddq=B\(tau-NL(q,dq)-m_tau_f) w.r.t q
  for (unsigned int idof=0;idof<m_dof;idof++)
  {
    Eigen::VectorXd qp=q;
    qp(idof)=q(idof)+delta_q(idof);
    Eigen::VectorXd qm=q;
    qm(idof)=q(idof)-delta_q(idof);

    m_F_continuos.block(m_dof,idof,m_dof,1)=(computeAcceleration(qp,dq,tau)-computeAcceleration(qm,dq,tau))/(2*delta_q(idof));
  }

  // dtau/d(dq)

  // numerical derivative of B*ddq+NL(q,dq) w.r.t dq
  for (unsigned int idof=0;idof<m_dof;idof++)
  {
    Eigen::VectorXd dqp=dq;
    dqp(idof)=dq(idof)+delta_dq(idof);
    Eigen::VectorXd dqm=dq;
    dqm(idof)=dq(idof)-delta_dq(idof);

    m_F_continuos.block(m_dof,m_dof+idof,m_dof,1)=(computeAcceleration(q,dqp,tau)-computeAcceleration(q,dqm,tau))/(2*delta_dq(idof));
  }

//   m_F=m_big_I+m_F_continuos*m_sampling_period;
  m_F=(m_big_I-m_F_continuos*m_sampling_period).inverse();


}

void EKFilter::setQR(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
  m_Q=Q;
  m_R=R;
}
}
