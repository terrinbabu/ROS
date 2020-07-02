#include "ros/ros.h"
#include <rosdyn_core/primitives.h>
#include <rosdyn_ekf/rosdyn_ekf.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "test_rosdyn_ekf");
  ros::NodeHandle nh;
  double sampling_period=8e-3;
  ros::Rate lp(1.0/sampling_period);
  ros::AsyncSpinner spin(4);
  spin.start();


  ros::Publisher ekf_pub=nh.advertise<sensor_msgs::JointState>("ekf",1);
  ros::Publisher raw_pub=nh.advertise<sensor_msgs::JointState>("raw",1);
  ros::Publisher ori_pub=nh.advertise<sensor_msgs::JointState>("ori",1);

  sensor_msgs::JointState raw_msg;
  sensor_msgs::JointState ekf_msg;
  sensor_msgs::JointState ori_msg;


  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  std::string base_frame = "ur10_35_base_link";
  std::string tool_frame = "ur10_35_tool0";

  rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  if (!chain)
  {
    ROS_ERROR("chain creation fail");
    return 0;
  }

  unsigned int dof=chain->getActiveJointsNumber();

  raw_msg.name=chain->getMoveableJointNames();
  ekf_msg.name=chain->getMoveableJointNames();
  ori_msg.name=chain->getMoveableJointNames();

  ekf_msg.position.resize(dof);
  ekf_msg.velocity.resize(dof);
  ekf_msg.effort.resize(2*dof);
  raw_msg.position.resize(dof);
  raw_msg.velocity.resize(dof);
  raw_msg.effort.resize(2*dof);
  ori_msg.position.resize(dof);
  ori_msg.velocity.resize(dof);
  ori_msg.effort.resize(2*dof);


  rosdyn::EKFilter ekf(chain,sampling_period);

  Eigen::VectorXd q(dof);
  Eigen::VectorXd dq(dof);
  Eigen::VectorXd ddq(dof);
  Eigen::VectorXd tau(dof);

  Eigen::VectorXd   q_noise(dof);
  Eigen::VectorXd  dq_noise(dof);
  Eigen::VectorXd ddq_noise(dof);
  Eigen::VectorXd tau_noise(dof);

  Eigen::VectorXd   q_est(dof);
  Eigen::VectorXd  dq_est(dof);
  Eigen::VectorXd ddq_est(dof);
  Eigen::VectorXd tau_est(dof);
  q.setZero();
  dq.setZero();
  ddq.setZero();
  tau.setZero();

  Eigen::MatrixXd Q(dof*2,dof*2);
  Eigen::MatrixXd R(dof*2,dof*2);

  Q.setIdentity();
  R.setIdentity();

  double r_q,r_dq;
  if (!nh.getParam("ekf/r_q",r_q))
    r_q=0.001;
  if (!nh.getParam("ekf/r_dq",r_dq))
    r_dq=0.01;

  R.block(0,0,dof,dof)*=r_q;
  R.block(dof,dof,dof,dof)*=r_dq;

  if (!ekf.initialize(q,dq,Q,R))
  {
    ROS_ERROR("initialization fail");
    return 0;
  }


  Eigen::VectorXd amplitude(q.rows());
  amplitude.setRandom();
  amplitude*=5;
  double period=1;

  double t=0;

  Eigen::VectorXd c0=Eigen::VectorXd::Constant(dof,12.18);
  Eigen::VectorXd c1=Eigen::VectorXd::Constant(dof,35.85);

  unsigned int iter=0;
  while(ros::ok())
  {

    tau=chain->getJointTorque(q,dq,ddq)+c0.cwiseProduct(dq.cwiseSign())+c1.cwiseProduct(dq);

    q_noise   =   q+0.001*Eigen::MatrixXd::Random(dof,1);
    dq_noise  =  dq+0.01*Eigen::MatrixXd::Random(dof,1);
    ddq_noise = ddq+Eigen::MatrixXd::Random(dof,1);
    tau_noise = tau+5*Eigen::MatrixXd::Random(dof,1);

    ekf.update(q_noise,
               dq_noise,
               tau_noise,
               q_est,
               dq_est,
               ddq_est);



    for (unsigned int idof=0;idof<dof;idof++)
    {
      ori_msg.position.at(idof)   =   q(idof);
      ori_msg.velocity.at(idof)   =  dq(idof);
      ori_msg.effort.at(idof)     = tau(idof);
      ori_msg.effort.at(idof+dof) = ddq(idof);

      raw_msg.position.at(idof)   =   q_noise(idof);
      raw_msg.velocity.at(idof)   =  dq_noise(idof);
      raw_msg.effort.at(idof)     = tau_noise(idof);
      raw_msg.effort.at(idof+dof) = ddq_noise(idof);

      ekf_msg.position.at(idof)   =   q_est(idof);
      ekf_msg.velocity.at(idof)   =  dq_est(idof);
      ekf_msg.effort.at(idof)     = tau_noise(idof);
      ekf_msg.effort.at(idof+dof) = ddq_est(idof);
    }
    ekf_msg.header.stamp=ros::Time::now();
    raw_msg.header.stamp=ekf_msg.header.stamp;
    ori_msg.header.stamp=ekf_msg.header.stamp;

    ekf_pub.publish(ekf_msg);
    raw_pub.publish(raw_msg);
    ori_pub.publish(ori_msg);


    ddq=amplitude*std::cos(2*M_PI*t/period);
    q+=dq*sampling_period+0.5*std::pow(sampling_period,2)*ddq;
    dq+=ddq*sampling_period;
    t+=sampling_period;

    if (iter++>100)
    {
      iter=0;
      Q.setIdentity();
      R.setIdentity();

      double r_q,r_dq;
      if (!nh.getParam("ekf/r_q",r_q))
        r_q=0.001;
      if (!nh.getParam("ekf/r_dq",r_dq))
        r_dq=0.01;

      R.block(0,0,dof,dof)*=r_q;
      R.block(dof,dof,dof,dof)*=r_dq;

      ekf.setQR(Q,R);
      ROS_FATAL("r_q=%f,r_dq=%f",r_q,r_dq);
    }
  }
  return 0;
}
