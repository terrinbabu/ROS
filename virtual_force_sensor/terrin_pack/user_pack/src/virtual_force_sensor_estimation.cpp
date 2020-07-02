#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosdyn_core/urdf_parser.h>
#include <subscription_notifier/subscription_notifier.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_force_sensor_estimation");
  ros::NodeHandle nh;
  
  //// creating rosdyn chain ////
  
  std::string base_frame = "base_link";
  std::string tool_frame = "ee_link";    
  
  urdf::Model model;
  model.initParam("robot_description");
  ROS_INFO("URDF Model Name : %s",model.getName().c_str());
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
    
  //// Robot Joints description - Represented as a Joint State message ////
  
  sensor_msgs::JointState model_js;
  
  model_js.name.resize(6);
  model_js.position.resize(6);
  model_js.velocity.resize(6);
  model_js.name.at(0) = "shoulder_pan_joint";
  model_js.name.at(1) = "shoulder_lift_joint";
  model_js.name.at(2) = "elbow_joint";
  model_js.name.at(3) = "wrist_1_joint";
  model_js.name.at(4) = "wrist_2_joint";
  model_js.name.at(5) = "wrist_3_joint";
  
  chain->setInputJointsName(model_js.name);
  
  unsigned int dof=model_js.name.size();
  
  // Friction Component  
  
  std::vector<rosdyn::ComponentPtr> m_components;
  
  try
  {
    for (unsigned int idx = 0;idx < dof ;idx++)
    {
      std::string component_type;
      if (nh.getParam( model.getName()+"/"+model_js.name.at(idx)+"/spring/type", component_type))
      {
        if (!component_type.compare("Ideal"))
        {
          ROS_INFO("JOINT '%s' has a spring component", model_js.name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::IdealSpring(model_js.name.at(idx), model.getName(), nh )));
        }
      }
      
      if (nh.getParam( model.getName()+"/"+model_js.name.at(idx)+"/friction/type", component_type))
      {
        if (!component_type.compare("Polynomial1"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial1 component", model_js.name.at(idx).c_str());
          m_components.push_back( rosdyn::ComponentPtr(new rosdyn::FirstOrderPolynomialFriction( model_js.name.at(idx), model.getName(), nh ) ));
        } 
        else if (!component_type.compare("Polynomial2"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial2 component", model_js.name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::SecondOrderPolynomialFriction(model_js.name.at(idx), model.getName(), nh) ));
        }
      }
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Exception: %s",e.what());
  }
  
  //// Obtaining the Robot Joint position, velocity and effort ( converting the joint motor current values) ////
  
  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd tau_measured(6);
  tau_measured.setZero();
  
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_rec(nh,"/joint_states",1);
  
  ROS_INFO("waiting for topic");
  if (!js_rec.waitForANewData(ros::Duration(1000)))
    return 0;
  
  //// Initialization before while ros::ok loop
  
  double rate_hz = 125;
  double forgetting_factor=0.95;
  
  ros::Rate rate(rate_hz);
  
  Eigen::VectorXd Dq_last(6);  // previous velocity to calculate acceleration
  Dq_last.setZero();
  Eigen::VectorXd DDq(6); // acceleration
  DDq.setZero();
  Eigen::VectorXd tau_estimated(6);
  Eigen::VectorXd tau_inertia(6);
  Eigen::VectorXd tau_friction(6);
  Eigen::VectorXd tau_friction_temp(6);
  Eigen::VectorXd wrench_of_t_in_b(6);
  wrench_of_t_in_b.setZero();
  
  sensor_msgs::JointState inertia_js;
  sensor_msgs::JointState friction_js;
  sensor_msgs::JointState estimated_js;
  sensor_msgs::JointState extra_js;
  ros::Publisher inertia_torque_pub=nh.advertise<sensor_msgs::JointState>("/ur10/inertial_joint_torques",1);
  ros::Publisher friction_torque_pub=nh.advertise<sensor_msgs::JointState>("/ur10/frictional_joint_torques",1);
  ros::Publisher estimated_torque_pub=nh.advertise<sensor_msgs::JointState>("/ur10/estimated_joint_torques",1);
  ros::Publisher extra_torque_pub=nh.advertise<sensor_msgs::JointState>("/ur10/residual_joint_torques",1);
  ros::Publisher wrench_pub=nh.advertise<geometry_msgs::WrenchStamped>("/ur10/virtual_wrench",1);
  
  if(js_rec.getData().effort.size() == 12)
    ROS_INFO("Start - virtual force estimation using kalman filtered joint state");
  else
    ROS_INFO("Start - virtual force estimation using joint state ( without filtering )");
  
  while (ros::ok())
  {
    // Read the joint states 
    
    for (unsigned int idx=0;idx<dof;idx++)
    {
      q(idx)=js_rec.getData().position.at(idx);
      Dq(idx)=js_rec.getData().velocity.at(idx);
      tau_measured(idx)=js_rec.getData().effort.at(idx);
    }
    
    estimated_js=js_rec.getData();
    inertia_js=estimated_js;
    inertia_js.effort.resize(12);
    friction_js=estimated_js;      
    extra_js=estimated_js;
    
    // Accelearation estimation
    
    if(
       js_rec.getData().effort.size()        == 12/* &&
       abs(js_rec.getData().effort[6])       <  20 &&
       abs(js_rec.getData().effort[7])       <  20 &&
       abs(js_rec.getData().effort[8])       <  20*/
      ) 
    {     
        for (unsigned int idx=0;idx<dof;idx++)
            DDq(idx) = js_rec.getData().effort.at(idx+dof); // Accelearation from kalman filter
    }
    else
        DDq=(Dq-Dq_last)*rate_hz; // joint states published at 125Hz
        
    Dq_last=Dq;
    
    // Estimated Joint Torque
    
    tau_inertia = chain->getJointTorque(q, Dq, DDq);
    
    for ( size_t iComponent = 0; iComponent<m_components.size(); iComponent++ )
    {
      tau_friction_temp = m_components.at(iComponent)->getTorque(q,Dq,DDq);
      tau_friction(iComponent) = tau_friction_temp(iComponent);
    }
    
    tau_estimated = tau_inertia + tau_friction;
    
    for (unsigned int idx=0;idx<dof;idx++)
    {
      inertia_js.effort.at(idx)=tau_inertia(idx);
      inertia_js.effort.at(idx+6) = DDq(idx);
      friction_js.effort.at(idx)=tau_friction(idx);       
      estimated_js.effort.at(idx)=tau_estimated(idx);
    }
    
    inertia_torque_pub.publish(inertia_js);
    friction_torque_pub.publish(friction_js);
    estimated_torque_pub.publish(estimated_js);
    
    // Extra Residual Joint torque 
    
    Eigen::VectorXd extra_tau=tau_measured-tau_estimated;
    
    for (unsigned int idx=0;idx<dof;idx++)
      extra_js.effort.at(idx)=-extra_tau(idx);
    
    extra_torque_pub.publish(extra_js);
    
    // Virtual Wrench
    
    Eigen::MatrixXd J = chain->getJacobian(q);
    
    Eigen::JacobiSVD<Eigen::MatrixXd> pinv_J(J.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    int rank = 0;
    while ( std::abs(pinv_J.singularValues()(rank))>(1e-2*std::abs(pinv_J.singularValues()(0))) )
    {
      rank++;
      if (rank == (pinv_J.matrixV().rows()))
        break;
    }
    
    if ((pinv_J.matrixV().cols()-rank)>0)   // dealing with singularity
    {
      Eigen::MatrixXd null_space;
      Eigen::VectorXd external_wrench_vect = -pinv_J.solve(extra_tau);
      null_space = pinv_J.matrixV().block(0, rank, pinv_J.matrixV().rows(),  pinv_J.matrixV().cols()-rank);
      Eigen::JacobiSVD<Eigen::MatrixXd> svd_null(null_space, Eigen::ComputeThinU | Eigen::ComputeThinV);
      wrench_of_t_in_b=external_wrench_vect+null_space*svd_null.solve(forgetting_factor*wrench_of_t_in_b-external_wrench_vect );
    }
    else
      wrench_of_t_in_b=-pinv_J.solve(extra_tau);
    
    Eigen::Affine3d T_bt = chain->getTransformation(q);
    
    Eigen::VectorXd wrench_of_t_in_t = rosdyn::spatialRotation(wrench_of_t_in_b,T_bt.linear().inverse());
    
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp=estimated_js.header.stamp;
    wrench_msg.header.frame_id=tool_frame;
    
    wrench_msg.wrench.force.x=wrench_of_t_in_t(0);
    wrench_msg.wrench.force.y=wrench_of_t_in_t(1);
    wrench_msg.wrench.force.z=wrench_of_t_in_t(2);
    wrench_msg.wrench.torque.x=wrench_of_t_in_t(3);
    wrench_msg.wrench.torque.y=wrench_of_t_in_t(4);
    wrench_msg.wrench.torque.z=wrench_of_t_in_t(5);
    
    wrench_pub.publish(wrench_msg);
    
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
    
