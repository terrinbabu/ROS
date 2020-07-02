
#include "HuberRotationInstance.h"



namespace assistance_policy {

double HuberRotationInstance::TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER = 0.04;
double HuberRotationInstance::TRANSLATION_DELTA_SWITCH_MODES = 0.075;  //point at which goes from quadratic to linear
double HuberRotationInstance::TRANSLATION_CONSTANT_COST_ADD = 4.0;

double HuberRotationInstance::ROTATION_COST_MULTIPLIER = 0.02;
double HuberRotationInstance::ROTATION_HUMAN_ACTION_COST_MULTIPLIER = 0.01;
double HuberRotationInstance::ROTATION_DELTA_SWITCH_MODES = M_PI/72.;  //point at which goes from quadratic to linear
double HuberRotationInstance::ROTATION_CONSTANT_COST_ADD = 4.;



double HuberRotationInstance::TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER + TRANSLATION_CONSTANT_COST_ADD;
double HuberRotationInstance::TRANSLATION_QUADRATIC_COST_MULTPLIER = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER/TRANSLATION_DELTA_SWITCH_MODES; //multiplier comes out to this for quadratic part, calculated to make derivative smooth at mode switch
double HuberRotationInstance::TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * TRANSLATION_QUADRATIC_COST_MULTPLIER;
double HuberRotationInstance::TRANSLATION_LINEAR_COST_SUBTRACT = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER * TRANSLATION_DELTA_SWITCH_MODES * 0.5; //this is what we subtract off from linear part, calculated to make it connect to quadratic part

double HuberRotationInstance::ROTATION_LINEAR_COST_MULTIPLIER_TOTAL = ROTATION_HUMAN_ACTION_COST_MULTIPLIER + ROTATION_CONSTANT_COST_ADD;
double HuberRotationInstance::ROTATION_QUADRATIC_COST_MULTPLIER = ROTATION_HUMAN_ACTION_COST_MULTIPLIER/ROTATION_DELTA_SWITCH_MODES; //multiplier comes out to this for quadratic part, calculated to make derivative smooth at mode switch
double HuberRotationInstance::ROTATION_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * ROTATION_QUADRATIC_COST_MULTPLIER;
double HuberRotationInstance::ROTATION_LINEAR_COST_SUBTRACT = ROTATION_HUMAN_ACTION_COST_MULTIPLIER * ROTATION_DELTA_SWITCH_MODES * 0.5; //this is what we subtract off from linear part, calculated to make it connect to quadratic part



double HuberRotationInstance::Value_Translation(double dist_translation)
{
  if (dist_translation <= TRANSLATION_DELTA_SWITCH_MODES)
  {
    return TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF * dist_translation*dist_translation + TRANSLATION_CONSTANT_COST_ADD*dist_translation;
  } //else {
  return TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL * dist_translation - TRANSLATION_LINEAR_COST_SUBTRACT;
}

double HuberRotationInstance::Value_Rotation(double dist_rotation)
{
  if (dist_rotation <= ROTATION_DELTA_SWITCH_MODES)
  {
    return ROTATION_QUADRATIC_COST_MULTPLIER_HALF * dist_rotation*dist_rotation + ROTATION_CONSTANT_COST_ADD*dist_rotation;
  } //else {
  return ROTATION_LINEAR_COST_MULTIPLIER_TOTAL * dist_rotation - ROTATION_LINEAR_COST_SUBTRACT;
}

double HuberRotationInstance::CostOfState_Translation(double dist_translation)
{ 
  if (dist_translation > TRANSLATION_DELTA_SWITCH_MODES)
  {
    return TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL;
  } //else {
  return TRANSLATION_QUADRATIC_COST_MULTPLIER * dist_translation + TRANSLATION_CONSTANT_COST_ADD;
}

double HuberRotationInstance::CostOfState_Rotation(double dist_rotation)
{
  if (dist_rotation > ROTATION_DELTA_SWITCH_MODES)
  {
    return ROTATION_LINEAR_COST_MULTIPLIER_TOTAL;
  }// else {
  return ROTATION_QUADRATIC_COST_MULTPLIER * dist_rotation + ROTATION_CONSTANT_COST_ADD;
}

HuberRotationInstance::ActionVector HuberRotationInstance::QDerivative_Arbitration(const StateVector& state, const ActionVector& action)
{
  StateVector transitioned_state = TransitionState(state, action);
  Eigen::Vector3d translation_derivative;
  Eigen::Vector3d translation_diff = transitioned_state.translation_ - goal_state_.translation_;
  double dist_translation = translation_diff.norm();
  //translation_derivative = TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL*(translation_diff/dist_translation); //0.16m along direction
  if (dist_translation > TRANSLATION_DELTA_SWITCH_MODES)
  {
    translation_derivative = TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL*(translation_diff/dist_translation); //0.16m along direction
  } else {
    translation_derivative = TRANSLATION_CONSTANT_COST_ADD*(translation_diff/dist_translation);  //4m along direction
    translation_derivative += TRANSLATION_QUADRATIC_COST_MULTPLIER*translation_diff;  //0.05m
  }
  translation_derivative /= ROBOT_TRANSLATION_COST_MULTIPLIER; //0.01m e 0.2m in direzione


  Eigen::Quaterniond quat_between = (goal_state_.rotation_)*(transitioned_state.rotation_.inverse());
  //std::cout << "This quaternion consists of a scalar " << quat_between.w() << " and a vector " << std::endl << quat_between.vec() << std::endl;
  
  double dist_rotation = StateDistance_Rotation(transitioned_state, goal_state_);
  Eigen::Vector3d rotation_derivative = quat_between.vec()/(quat_between.vec().norm());
  
  double rotation_deriv_magnitude;
  //rotation_deriv_magnitude = ROTATION_LINEAR_COST_MULTIPLIER_TOTAL;
  if (dist_rotation > ROTATION_DELTA_SWITCH_MODES)
  {
    rotation_deriv_magnitude = ROTATION_LINEAR_COST_MULTIPLIER_TOTAL;
  } else {
    rotation_deriv_magnitude = ROTATION_CONSTANT_COST_ADD;
    rotation_deriv_magnitude += ROTATION_QUADRATIC_COST_MULTPLIER*dist_rotation;
  }
  rotation_derivative *= ROTATION_COST_MULTIPLIER*rotation_deriv_magnitude/ROBOT_ROTATION_COST_MULTIPLIER;  //0.133rad = 7gradi

  //flip if necessary
  if (QuaternionDot(goal_state_.rotation_, transitioned_state.rotation_) > 0)
  {
    rotation_derivative *= -1;
  }
  
  //std::cout << "dist rotation: " << dist_rotation << std::endl;

  //hacky part to make it not jumpy
  const double translation_limit = 2e-2;
  const double rotation_limit = M_PI/12;
  if (dist_translation < translation_limit) {
    translation_derivative *= dist_translation/translation_limit;
  }
  if (dist_rotation < rotation_limit) {
    rotation_derivative *= dist_rotation/rotation_limit;
  }

  ActionVector deriv;
  SetTranslationPart(deriv, translation_derivative);
  SetRotationPart(deriv, rotation_derivative);
  
/*  
  for (int i=0; i<deriv.rows(); i++)
      std::cout << std::isnan(deriv(i,0)) << std::endl;
      if (std::isnan(deriv(i,0)))
          deriv(i,0) = 0.0;   */       
  
  //std::cout << "deriv: " << deriv << std::endl;
  return deriv;
}

void HuberRotationInstance::SetHuberConstants(double human_cost_multiplier, double delta_switch_modes, double constant_cost_add )
{
  TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER = human_cost_multiplier;
  TRANSLATION_DELTA_SWITCH_MODES = delta_switch_modes;
  TRANSLATION_CONSTANT_COST_ADD = constant_cost_add;

  TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER + TRANSLATION_CONSTANT_COST_ADD;
  TRANSLATION_QUADRATIC_COST_MULTPLIER = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER/TRANSLATION_DELTA_SWITCH_MODES;
  TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * TRANSLATION_QUADRATIC_COST_MULTPLIER;
  TRANSLATION_LINEAR_COST_SUBTRACT = TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER * TRANSLATION_DELTA_SWITCH_MODES * 0.5;
}


HuberRotationInstance::ActionVector HuberRotationInstance::ControlToAction(const Eigen::Vector3d& control)
{
  ActionVector action;
  SetTranslationPart(action, control); 
  SetRotationPart(action, Eigen::Vector3d::Zero()); 
  return action;
}

HuberRotationInstance::StateVector HuberRotationInstance::TransitionState_Translation(const StateVector& curr_state, const Eigen::Vector3d& translation_velocity) 
{
  StateVector transitioned_state(curr_state);
  transitioned_state.translation_ += translation_velocity*LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF;
  return transitioned_state;
}

HuberRotationInstance::StateVector HuberRotationInstance::TransitionState_Rotation(const StateVector& curr_state, const Eigen::Vector3d& angular_velocity)
{
  if (angular_velocity.norm() < 1e-12)
  {
    return curr_state;
  }
  StateVector transitioned_state(curr_state);
  //transitioned_state.rotation_ = curr_state.rotation_* Eigen::AngleAxisd(LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF*angular_velocity.norm(), angular_velocity.normalized());
  transitioned_state.rotation_ = Eigen::AngleAxisd(LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF*angular_velocity.norm(), angular_velocity.normalized()) * curr_state.rotation_;
  return transitioned_state;

}


Eigen::Affine3d HuberRotationInstance::GetServoPoseFromAction(const Eigen::Affine3d& curr_pose, const ActionVector& action_to_apply)
{ 
  ActionVector action_for_servo_pose = action_to_apply / LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF;
  return ApplyActionToPose(curr_pose, action_for_servo_pose);
}

Eigen::Affine3d HuberRotationInstance::ApplyActionToPose(const Eigen::Affine3d& curr_pose, const ActionVector& action_to_apply)
{ 
  StateVector state(curr_pose);
  state = TransitionState(state, action_to_apply);
  return StateToPose(state);
}

void HuberRotationInstance::CacheDistanceInfo(const StateVector& state, const ActionVector& action, const AssistanceGoal<StateVector>& goal)
{
  //cache state and action
  curr_state_cached_ = state;
  curr_action_cached_ = action;

  //calculate necessary distances
  double translation_distance = StateDistance_Translation(state, goal_state_);
  double rotation_distance = StateDistance_Rotation(state, goal_state_);

  StateVector transitioned_state = TransitionState(state, action);
  double translation_distance_aftertransition = StateDistance_Translation(transitioned_state, goal_state_);
  double rotation_distance_aftertransition = StateDistance_Rotation(transitioned_state, goal_state_);

  value_prediction_cached_ = Value_Prediction_FromDists(translation_distance, rotation_distance);
  value_arbitration_cached_ = value_prediction_cached_;
  qvalue_prediction_cached_ = QValue_Prediction_FromDists(translation_distance, rotation_distance, translation_distance_aftertransition, rotation_distance_aftertransition);
  qvalue_arbitration_cached_ = qvalue_prediction_cached_;

  //derivative_cached_ = QDerivative_Arbitration(state, action);

  //ARMS_DEBUG("center: " << target.obj_ray_intersection_.center_.transpose());
  //ARMS_DEBUG("num other targets: " <<other_targets.size());
}



}


