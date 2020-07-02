#ifndef HUBER_ROTATION_INSTANCE_H
#define HUBER_ROTATION_INSTANCE_H

#include "PolicyInstanceBase.h"
#include "UtilFunctions.h"
#include <iostream>
#include <math.h>

namespace assistance_policy {

const int STATE_DIMENSION_HUBERROTATION = 3;
const int ACTION_DIMENSION_HUBERROTATION = 6;


const double ROBOT_TRANSLATION_COST_MULTIPLIER = 15.0;  //15
const double ROBOT_ROTATION_COST_MULTIPLIER = 0.15;  //0.15

//maybe just store pose?
struct StateVector_HuberRotation
{
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_;

  StateVector_HuberRotation() { }

  StateVector_HuberRotation(const Eigen::Affine3d& pose)
    : translation_(pose.translation()), rotation_(pose.linear()) { rotation_.normalize();}
  
  StateVector_HuberRotation(const StateVector_HuberRotation& state)
    : translation_(state.translation_), rotation_(state.rotation_) { rotation_.normalize();}
};

typedef Eigen::Matrix< double, ACTION_DIMENSION_HUBERROTATION, 1 > ActionVector_HuberRotation; //top 3 translation, bottom 3 rotation


//inline StateVector_Huber Huber_PoseToState(const Eigen::Affine3d& pose) { return pose.translation();}

class HuberRotationInstance : public virtual PolicyInstance<StateVector_HuberRotation, ActionVector_HuberRotation>
{
  //static const double ROTATION_VELOCITY_COMMAND_TO_STATE_DIFF = 0.025;

  public:
    static const int STATE_DIMENSION = STATE_DIMENSION_HUBERROTATION;
    static const int ACTION_DIMENSION = ACTION_DIMENSION_HUBERROTATION;

    HuberRotationInstance(const StateVector& goal_state) : PolicyInstance(goal_state) { };

    //double Value_Translation(const StateVector& state);
    //double Value_Rotation(const StateVector& state);
//    inline double Value_Prediction(const StateVector& state)
//    {
//      return Value_Translation(state) + ROTATION_COST_MULTIPLIER*Value_Rotation(state);
//    }
//    inline double QValue_Prediction(const StateVector& state, const ActionVector& action)

    double Value_Translation(double dist_translation);
    double Value_Rotation(double dist_rotation);

    double Value_Prediction(const StateVector& state)
    {
      double translation_distance = StateDistance_Translation(state, goal_state_);
      double rotation_distance = StateDistance_Rotation(state, goal_state_);
      return Value_Prediction_FromDists(translation_distance, rotation_distance);
    }

    inline double Value_Prediction_FromDists(double translation_distance, double rotation_distance)
    {
      return Value_Translation(translation_distance) + ROTATION_COST_MULTIPLIER*Value_Rotation(rotation_distance);
    }

    double QValue_Prediction(const StateVector& state, const ActionVector& action)
    {
      double translation_distance = StateDistance_Translation(state, goal_state_);
      double rotation_distance = StateDistance_Rotation(state, goal_state_);

      StateVector transitioned_state = TransitionState(state, action);
      double translation_distance_aftertransition = StateDistance_Translation(transitioned_state, goal_state_);
      double rotation_distance_aftertransition = StateDistance_Rotation(transitioned_state, goal_state_);

      return QValue_Prediction_FromDists(translation_distance, rotation_distance, translation_distance_aftertransition, rotation_distance_aftertransition);
    }

    double QValue_Prediction_FromDists(double translation_distance, double rotation_distance, double translation_distance_aftertransition, double rotation_distance_aftertransition)
    {
      double value_transition_state = Value_Prediction_FromDists(translation_distance_aftertransition, rotation_distance_aftertransition);
      double cost_curr_state = CostOfState_FromDists(translation_distance, rotation_distance);
      return value_transition_state + cost_curr_state;
    }

//    inline double QValue_Arbitration()
//    {
//      return QValue_Prediction();
//    }

    void CacheDistanceInfo(const StateVector& state, const ActionVector& action, const AssistanceGoal<StateVector>& goal);
    ActionVector QDerivative_Arbitration(const StateVector& state, const ActionVector& action);

    double CostOfState(const StateVector& state)
    {
      double translation_distance = StateDistance_Translation(state, goal_state_);
      double rotation_distance = StateDistance_Rotation(state, goal_state_);
      return CostOfState_FromDists(translation_distance, rotation_distance);
    }

    double CostOfState_FromDists(double translation_distance, double rotation_distance)
    {
      double translation_cost = CostOfState_Translation(translation_distance);
      double rotation_cost = CostOfState_Rotation(rotation_distance);
      return translation_cost + ROTATION_COST_MULTIPLIER*rotation_cost;
    }

    double CostOfState_Translation(double dist_translation);
    double CostOfState_Rotation(double dist_rotation);


    inline static double StateDistance_Translation(const StateVector& state1, const StateVector& state2)
    {return (state1.translation_-state2.translation_).norm(); }

    inline static double StateDistance_Rotation(const StateVector& state1, const StateVector& state2)
    { return QuaternionDistance(state1.rotation_, state2.rotation_); }





    //inline static double QuaternionDistance(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2)
    //{ return std::min( (quat1.coeffs()-quat2.coeffs()).norm(), (quat1.coeffs()+quat2.coeffs()).norm()); }

    inline static Eigen::Vector3d GetTranslationPart(const ActionVector& action) { return action.head<3>();}
    inline static Eigen::Vector3d GetRotationPart(const ActionVector& action) { return action.tail<3>();}
    inline static void SetTranslationPart(ActionVector& action, const Eigen::Vector3d& translation) { action.head<3>() = translation;}
    inline static void SetRotationPart(ActionVector& action, const Eigen::Vector3d& rotation) { action.tail<3>() = rotation;}


    inline static StateVector PoseToState(const Eigen::Affine3d& pose) { return StateVector(pose); }
    inline static Eigen::Affine3d StateToPose(const StateVector& state) {
      Eigen::Affine3d pose(state.rotation_);
      pose.linear() = state.rotation_.toRotationMatrix();
      pose.translation() = state.translation_;
      return pose;
    }

    static ActionVector ControlToAction(const Eigen::Vector3d& control);

    static StateVector TransitionState_Translation(const StateVector& curr_state, const Eigen::Vector3d& translation_velocity);
    static StateVector TransitionState_Rotation(const StateVector& curr_state, const Eigen::Vector3d& angular_velocity);

    inline static StateVector TransitionState(const StateVector& curr_state, const ActionVector& action) {
      StateVector transitioned_state = TransitionState_Translation(curr_state, GetTranslationPart(action));
      return TransitionState_Rotation(transitioned_state, GetRotationPart(action));
    }

    static inline double CostOfAction_Robot(const ActionVector& action) 
    { 
      return 0.5*(GetTranslationPart(action).squaredNorm()*ROBOT_TRANSLATION_COST_MULTIPLIER + GetRotationPart(action).squaredNorm()*ROBOT_ROTATION_COST_MULTIPLIER);
    }


    //turns the action into a pose offset
    static Eigen::Affine3d GetServoPoseFromAction(const Eigen::Affine3d& curr_pose, const ActionVector& action_to_apply);
    static Eigen::Affine3d ApplyActionToPose(const Eigen::Affine3d& pose, const ActionVector& action_to_apply);
    
    static void SetHuberConstants(double human_cost_multiplier, double delta_switch_modes, double constant_cost_add);


    //parameters
    static double TRANSLATION_HUMAN_ACTION_COST_MULTIPLIER;
    static double TRANSLATION_DELTA_SWITCH_MODES;
    static double TRANSLATION_CONSTANT_COST_ADD;

    static double ROTATION_COST_MULTIPLIER;
    static double ROTATION_HUMAN_ACTION_COST_MULTIPLIER;
    static double ROTATION_DELTA_SWITCH_MODES;
    static double ROTATION_CONSTANT_COST_ADD;

    //calculated for efficiency
    static double TRANSLATION_LINEAR_COST_MULTIPLIER_TOTAL;
    static double TRANSLATION_QUADRATIC_COST_MULTPLIER;
    static double TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF;
    static double TRANSLATION_LINEAR_COST_SUBTRACT;

    static double ROTATION_LINEAR_COST_MULTIPLIER_TOTAL;
    static double ROTATION_QUADRATIC_COST_MULTPLIER;
    static double ROTATION_QUADRATIC_COST_MULTPLIER_HALF;
    static double ROTATION_LINEAR_COST_SUBTRACT;


  protected:



};



} //namespace


#endif
