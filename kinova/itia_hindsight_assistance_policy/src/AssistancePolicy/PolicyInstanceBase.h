#ifndef POLICY_INSTANCE_BASE_H
#define POLICY_INSTANCE_BASE_H

#include <eigen3/Eigen/Geometry>
#include "AssistanceGoal.h"

namespace assistance_policy {

template <typename StateVector_Spec, typename ActionVector_Spec>
class PolicyInstance
{
  public:
    typedef StateVector_Spec StateVector;
    typedef ActionVector_Spec ActionVector;

    PolicyInstance(const StateVector& goal_state)
      : goal_state_(goal_state) { }
    
    //PolicyInstance(const Eigen::Affine3d& goal_pose) { goal_state_ = PoseToStateVector(goal_pose); }

    inline StateVector StateInInstance(const StateVector& state)
    {
      return state - goal_state_;
    }

    const StateVector& GoalState(){return goal_state_;} 

    //can override
    virtual double Value_Arbitration(const StateVector& state) {return Value_Prediction(state);}
    virtual double QValue_Arbitration(const StateVector& state, const ActionVector& action) {return QValue_Prediction(state, action);}
    virtual double CostOfState(const StateVector& state) { return 0;}

    //must define for specific instance
    virtual double Value_Prediction(const StateVector& state) = 0;
    virtual double QValue_Prediction(const StateVector& state, const ActionVector& action) = 0;
    virtual ActionVector QDerivative_Arbitration(const StateVector& state, const ActionVector& action) = 0;

    //Given the current state and action, cache everything
    virtual void CacheDistanceInfo(const StateVector& state, const ActionVector& action, const AssistanceGoal<StateVector>& goal)
    {
      curr_state_cached_ = state;
      curr_action_cached_ = action;

      value_prediction_cached_ = Value_Prediction(state);
      value_arbitration_cached_ = Value_Arbitration(state);
      qvalue_prediction_cached_ = QValue_Prediction(state, action);
      qvalue_arbitration_cached_ = QValue_Arbitration(state, action);
      //qderivative_cached_ = QDerivative_Arbitration(state, action);
    }

    //These don't have to be the same as what was passed in - so don't return for now
    //inline StateVector GetCachedState() { return curr_state_cached_; }
    //inline ActionVector GetCachedAction() { return curr_action_cached_; }

    inline double CostOfState_Cached() const {return cost_state_cached_;}
    inline double Value_Arbitration_Cached() const {return value_arbitration_cached_;}
    inline double QValue_Arbitration_Cached() const {return qvalue_arbitration_cached_;}
    inline double Value_Prediction_Cached() const {return value_prediction_cached_;} 
    inline double QValue_Prediction_Cached() const {return qvalue_prediction_cached_;}
    //inline ActionVector QDerivative_Arbitration_Cached() const {return qderivative_cached_;}
    inline ActionVector QDerivative_Arbitration_Cached(){return QDerivative_Arbitration(curr_state_cached_, curr_action_cached_);}



//  ALSO NEED THESE
//    static Action ControlToAction(const Eigen::Vector3d& control) {return control;}
//    static StateVector PoseToState(const Eigen::Affine3d& pose) { return pose.translation();}
//    static StateVector TransitionState(const StateVector& curr_state, const ActionVector& curr_control) { return curr_state + curr_control;}
//    static inline double CostOfAction_Human(const ActionVector& action) { return HUBER_HUMAN_ACTION_COST_MULTIPLIER;}
//    static inline double CostOfAction_Robot(const ActionVector& action) { return action.squaredNorm(); }
//


  protected:
    StateVector goal_state_;

    //cached values since we the same state and action each update
    StateVector curr_state_cached_;
    ActionVector curr_action_cached_;
    double cost_state_cached_;
    double value_prediction_cached_;
    double value_arbitration_cached_;
    double qvalue_prediction_cached_;
    double qvalue_arbitration_cached_;
    //ActionVector qderivative_cached_;


};

} //namespace
#endif
