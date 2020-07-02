#ifndef HUBER_INSTANCE_H
#define HUBER_INSTANCE_H

#include "PolicyInstanceBase.h"

namespace assistance_policy {

const int STATE_DIMENSION_HUBER = 3;
const int ACTION_DIMENSION_HUBER = 3;
typedef Eigen::Matrix< double, STATE_DIMENSION_HUBER, 1 > StateVector_Huber;
typedef Eigen::Matrix< double, ACTION_DIMENSION_HUBER, 1 > ActionVector_Huber;


const double HUBER_ROBOT_COST_MULTIPLIER = 2.5;

//inline StateVector_Huber Huber_PoseToState(const Eigen::Affine3d& pose) { return pose.translation();}

class HuberInstance : public PolicyInstance<StateVector_Huber, ActionVector_Huber>
{
  public:
    static const int STATE_DIMENSION = STATE_DIMENSION_HUBER;
    static const int ACTION_DIMENSION = ACTION_DIMENSION_HUBER;

    //typedef StateVector_Huber StateVector;
    //typedef ActionVector_Huber ActionVector;

   
    HuberInstance(const StateVector& goal_state) : PolicyInstance(goal_state) { };


    //////HELPER FUNCTION ADDED JUST FOR RSS EXPERIMENTS
    inline double StateDistance_Translation(const StateVector& state1, const StateVector& state2)
    {
      return (state1 - state2).norm();
    }


    inline double Value_Prediction_Cached()
    {
      return Value_Prediction(curr_state_cached_);
    }

    inline double QValue_Prediction_Cached()
    {
      return QValue_Prediction(curr_state_cached_, curr_action_cached_);
    }

    inline ActionVector QDerivative_Arbitration_Cached()
    {
      return QDerivative_Arbitration(curr_state_cached_, curr_action_cached_);
    }


    inline double Value_Prediction(const StateVector& state)
    {
      StateVector state_in_instance = StateInInstance(state);
      double dist = state_in_instance.norm();
      if (dist <= HUBER_DELTA_SWITCH_MODES)
      {
        return HUBER_QUADRATIC_COST_MULTPLIER_HALF * dist*dist + HUBER_CONSTANT_COST_ADD*dist;
      } //else
      return HUBER_LINEAR_COST_MULTIPLIER_TOTAL * dist - HUBER_LINEAR_COST_SUBTRACT;
    }

    inline double QValue_Prediction(const StateVector& state, const ActionVector& action)
    {
      return Value_Prediction(TransitionState(state, action)) + CostOfState(state);
    }

//    inline double QValue_Arbitration(const StateVector& state, const ActionVector& action)
//    {
//      StateVector state_in_instance = StateInInstance(state);
//      return ValueAtState(TransitionState(state_in_instance, action)) + CostOfAction_Robot(action);
//    }

    ActionVector QDerivative_Arbitration(const StateVector& state, const ActionVector& human_action)
    {
      StateVector state_in_instance = StateInInstance(state); 
      StateVector transitioned_state = TransitionState(state_in_instance, human_action);
      if (transitioned_state.norm() > HUBER_DELTA_SWITCH_MODES)
      {
        return (transitioned_state/(transitioned_state.norm()) )/HUBER_ROBOT_COST_MULTIPLIER;
      }

      return transitioned_state/HUBER_ROBOT_COST_MULTIPLIER;
    }

    inline double CostOfState(const StateVector& state)
    { 
      StateVector state_in_instance = StateInInstance(state); 
      double state_norm = state_in_instance.norm();
      if (state_norm > HUBER_DELTA_SWITCH_MODES)
      {
        return HUBER_HUMAN_ACTION_COST_MULTIPLIER + HUBER_CONSTANT_COST_ADD;
      } else {
        return HUBER_QUADRATIC_COST_MULTPLIER * state_norm + HUBER_CONSTANT_COST_ADD;
      }
    
    }

    //static inline StateVector TransitionState(const StateVector& curr_state, const ActionVector& curr_control) { return curr_state + curr_control;}
    //static inline StateVector PoseToState(const Eigen::Affine3d& pose) { return pose.translation();}




    //static funcs
    static ActionVector ControlToAction(const Eigen::Vector3d& control) {return control;}
    static StateVector PoseToState(const Eigen::Affine3d& pose) { return pose.translation();}
    static StateVector TransitionState(const StateVector& curr_state, const ActionVector& curr_control) { return curr_state + curr_control*LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF;}
    static inline double CostOfAction_Robot(const ActionVector& action) { return action.squaredNorm() * (HUBER_ROBOT_COST_MULTIPLIER/2.); }
    static Eigen::Affine3d StateToPose(const StateVector& state) {
      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
      pose.translation() = state;
      return pose;
    }


    static Eigen::Affine3d GetServoPoseFromAction(const Eigen::Affine3d& curr_pose, const ActionVector& action_to_apply)
    {
      ActionVector action_for_servo_pose = action_to_apply/LINEAR_VELOCITY_COMMAND_TO_STATE_DIFF;
      return ApplyActionToPose(curr_pose, action_for_servo_pose);
    }

    static Eigen::Affine3d ApplyActionToPose(const Eigen::Affine3d& curr_pose, const ActionVector& action_to_apply)
    {
      StateVector ServoState = TransitionState(PoseToState(curr_pose), action_to_apply);
      Eigen::Affine3d servo_pose = StateToPose(ServoState);
      servo_pose.linear() = curr_pose.linear();
      return servo_pose;
    }

    static void SetHuberConstants(double human_cost_multiplier, double delta_switch_modes, double constant_cost_add )
    {
      HUBER_HUMAN_ACTION_COST_MULTIPLIER = human_cost_multiplier;
      HUBER_DELTA_SWITCH_MODES = delta_switch_modes;
      HUBER_CONSTANT_COST_ADD = constant_cost_add;

      HUBER_LINEAR_COST_MULTIPLIER_TOTAL = HUBER_HUMAN_ACTION_COST_MULTIPLIER + HUBER_CONSTANT_COST_ADD;
      HUBER_QUADRATIC_COST_MULTPLIER = HUBER_HUMAN_ACTION_COST_MULTIPLIER/HUBER_DELTA_SWITCH_MODES;
      HUBER_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * HUBER_QUADRATIC_COST_MULTPLIER;
      HUBER_LINEAR_COST_SUBTRACT = HUBER_HUMAN_ACTION_COST_MULTIPLIER * HUBER_DELTA_SWITCH_MODES * 0.5;
    }

    //parameters
    static double HUBER_HUMAN_ACTION_COST_MULTIPLIER;
    static double HUBER_DELTA_SWITCH_MODES;
    static double HUBER_CONSTANT_COST_ADD;
    
    //calculated for efficiency
    static double HUBER_LINEAR_COST_MULTIPLIER_TOTAL;
    static double HUBER_QUADRATIC_COST_MULTPLIER;
    static double HUBER_QUADRATIC_COST_MULTPLIER_HALF;
    static double HUBER_LINEAR_COST_SUBTRACT;

};


} //namespace


#endif
