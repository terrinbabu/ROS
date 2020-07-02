#include "AssistancePolicyBase.h"

    
namespace assistance_policy {

template <typename Instance>
  AssistancePolicy<Instance>::~AssistancePolicy()
  { }

template <typename Instance>
void AssistancePolicy<Instance>::GetCostAtPoses_OneGoal(const std::vector<Eigen::Affine3d>& poses, int goal, std::vector<double>& vals)
{
  vals.resize(poses.size());
  for (size_t i=0; i < poses.size(); i++)
  {
    vals[i] = GetCostAtPose_OneGoal(poses[i], goal);
  }
}


template <typename Instance>
void AssistancePolicy<Instance>::Initialize(const std::vector<std::vector<Eigen::Affine3d> >& target_poses, const std::vector<Eigen::Affine3d>& goal_poses, const std::vector<double>& goal_dist)
{
  assert(goal_poses.size() == goal_dist.size());

//STE: deleted from original code
//   goal_predictor_.reset(new GoalPredictor<Instance>());
//   goal_predictor_->Initialize(goal_dist);
//   
  
  //create instances
  for (size_t i=0; i < target_poses.size(); i++)
  {
    goal_policies_.push_back(PosesToGoal(target_poses[i], goal_poses[i]));
  }
 
//STE: deleted from original code
//  goal_predictor_->SetGoalPolicies(goal_policies_);
}


template <typename Instance>
AssistanceGoal<typename Instance::StateVector> AssistancePolicy<Instance>::PosesToGoal(const std::vector<Eigen::Affine3d>& target_poses, const Eigen::Affine3d& object_pose)
{
  std::vector<StateVector> target_states;

  //TODO
  // maybe check for IK solutions? Or do we assume all poses have already been checked?
  for (size_t i=0; i < target_poses.size(); i++)
  {
    target_states.push_back(Instance::PoseToState(target_poses[i]));   //vettore degli stati
  }

  AssistanceGoal<StateVector> assist_targ(target_states, object_pose);  //inizializza in assistancegoal targets and goals
  return assist_targ;
}

template <typename Instance>
typename Instance::ActionVector AssistancePolicy<Instance>::GetAssistedAction_Cached(std::vector<double> goal_dist)
{
//STE: deleted from original - since curr_control_cached_is a null vector the following line can be decommetned
  //ActionVector curr_action = Instance::ControlToAction(curr_control_cached_);

  ActionVector action_derivative;
  action_derivative.setZero();
  double prob_normalizer = 0;
  for (size_t i=0; i < goal_policies_.size(); i++)
  {
//STE: deleted from original 
    //action_derivative -= goal_predictor_->ProbAtInd(i) * goal_policies_[i].QDerivative_Arbitration_Cached();
    //prob_normalizer += goal_predictor_->ProbAtInd(i);
    action_derivative -= goal_dist[i] * goal_policies_[i].QDerivative_Arbitration_Cached();
    prob_normalizer += goal_dist[i];
  }
  action_derivative /= prob_normalizer;

  //VisualizeAction(curr_pose, action_derivative + human_action);

  return action_derivative; //+ curr_action;
}

template <typename Instance>
typename Instance::ActionVector AssistancePolicy<Instance>::GetAssistedAction_Cached()
{
    ActionVector curr_action = Instance::ControlToAction(curr_control_cached_);
    //return curr_action;
    
    ActionVector action_derivative;
    action_derivative.setZero();
    double prob_normalizer = 0;
    for (size_t i=0; i < goal_policies_.size(); i++)
    {
        action_derivative -= goal_predictor_->ProbAtInd(i) * goal_policies_[i].QDerivative_Arbitration_Cached();
        prob_normalizer += goal_predictor_->ProbAtInd(i);
    }
    action_derivative /= prob_normalizer;
    
    //VisualizeAction(curr_pose, action_derivative + human_action);
    
    return action_derivative + curr_action;
}

template <typename Instance>
void AssistancePolicy<Instance>::GetValueAtPoses_OneGoal(const std::vector<Eigen::Affine3d>& poses, int goal, std::vector<double>& vals)
{
  vals.resize(poses.size());
  for (size_t i=0; i < poses.size(); i++)
  {
    vals[i] = GetValueAtPose_OneGoal(poses[i], goal);
  }
}


//template class AssistancePolicy<pathlength::PathLengthInstance>;
//template class AssistancePolicy<lqr::LQRInstance>;
//template class AssistancePolicy<HuberInstance>;
template class AssistancePolicy<HuberRotationInstance>;

}
