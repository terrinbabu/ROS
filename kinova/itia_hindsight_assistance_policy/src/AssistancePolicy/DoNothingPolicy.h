#ifndef DO_NOTHING_POLICY_H
#define DO_NOTHING_POLICY_H


#include "AssistancePolicyBase.h"
#include "DoNothingPrediction.h"

class DoNothingPolicy : public AssistancePolicy
{
  public:
    DoNothingPolicy(const std::vector<Eigen::Affine3d>& goals, const std::vector<double>& goal_dist)
    {
      Initialize(goals, goal_dist);
    };
    void Initialize(const std::vector<Eigen::Affine3d>& goals, const std::vector<double>& goal_dist)
    {
      goals_ = goals;
      goal_predictor_.reset(new DoNothingPredictor());
      goal_predictor_->Initialize(goal_dist);
    }
    Eigen::Vector3d GetAssistedAction(const Eigen::Affine3d& curr_state, const Eigen::Vector3d& human_action)
    {
      return human_action;
    }

};



#endif /* DO_NOTHING_POLICY_H */
