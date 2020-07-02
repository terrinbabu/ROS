#ifndef PREDICTION_BASE_H
#define PREDICTION_BASE_H

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "GoalPolicy.h"

namespace assistance_policy {



template <typename Instance>
class GoalPredictor
{
  typedef typename Instance::StateVector StateVector;
  typedef typename Instance::ActionVector ActionVector;


  public:
    //GoalPredictor(const std::vector<double>& initial_goal_prob){Initialize(initial_goal_prob);}
    ~GoalPredictor() { }
    
    void Initialize(const std::vector<double>& initial_goal_prob) {
      log_goal_distribution_.resize(initial_goal_prob.size());
      for (size_t i=0; i < initial_goal_prob.size(); i++) {
        log_goal_distribution_[i] = log(initial_goal_prob[i]);
      }
    }

    //get the current distribution over Goals
    std::vector<double> GetGoalDistribution() {
      std::vector<double> goal_distribution(log_goal_distribution_.size());
      for (size_t i=0; i < log_goal_distribution_.size(); i++)
      {
        goal_distribution[i] = exp(log_goal_distribution_[i]);
      }
      return goal_distribution;
    }

    double ProbAtInd(size_t i){return exp(log_goal_distribution_[i]);}
    void SetGoalPolicies(std::vector<GoalPolicy<Instance> >& goal_policies)
    { 
      goal_policies_ = &goal_policies;
    }


    //virtual void UpdatePrediction(const StateVector& curr_state, const ActionVector& human_action)

    virtual void UpdatePrediction_Cached()
    {
      
      for (size_t i=0; i < goal_policies_->size(); i++)
      {
        double q_val = goal_policies_->at(i).QValue_Prediction_Cached();
        double v_val = goal_policies_->at(i).Value_Prediction_Cached();
        log_goal_distribution_[i] -= q_val - v_val;
      }
      

      //calculate normalization constant
      NormalizeLogDistribution();

//      std::cout << "Log Goal Distribution: ";
//      for (size_t i=0; i < log_goal_distribution_.size(); i++)
//      {
//        std::cout << log_goal_distribution_[i] << "  ";
//      }
//      std::cout << std::endl;
      //std::cout << "\t normalization: " << log_normalization_val << std::endl;
    }


    double NormalizeLogDistribution()
    {
      double log_normalization_val = LogSumExp(log_goal_distribution_);

      for (size_t i=0; i < log_goal_distribution_.size(); i++)
      {
        log_goal_distribution_[i] -= log_normalization_val;
      }

      return log_normalization_val;
    }

  protected:
    int init_vis_count_;
    std::vector<double> log_goal_distribution_;
    std::vector<GoalPolicy<Instance> >* goal_policies_;
};

}


#endif /* PREDICTION_BASE_H */
