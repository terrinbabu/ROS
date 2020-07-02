#ifndef GOAGOALCY_H
#define GOAGOALCY_H

/***********************************************************
 * Policy over Goals, which just uses a set of policy instances
 ***********************************************************/

#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Geometry>
#include "AssistanceGoal.h"

namespace assistance_policy {

template <typename Instance>
class GoalPolicy
{
  typedef typename Instance::StateVector StateVector;
  typedef typename Instance::ActionVector ActionVector;

  public:
    GoalPolicy(const AssistanceGoal<StateVector>& goal)
      : goal_(goal)
    {
      for (size_t i=0; i < goal_.targets_.size(); i++)
      {
        instances_.push_back(Instance(goal_.targets_[i]));
      }
    }

    double QValue_Prediction(const StateVector& state, const ActionVector& action)
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Prediction(state, action, q_values);
      //return *std::min_element(q_values.begin(), q_values.end());
      return SoftMin(q_values);
    }

    double QValue_Arbitration(const StateVector& state, const ActionVector& action)
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration(state, action, q_values);
      return *std::min_element(q_values.begin(), q_values.end());
    }

    double Value_Prediction(const StateVector& state)
    {
      std::vector<double> values(instances_.size());
      GetValues_Prediction(state, values);
      //return *std::min_element(values.begin(), values.end());
      return SoftMin(values);
    }
    
    double Value_Arbitration(const StateVector& state)
    {
      std::vector<double> values(instances_.size());
      GetValues_Arbitration(state, values);
      return *std::min_element(values.begin(), values.end());
    }

    double CostOfState(const StateVector& state)
    {
      std::vector<double> costs(instances_.size());
      GetCosts(state, costs);
      return *std::min_element(costs.begin(), costs.end());
    }

    void GetQValues_Prediction(const StateVector& state, const ActionVector& action, std::vector<double>& q_values)
    {
      q_values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        q_values[i] = instances_[i].QValue_Prediction(state, action);
      }
    }

    void GetQValues_Arbitration(const StateVector& state, const ActionVector& action, std::vector<double>& q_values)
    {
      q_values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        q_values[i] = instances_[i].QValue_Arbitration(state, action);
      }
    }

    void GetValues_Prediction(const StateVector& state, std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].Value_Prediction(state);
      }
    }

    void GetValues_Arbitration(const StateVector& state, std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].Value_Arbitration(state);
      }
    }
  
    void GetCosts(const StateVector& state, std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].CostOfState(state);
      }
    }

    ActionVector QDerivative_Arbitration(const StateVector& state, const ActionVector& action)
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration(state, action, q_values);
      
      size_t min_index = std::min_element(q_values.begin(), q_values.end()) - q_values.begin();

      return instances_[min_index].QDerivative_Arbitration(state, action);
    }

    Eigen::Affine3d MinQValPose(const StateVector& state, const ActionVector& action)
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration(state, action, q_values);
      
      size_t min_index = std::min_element(q_values.begin(), q_values.end()) - q_values.begin();

      return Instance::StateToPose(goal_.targets_[min_index]);
    }

    Eigen::Affine3d MinValuePose(const StateVector& state)
    {
      std::vector<double> values(instances_.size());
      GetValues_Arbitration(state, values);
      
      size_t min_index = std::min_element(values.begin(), values.end()) - values.begin();

      return Instance::StateToPose(goal_.targets_[min_index]);
    }

    double QValue_Prediction_Cached()
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Prediction_Cached(q_values);
      //return *std::min_element(q_values.begin(), q_values.end());
      return SoftMin(q_values);
    }

    double QValue_Arbitration_Cached()
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration_Cached(q_values);
      return *std::min_element(q_values.begin(), q_values.end());
    }

    double Value_Prediction_Cached()
    {
      std::vector<double> values(instances_.size());
      GetValues_Prediction_Cached(values);
      //return *std::min_element(values.begin(), values.end());
      return SoftMin(values);
    }
    
    double Value_Arbitration_Cached()
    {
      std::vector<double> values(instances_.size());
      GetValues_Arbitration_Cached(values);
      return *std::min_element(values.begin(), values.end());
    }

    double CostOfState_Cached()
    {
      std::vector<double> costs(instances_.size());
      GetCosts_Cached(costs);
      return *std::min_element(costs.begin(), costs.end());
    }

    void GetQValues_Prediction_Cached(std::vector<double>& q_values)
    {
      q_values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        q_values[i] = instances_[i].QValue_Prediction_Cached();
      }
    }

    void GetQValues_Arbitration_Cached(std::vector<double>& q_values)
    {
      q_values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        q_values[i] = instances_[i].QValue_Arbitration_Cached();
      }
    }

    void GetValues_Prediction_Cached(std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].Value_Prediction_Cached();
      }
    }

    void GetValues_Arbitration_Cached(std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].Value_Arbitration_Cached();
      }
    }
  
    void GetCosts_Cached(std::vector<double>& values)
    {
      values.resize(instances_.size());
      for (size_t i=0; i < instances_.size(); i++)
      {
        values[i] = instances_[i].CostOfState_Cached();
      }
    }

    ActionVector QDerivative_Arbitration_Cached()
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration_Cached(q_values);
      
      size_t min_index = std::min_element(q_values.begin(), q_values.end()) - q_values.begin();

      return instances_[min_index].QDerivative_Arbitration_Cached();
    }

    //////HELPER FUNCTION ADDED JUST FOR RSS EXPERIMENTS
    double MinTranslationDist_Cached()
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration_Cached(q_values);
      
      size_t min_index = std::min_element(q_values.begin(), q_values.end()) - q_values.begin();
        
      return instances_[min_index].StateDistance_Translation(curr_state_cached_, instances_[min_index].GoalState());
    }

    Eigen::Affine3d MinQValPose_Cached()
    {
      std::vector<double> q_values(instances_.size());
      GetQValues_Arbitration_Cached(q_values);
      
      size_t min_index = std::min_element(q_values.begin(), q_values.end()) - q_values.begin();

      return goal_.targets_[min_index];
    }

    Eigen::Affine3d MinValuePose_Cached()
    {
      std::vector<double> values(instances_.size());
      GetValues_Arbitration_Cached();
      
      size_t min_index = std::min_element(values.begin(), values.end()) - values.begin();

      return Instance::StateToPose(goal_.targets_[min_index]);
    }

    std::vector<Eigen::Affine3d> AllGoalPoses()
    {
      std::vector<Eigen::Affine3d> poses;
      for (size_t i=0; i < goal_.targets_.size(); i++)
      {
        poses.push_back(Instance::StateToPose(goal_.targets_[i])); 
      }
      return poses;
    }

    void CacheDistanceInfo(const StateVector& state, const ActionVector& action)
    {
      //ARMS_DEBUG("caching target at " << target_.pose_.translation().transpose());
      //ARMS_DEBUG("target center " << target_.obj_ray_intersection_.center_.transpose());
      
      curr_state_cached_ = state;
      curr_action_cached_ = action;

      for (size_t i=0; i < instances_.size(); i++)
      {
        instances_[i].CacheDistanceInfo(state, action, goal_);
      }
    }

    inline const StateVector& GetCachedState() const {return curr_state_cached_;}
    inline const ActionVector& GetCachedAction() const {return curr_action_cached_;}



    inline AssistanceGoal<StateVector>& GetGoal() { return goal_;}
    const inline AssistanceGoal<StateVector>& GetGoal() const { return goal_;}


  protected:  
    std::vector<Instance> instances_;
    AssistanceGoal<StateVector> goal_;

    StateVector curr_state_cached_;
    ActionVector curr_action_cached_;

};

}


#endif /* PREDICTION_BASE_H */
