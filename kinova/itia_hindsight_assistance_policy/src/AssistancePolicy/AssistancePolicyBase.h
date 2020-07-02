#ifndef ASSISTANCE_POLICY_BASE_H
#define ASSISTANCE_POLICY_BASE_H

#include "GoalPredictionBase.h"
#include "GoalPolicy.h"
#include "AssistanceGoal.h"
//#include "PathLengthInstance.h"
//#include "LQRInstance.h"
#include "HuberInstance.h"
#include "HuberRotationInstance.h"

#include <boost/shared_ptr.hpp> 
#include <sstream>
#include <math.h>
#include <algorithm>
#include <assert.h>


#include <iostream>
#include <fstream>

namespace assistance_policy {

const std::string plotting_obj_base_name_ = "plotting_box_";
const std::string plotting_grasppose_base_name_ = "best_grasp_pose_";

template <typename Instance>
class AssistancePolicy
{
  typedef typename Instance::StateVector StateVector;
  typedef typename Instance::ActionVector ActionVector;

  public:  
    AssistancePolicy(const std::vector< std::vector<Eigen::Affine3d> >& target_poses, 
                     const std::vector<Eigen::Affine3d>& goal_poses, 
                     const std::vector<double>& goal_dist) 
                     { Initialize(target_poses, goal_poses, goal_dist);}

    ~AssistancePolicy();
    void Initialize(const std::vector<std::vector<Eigen::Affine3d> >& target_poses, 
                    const std::vector<Eigen::Affine3d>& goal_poses, 
                    const std::vector<double>& goal_dist); 

    void Reinitialize(const std::vector<std::vector<Eigen::Affine3d> >& target_poses, 
                      const std::vector<Eigen::Affine3d>& goal_poses, 
                      const std::vector<double>& goal_dist)
    {
        goal_policies_.clear();
        for (size_t i=0; i < target_poses.size(); i++)
        {
            goal_policies_.push_back(PosesToGoal(target_poses[i], goal_poses[i]));
        }
    } 
   // virtual ActionVector GetAssistedAction(const Eigen::Affine3d& curr_pose, const Eigen::Vector3d& human_control);
    
    inline Eigen::Affine3d ApplyActionToPose(const Eigen::Affine3d& pose, const ActionVector& action_to_apply)
    {
      return Instance::ApplyActionToPose(pose, action_to_apply);
    }

    inline Eigen::Affine3d ApplyControlToPose(const Eigen::Affine3d& pose, const Eigen::Vector3d& control_to_apply)
    {
      return ApplyActionToPose(pose, Instance::ControlToAction(control_to_apply));
    }


    ActionVector GetAssistedAction_OneGoal_Cached(int goal_ind)
    {
        return goal_policies_[goal_ind].QDerivative_Arbitration_Cached();
    }
 
    //////HELPER FUNCTION ADDED JUST FOR RSS EXPERIMENTS
    double GetTranslationDistance_OneGoal_Cached(int goal_ind)
    {
        return goal_policies_[goal_ind].MinTranslationDist_Cached();
    }   

    void UpdatePoseAndAction(const Eigen::Affine3d& curr_pose, const Eigen::Vector3d& robot_control)
    {
//       std::ofstream myfile ("/home/spelle/ros_ws/minpose.txt", std::ios::out | std::ios::app);
//       std::ofstream myfile2 ("/home/spelle/ros_ws/returned_targets.txt", std::ios::out);
//       myfile << "************************\n";
      
      curr_pose_cached_ = curr_pose;
      curr_control_cached_ = robot_control;
      StateVector curr_state_cached = Instance::PoseToState(curr_pose);  //traslazione e rotazione
      ActionVector curr_action_cached = Instance::ControlToAction(robot_control);   //traforma parte traslazionale di controllo in parte traslazionale di azione

      for (size_t i=0; i < goal_policies_.size(); i++)
      {
        goal_policies_[i].CacheDistanceInfo(curr_state_cached, curr_action_cached);
        
        
//         Eigen::Affine3d optpose = goal_policies_[i].MinValuePose(curr_state_cached);
//         myfile << "optimal_poses minvalue goal "<< i+1 <<"\n" << optpose.matrix() << "\n";
//         Eigen::Affine3d optq = goal_policies_[i].MinQValPose(curr_state_cached,curr_action_cached);
//         myfile << "optimal_poses minqvalue goal "<< i+1 <<"\n" << optq.matrix() << "\n";       
//         
//         std::vector<Eigen::Affine3d> xx  = goal_policies_[i].AllGoalPoses();
//         for (int o=0;o<xx.size();o++)
//         {
//             myfile2 << "poses "<< "\n";
//             myfile2 << xx[o].matrix() << "\n";
//         }
      }
//       myfile.close();           
    }
    
    inline void UpdatePrediction_Cached()
    {
        goal_predictor_->UpdatePrediction_Cached();
    }
    
    void UpdatePrediction(const Eigen::Affine3d& curr_pose, const Eigen::Vector3d& human_control)
    {
        UpdatePoseAndAction(curr_pose, human_control);
        UpdatePrediction_Cached();
    }
    //inline void VisualizePrediction(){goal_predictor_->VisualizePrediction();}
    
    inline std::vector<double> GetGoalDistribution(){return goal_predictor_->GetGoalDistribution();}
    boost::shared_ptr<GoalPredictor<Instance> > GetGoalPredictor(){return goal_predictor_;}
    

    // Get the instantaneous cost of each pose
    void GetCostAtPoses_OneGoal(const std::vector<Eigen::Affine3d>& poses, int goal, std::vector<double>& vals);
    void GetValueAtPoses_OneGoal(const std::vector<Eigen::Affine3d>& poses, int goal, std::vector<double>& vals);
     double GetCostAtPose_OneGoal(const Eigen::Affine3d& pose, int goal)
    {
      StateVector state = Instance::PoseToState(pose);
      return goal_policies_[goal].CostOfState(state);
    };

    double GetValueAtPose_OneGoal(const Eigen::Affine3d& pose, int goal)
    {
      StateVector state = Instance::PoseToState(pose);
      return goal_policies_[goal].Value_Arbitration(state);
    }
    
    Eigen::Affine3d  GetMinValuePose_OneGoal(int goal)
    {
        StateVector curr_state_cached = Instance::PoseToState(curr_pose_cached_); 
        return goal_policies_[goal].MinValuePose(curr_state_cached);
    }

    ActionVector GetAssistedAction_Cached(std::vector<double> goal_dist);
    ActionVector GetAssistedAction_Cached();


    

    
    
  protected:
    AssistanceGoal<typename Instance::StateVector> PosesToGoal(const std::vector<Eigen::Affine3d>& goal_poses, const Eigen::Affine3d& object_pose);

    std::vector<GoalPolicy<Instance> > goal_policies_;
    boost::shared_ptr<GoalPredictor<Instance> > goal_predictor_;
    
    
    Eigen::Affine3d curr_pose_cached_;
    Eigen::Vector3d curr_control_cached_;
    //StateVector curr_state_cached_;
    //ActionVector curr_action_cached_;


};

}

#endif /* ASSISTANCE_POLICY_BASE_H */
