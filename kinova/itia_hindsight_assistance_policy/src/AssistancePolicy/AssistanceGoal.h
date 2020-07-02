#ifndef ASSISTANCE_GOAL_H
#define ASSISTANCE_GOAL_H

#include <eigen3/Eigen/Geometry>
#include "UtilFunctions.h"

namespace assistance_policy {

template <typename State>
class AssistanceGoal
{
  public:
    AssistanceGoal() {};
    AssistanceGoal(const std::vector<State>& targets, const Eigen::Affine3d& pose) : targets_(targets), pose_(pose) {};

    std::vector<State> targets_;
    Eigen::Affine3d pose_;
};


} //namespace
#endif
