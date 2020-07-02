#ifndef DO_NOTHING_PREDICTION_H
#define DO_NOTHING_PREDICTION_H

#include "GoalPredictionBase.h"
#include <eigen3/Eigen/Geometry>


class DoNothingPredictor : public GoalPredictor
{
  public:
    void UpdatePrediction(const Eigen::Affine3d& curr_state, const Eigen::Vector3d& human_action) { };

  //protected:
};

#endif /* DO_NOTHING_PREDICTION_H */
