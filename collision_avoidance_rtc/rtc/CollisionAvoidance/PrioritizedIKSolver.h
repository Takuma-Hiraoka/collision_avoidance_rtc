#ifndef PRIORITIZEDIKSOLVER_H
#define PRIORITIZEDIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>

class PrioritizedIKSolver{
public:
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
protected:
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  void init(const cnoid::BodyPtr& Robot, const GaitParam& gaitParam){
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
  }
  bool solveFullBodyIK(double dt, const GaitParam& gaitParam,cnoid::BodyPtr& robot) const;
};

#endif
