#ifndef PRIORITIZEDIKSOLVER_H
#define PRIORITIZEDIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>
#include "CollisionChecker.h"

class PrioritizedIKSolver{
public:
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > selfCollisionConstraint;
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > envCollisionConstraint;
protected:
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  void init(const cnoid::BodyPtr& Robot, const GaitParam& gaitParam, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> collisionPairs){
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
    this->selfCollisionConstraint.clear();
    for (int i=0;i<collisionPairs.size();i++) this->selfCollisionConstraint.push_back(std::make_shared<IK::ClientCollisionConstraint>());
  }
  bool solveFullBodyIK(double dt, const GaitParam& gaitParam, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> selfCollisionPairs, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> envCollisionPairs, cnoid::BodyPtr& robot) const;
};

#endif
