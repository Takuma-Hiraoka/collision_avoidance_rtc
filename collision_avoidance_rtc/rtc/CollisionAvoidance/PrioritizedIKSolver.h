#ifndef PRIORITIZEDIKSOLVER_H
#define PRIORITIZEDIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
#include <ik_constraint/JointLimitConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>
#include "CollisionChecker.h"

class PrioritizedIKSolver{
public:
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > selfCollisionConstraint;
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > envCollisionConstraint;
  mutable std::vector<std::shared_ptr<IK::JointLimitConstraint> > jointLimitConstraint;
  mutable std::vector<std::shared_ptr<IK::JointAngleConstraint> > refJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<IK::PositionConstraint> rootPositionConstraint = std::make_shared<IK::PositionConstraint>();
protected:
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  void init(const cnoid::BodyPtr& robot, const GaitParam& gaitParam, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> collisionPairs){
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
    this->selfCollisionConstraint.clear();
    for (int i=0;i<collisionPairs.size();i++) this->selfCollisionConstraint.push_back(std::make_shared<IK::ClientCollisionConstraint>());
    jointLimitConstraint.clear();
    for(int i=0;i<robot->numJoints();i++) jointLimitConstraint.push_back(std::make_shared<IK::JointLimitConstraint>());
    refJointAngleConstraint.clear();
    for(int i=0;i<robot->numJoints();i++) refJointAngleConstraint.push_back(std::make_shared<IK::JointAngleConstraint>());
  }
  bool solveFullBodyIK(double dt, const GaitParam& gaitParam, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> selfCollisionPairs, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> envCollisionPairs, cnoid::BodyPtr& robot, const int iter) const;
};

#endif
