#include "PrioritizedIKSolver.h"

bool PrioritizedIKSolver::solveFullBodyIK(double dt, const GaitParam& gaitParam, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> selfCollisionPairs, const std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> envCollisionPairs, cnoid::BodyPtr& robot) const{

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint0;
  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint1;

  // self collision
  for(int i=0;i<selfCollisionPairs.size();i++){
    this->selfCollisionConstraint[i]->A_link() = robot->link(selfCollisionPairs[i]->link1->name().c_str());
    this->selfCollisionConstraint[i]->A_localp() = selfCollisionPairs[i]->localp1;
    this->selfCollisionConstraint[i]->B_link() = robot->link(selfCollisionPairs[i]->link2->name().c_str());
    this->selfCollisionConstraint[i]->B_localp() = selfCollisionPairs[i]->localp2;
    this->selfCollisionConstraint[i]->tolerance() = 0.01; //TODO
    this->selfCollisionConstraint[i]->maxError() = 10.0*gaitParam.footstepNodesList[0].remainTime;
    this->selfCollisionConstraint[i]->precision() = 0.0; // 強制的にIKをmax loopまで回す
    this->selfCollisionConstraint[i]->weight() = 1.0;
    this->selfCollisionConstraint[i]->velocityDamper() = 0.1 / gaitParam.footstepNodesList[0].remainTime;
    this->selfCollisionConstraint[i]->direction() = selfCollisionPairs[i]->direction21;
    ikConstraint0.push_back(this->selfCollisionConstraint[i]);
  }

  this->envCollisionConstraint.clear();
  for (int i=0;i<envCollisionPairs.size();i++) this->envCollisionConstraint.push_back(std::make_shared<IK::ClientCollisionConstraint>());
  // env collision
  for(int i=0;i<envCollisionPairs.size();i++){
    this->envCollisionConstraint[i]->A_link() = robot->link(envCollisionPairs[i]->link1->name().c_str());
    this->envCollisionConstraint[i]->A_localp() = envCollisionPairs[i]->localp1;
    this->envCollisionConstraint[i]->B_link() = nullptr;
    this->envCollisionConstraint[i]->B_localp() = envCollisionPairs[i]->localp2;
    this->envCollisionConstraint[i]->tolerance() = 0.1; //TODO
    this->envCollisionConstraint[i]->maxError() = 10.0*gaitParam.footstepNodesList[0].remainTime;
    this->envCollisionConstraint[i]->precision() = 0.0; // 強制的にIKをmax loopまで回す
    this->envCollisionConstraint[i]->weight() = 10.0;
    this->envCollisionConstraint[i]->velocityDamper() = 1.0 / gaitParam.footstepNodesList[0].remainTime;
    this->envCollisionConstraint[i]->direction() = envCollisionPairs[i]->direction21;
    ikConstraint0.push_back(this->envCollisionConstraint[i]);
  }

  // EEF
  for(int i=0;i<NUM_LEGS;i++){
    this->ikEEPositionConstraint[i]->A_link() = robot->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->ikEEPositionConstraint[i]->B_link() = nullptr;
    this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.eeTargetPose[i];
    this->ikEEPositionConstraint[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->ikEEPositionConstraint[i]->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    if(i<NUM_LEGS) this->ikEEPositionConstraint[i]->weight() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    else this->ikEEPositionConstraint[i]->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    this->ikEEPositionConstraint[i]->eval_link() = nullptr;
    this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->B_localpos().linear();
    ikConstraint1.push_back(this->ikEEPositionConstraint[i]);
  }

  // COM
  {
    this->comConstraint->A_robot() = robot;
    this->comConstraint->A_localp() = cnoid::Vector3::Zero();
    this->comConstraint->B_robot() = nullptr;
    this->comConstraint->B_localp() = gaitParam.tgtCog;
    this->comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    this->comConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->comConstraint->weight() << 3.0, 3.0, 1.0;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint1.push_back(this->comConstraint);
  }

  std::vector<cnoid::LinkPtr> variables;
  variables.push_back(robot->rootLink());
  for(size_t i=0;i<robot->numJoints();i++){
    variables.push_back(robot->joint(i));
  }
  std::vector<std::vector<std::shared_ptr<IK::IKConstraint> > > constraints{ikConstraint0,ikConstraint1};
  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debuglevel() = 0;//debug
    }
  }

  prioritized_inverse_kinematics_solver::solveIKLoop(variables,
						     constraints,
						     this->tasks,
						     1,//loop
						     1e-6, // wn
						     0, //debug
						     gaitParam.footstepNodesList[0].remainTime
						     );

  return true;
}
