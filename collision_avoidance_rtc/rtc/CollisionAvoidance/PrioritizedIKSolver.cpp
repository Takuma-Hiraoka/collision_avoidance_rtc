#include "PrioritizedIKSolver.h"

bool PrioritizedIKSolver::solveFullBodyIK(double dt, const GaitParam& gaitParam,cnoid::BodyPtr& robot) const{

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint0;
  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint1;
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
						     gaitParam.dt //dtでよいのか？
						     );

  return true;
}
