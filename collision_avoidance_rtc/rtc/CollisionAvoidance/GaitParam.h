#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil>
#include <vector>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  
  class FootStepNodes {
  public:
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame.
    std::vector<bool> isSupportPhase = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. footstepNodesListの末尾の要素が両方falseであることは無い
    double remainTime = 0.0;
  };
  std::vector<FootStepNodes> footstepNodesList;

  std::vector<std::vector<cnoid::Vector2> > steppable_region; // 要素数任意. supportLeg相対. endCoordsが存在できる領域
  std::vector<double> steppable_height; // 要素数はsteppable_regionと同じ. supportLeg相対. 各polygonごとのおおよその値.

  // 予見制御用
  cnoid::Vector3 curZmp = cnoid::Vector3(0,0,0);
  cnoid::Vector3 genCog = cnoid::Vector3(0,0,0);
  cnoid::Vector3 genCogVel = cnoid::Vector3(0,0,0);
  double omega = std::sqrt(9.8 / 1.0);
  cnoid::Vector3 l = cnoid::Vector3(0,0,0);
  double dt = 0.2;
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj;

  // IK用
  cnoid::Vector3 tgtCog;
};

inline std::ostream &operator<<(std::ostream &os, const std::vector<GaitParam::FootStepNodes>& footstepNodesList) {
  for(int i=0;i<footstepNodesList.size();i++){
    os << "footstep" << i << std::endl;
    os << " RLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[RLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[RLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << " LLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[LLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[LLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << " time = " << footstepNodesList[i].remainTime << "[s]" << std::endl;;
  }
  return os;
};
#endif
