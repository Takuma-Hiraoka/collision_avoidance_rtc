#include "AvoidancePlanner.h"

void AvoidancePlanner::calcSafeHulls(const std::vector<GaitParam::FootStepNodes>& footStepNodesList, const std::vector<std::vector<cnoid::Vector2> > steppable_region, const std::vector<double> steppable_height, std::vector<std::vector<cnoid::Vector3> > o_steppableHulls, std::vector<double> o_steppableHeights, std::vector<std::vector<cnoid::Vector3>> o_safeHulls) const{
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footStepNodesList.size() > 1 &&
       (footStepNodesList[1].isSupportPhase[RLEG] && footStepNodesList[1].isSupportPhase[LLEG]) &&
       ((footStepNodesList[0].isSupportPhase[RLEG] && !footStepNodesList[0].isSupportPhase[LLEG]) || (!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[0].isSupportPhase[LLEG]))))
     return;

  int swingLeg = footStepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position supportPose = footStepNodesList[0].dstCoords[supportLeg];
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  // footStepNodesListをもとにstridelimitatinoを満たしsteppableなregionを計算
  // 一歩の間に起こる動的な外乱に対応するためのものではないので、残り時間でreachableかどうかは考えず、StrideLimitationとSteppableReginのみを考える。

  // これによって出されたsafeHullsを使って、計画したfootStepNodesListの次の一歩を、実行可能なものに修正する
  // 理想的にはauto_stabilizerが次の一歩がSteppableなfootStepNodesListを計算しているはずなので、CollisionAvoidanceで足を動かせる領域を計算しておくことが主目的

  std::vector<std::vector<cnoid::Vector3>> candidates;
  std::vector<std::vector<cnoid::Vector3>> steppableHulls;
  std::vector<double> steppableHeights;

  // strideLimitation とreachable
  {
    std::vector<cnoid::Vector3> strideLimitationHull;
    for(int i=0;i<this->overwritableStrideLimitationHull[swingLeg].size();i++){
      cnoid::Vector3 p = supportPoseHorizontal * this->overwritableStrideLimitationHull[swingLeg][i];
      strideLimitationHull.emplace_back(p[0],p[1],0.0);
    }
    candidates.emplace_back(strideLimitationHull);

    for(int i=0;i<steppable_region.size();i++){
      std::vector<cnoid::Vector3> steppableHull;
      for(int j=0;j<steppable_region[i].size();j++){
	cnoid::Vector3 p = supportPoseHorizontal * cnoid::Vector3(steppable_region[i][j](0), steppable_region[i][j](1),0);
	steppableHull.emplace_back(p);
      }
      double steppableHeight = supportPoseHorizontal.translation()[2] + steppable_height[i];
      steppableHulls.emplace_back(steppableHull);
      steppableHeights.emplace_back(steppableHeight);
    }
    std::vector<std::vector<cnoid::Vector3>> nextCandidates;
    for(int i=0;i<steppableHulls.size();i++) {
      std::vector<cnoid::Vector3> hull = mathutil::calcIntersectConvexHull(strideLimitationHull, steppableHulls[i]);
      if(hull.size() > 0) nextCandidates.emplace_back(hull);
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
  }

  o_steppableHulls = steppableHulls;
  o_steppableHeights = steppableHeights;
  o_safeHulls = candidates;  
};

void AvoidancePlanner::updateSafeFootStep(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const std::vector<std::vector<cnoid::Vector3> > steppableHulls, const std::vector<double> steppableHeights, const std::vector<std::vector<cnoid::Vector3>> safeHulls) const {
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footStepNodesList.size() > 1 &&
       (footStepNodesList[1].isSupportPhase[RLEG] && footStepNodesList[1].isSupportPhase[LLEG]) &&
       ((footStepNodesList[0].isSupportPhase[RLEG] && !footStepNodesList[0].isSupportPhase[LLEG]) || (!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[0].isSupportPhase[LLEG]))))
     return;

  if(safeHulls.size() == 0) return;

  int swingLeg = footStepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;

  for(int i=0;i<safeHulls.size();i++){
    if(mathutil::isInsideHull(footStepNodesList[0].dstCoords[swingLeg].translation(), safeHulls[i])) return;
  }

  // 最近傍点に修正する。一歩で乗り越えてしまうような薄い壁は無視してしまう．
  Eigen::Vector3d nearestPoint = Eigen::Vector3d::Zero();
  double distance = std::numeric_limits<double>::max();
  for(int i=0;i<safeHulls.size();i++){
     Eigen::Vector3d p = mathutil::calcNearestPointOfHull(footStepNodesList[0].dstCoords[swingLeg].translation(), safeHulls[i]);
    double d = (p - footStepNodesList[0].dstCoords[swingLeg].translation()).norm();
    if(distance > d){
      distance = d;
      nearestPoint = p;
    }
  }
  // TODO 高さ．どのsteppable_hullの中にあるかを調べて同じ位置のsteppable_heightにする．
  footStepNodesList[0].dstCoords[swingLeg].translation() = nearestPoint;
};
