#ifndef AVOIDANCEPLANNER_H
#define AVOIDANCEPLANNER_H

#include "GaitParam.h"
#include "MathUtil.h"
#include <iostream>
#include "CollisionChecker.h"

class AvoidancePlanner{
public:
  // remainTimeで着地するものとしてSteppableRegionを出す．
  std::vector<std::vector<cnoid::Vector3> > overwritableStrideLimitationHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>{cnoid::Vector3(0.4,-0.18,0),cnoid::Vector3(-0.3,-0.18,0),cnoid::Vector3(-0.3,-0.30,0),cnoid::Vector3(-0.15,-0.45,0),cnoid::Vector3(0.15,-0.45,0),cnoid::Vector3(0.4,-0.30,0)},std::vector<cnoid::Vector3>{cnoid::Vector3(0.4,0.30,0),cnoid::Vector3(0.15,0.45,0),cnoid::Vector3(-0.15,0.45,0),cnoid::Vector3(-0.3,0.30,0),cnoid::Vector3(-0.3,0.18,0),cnoid::Vector3(0.4,0.18,0)}}; // 要素数2. 0: rleg用, 1: lleg用. 着地位置修正時に自動生成されるfootstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). あったほうが扱いやすいのでZ成分があるが、Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaとは独立に評価されるので、defaultStrideLimitationThetaだけ傾いていても大丈夫なようにせよ. 斜め方向の角を削るなどして、IKが解けるようにせよ. 歩行中は急激に変更されない．着地位置修正を行って段差を登る場合、段差の端から足裏ぶん離れた位置も領域に含めるために、X成分を大きめに取る必要がある。
  std::vector<std::vector<cnoid::Vector3> > steppableHulls; // 要素数任意. generate frame. endCoordsが存在できる領域
  std::vector<double> steppableHeights; // 要素数はsteppable_regionと同じ. generate frame. 各polygonごとのおおよその値.
  std::vector<std::vector<cnoid::Vector3>> safeHulls; // 要素数任意．generate frame．endCoordsが環境と干渉せず、かつ到達できる領域
  std::vector<std::vector<double>> angleTrajectory; //SequencePlayerに送る関節角度軌道

  double checkPlanTime = 0.3; // remainTimeがこの時間を上回るときのみ関節角度やfootstepを送る．

  bool checkPlanExecute(const std::vector<GaitParam::FootStepNodes> footstepNodesList) const; // これがtrueのときのみ計画、結果を出力する．逆に計画時はここでの条件を前提として良い．
  
  void calcSafeHulls(const std::vector<GaitParam::FootStepNodes> footStepNodesList, const std::vector<std::vector<cnoid::Vector2> > steppable_region, const std::vector<double> steppable_height, std::vector<std::vector<cnoid::Vector3>>& o_steppableHulls, std::vector<double>& o_steppableHeights, std::vector<std::vector<cnoid::Vector3>>& o_safeHulls) const; // footStepNodesListをもとにstridelimitatinoを満たしsteppableなregionを計算

  void updateSafeFootStep(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const std::vector<std::vector<cnoid::Vector3> > o_steppableHulls, const std::vector<double> o_steppableHeights, const std::vector<std::vector<cnoid::Vector3>> o_safeHulls) const; // footStepNodesListの先頭をSafeHullに近づけるように修正

  void calcAngleTrajectory(cnoid::BodyPtr& robot, const cnoid::BodyPtr& orgRobot, const double remainTime, const double dt, const CollisionChecker collisionChecker, std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& collisionPairs, std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> >& vclipModelMap, const std::shared_ptr<distance_field::PropagationDistanceField> field, const cnoid::Position fieldOrigin, const std::vector<cnoid::LinkPtr>& targetLinks, std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> >& verticesMap, std::vector<std::vector<double>> o_angleTrajectory) const; // 現在姿勢から逆運動学で出された干渉しない姿勢まで、干渉せずに到達できる関節角度軌道を計算．現在位置から最も動いた姿勢でもぶつからなければ、それまでの間もぶつからないと仮定し、rootLinkは逆運動学時と同じ(=最も動いたとき)とする。
};

#endif
