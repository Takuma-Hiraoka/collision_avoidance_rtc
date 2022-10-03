#ifndef COMCOORDSGENERATOR_H
#define COMCOORDSGENERATOR_H

#include "FootGuidedController.h"
#include "GaitParam.h"

class ComCoordsGenerator{
public:
  int previewStepNum = 4; //footstepNodesListがこれ以上あっても予見制御は無視する
  double footGuidedBalanceTime = 0.6; // 終端条件のためfootstepの合計時間がこれより短いときはこの時間になるまで最後の状態を維持する

  // auto_stabilizerから来たzmp,cog,cogvel,omega,l,dt,footstep(修正後も含む)をもとに、大雑把なzmp軌道を出す。
  void calcZmpTrajectory(const GaitParam gaitParam, std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj) const;

  // calcZmpTrajectoryで出された大雑把なzmp軌道をfootguidedcontrollerによって実行zmp軌道に直し、倒立振子に従うとして目標時刻のcomを出す
  void calcComCoords(const GaitParam gaitParam, cnoid::Vector3& o_tgtCog) const;
};

#endif
