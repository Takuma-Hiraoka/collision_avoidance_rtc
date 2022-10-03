#include "ComCoordsGenerator.h"

void ComCoordsGenerator::calcZmpTrajectory(const GaitParam gaitParam, std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj) const{
  
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = gaitParam.refZmpTraj;
  {
    cnoid::Vector3 refZmp = gaitParam.curZmp;
    refZmpTraj.clear();
    for(int i=0;i<gaitParam.footstepNodesList.size() && i < this->previewStepNum;i++){

      if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){
        // 跳躍についてはひとまず考えない TODO
	// copOffsetもひとまず考えない TODO
      }else if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){// 右脚がswing. refzmpは左脚の位置
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpGoalPos = llegGoalCoords.translation();
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, gaitParam.dt)));
        refZmp = zmpGoalPos;
      }else if(gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){ // 左脚がswing. refzmpは右脚の位置
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpGoalPos = rlegGoalCoords.translation();
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, gaitParam.dt)));
        refZmp = zmpGoalPos;
      }else{ // 両脚がsupport. refzmpは一つ前の区間と一つ後の区間を線形につなぐ
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG];
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG];
        cnoid::Vector3 rlegCOP = rlegGoalCoords.translation();
        cnoid::Vector3 llegCOP = llegGoalCoords.translation();
        cnoid::Vector3 zmpGoalPos;
        if(i==gaitParam.footstepNodesList.size()-1 || //末尾. 以降は末尾の状態がずっと続くとして扱うので、refzmpは両足の中心
           (gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]) // 次も両脚支持. refzmpは両足の中心
           ){
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }else if(!gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]){ // 次は右脚がswing
          zmpGoalPos = llegCOP;
        }else if(gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]){ // 次は左脚がswing
          zmpGoalPos = rlegCOP;
        }else{// 次は跳躍. TODO
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, gaitParam.dt)));
        refZmp = zmpGoalPos;
      }
    }
    if(gaitParam.footstepNodesList.size() == 1){
      // footGuidedBalanceTime[s]に満たない場合、満たないぶんだけ末尾に加える. そうしないと終端条件が厳しすぎる. 一方で、常に末尾にfootGuidedBalanceTime[s]だけ加えると、終端条件がゆるすぎて重心を動かすのが遅すぎる.
      double totalTime = 0;
      for(int i=0;i<refZmpTraj.size();i++) totalTime += refZmpTraj[i].getTime();
      if(totalTime < this->footGuidedBalanceTime){
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,refZmp, std::max(this->footGuidedBalanceTime - totalTime, gaitParam.dt)));
      }
    }
  }
  o_refZmpTraj = refZmpTraj;
}

void ComCoordsGenerator::calcComCoords() const{
}
