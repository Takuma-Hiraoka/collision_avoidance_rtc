#include "CollisionAvoidance.h"

CollisionAvoidance::CollisionAvoidance(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_steppableRegionIn_("steppableRegionIn", m_steppableRegion_),
  m_refFootStepNodesListIn_("refFootStepNodesListIn", m_refFootStepNodesList_),
  m_comPredictParamIn_("comPredictParamIn", m_comPredictParam_),
  m_footStepNodesListOut_("footStepNodesListOut", m_footStepNodesList_)
{
}

RTC::ReturnCode_t CollisionAvoidance::onInitialize(){
  addInPort("steppableRegionIn", m_steppableRegionIn_);
  addInPort("refFootStepNodesListIn", m_refFootStepNodesListIn_);
  addInPort("comPredictParamIn", m_comPredictParamIn_);
  addOutPort("footStepNodesListOut", m_footStepNodesListOut_);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionAvoidance::onExecute(RTC::UniqueId ec_id){
  std::cerr << "Collision Avoidance rtc onExecute" << std::endl;
  cnoid::TimeMeasure timer;
  timer.begin();
  // TODO 本来はserviceにするべき？

  // read port
  if(this->m_steppableRegionIn_.isNew()){
    m_steppableRegionIn_.read();
    if ((gaitParam_.footstepNodesList[0].isSupportPhase[RLEG] && (m_steppableRegion_.data.l_r == 0)) ||
	(gaitParam_.footstepNodesList[0].isSupportPhase[LLEG] && (m_steppableRegion_.data.l_r == 1))){ //現在支持脚と計算時支持脚が同じ
      gaitParam_.steppable_region.resize(m_steppableRegion_.data.region.length());
      gaitParam_.steppable_height.resize(m_steppableRegion_.data.region.length());
      for (int i=0; i<gaitParam_.steppable_region.size(); i++){
	gaitParam_.steppable_region[i].resize(m_steppableRegion_.data.region[i].length()/3);
	double height_sum = 0.0;
	for (int j=0; j<gaitParam_.steppable_region[i].size(); j++){
	  gaitParam_.steppable_region[i][j](0) = m_steppableRegion_.data.region[i][3*j];
	  gaitParam_.steppable_region[i][j](1) = m_steppableRegion_.data.region[i][3*j+1];
	  height_sum += m_steppableRegion_.data.region[i][3*j+2];
	}
	gaitParam_.steppable_height[i] = height_sum / gaitParam_.steppable_region[i].size();
      }
    }
  }
  
  if(this->m_refFootStepNodesListIn_.isNew()){
    m_refFootStepNodesListIn_.read();
    gaitParam_.footstepNodesList.resize(m_refFootStepNodesList_.data.length());
    for(int i=0;i<gaitParam_.footstepNodesList.size();i++) {
      for(int j=0;j<NUM_LEGS;j++){
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[0] = m_refFootStepNodesList_.data[i].dstCoords[j].position.x;
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[1] = m_refFootStepNodesList_.data[i].dstCoords[j].position.y;
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[2] = m_refFootStepNodesList_.data[i].dstCoords[j].position.z;
	gaitParam_.footstepNodesList[i].dstCoords[j].linear() = cnoid::rotFromRpy(m_refFootStepNodesList_.data[i].dstCoords[j].orientation.r, m_refFootStepNodesList_.data[i].dstCoords[j].orientation.p, m_refFootStepNodesList_.data[i].dstCoords[j].orientation.y);
	gaitParam_.footstepNodesList[i].isSupportPhase[j] = m_refFootStepNodesList_.data[i].isSupportPhase[j];
      }
      gaitParam_.footstepNodesList[i].remainTime = m_refFootStepNodesList_.data[i].remainTime;
    }
  }

  if(this->m_comPredictParamIn_.isNew()){
    m_comPredictParamIn_.read();
    gaitParam_.curZmp[0] = m_comPredictParam_.curZmp.x;
    gaitParam_.curZmp[1] = m_comPredictParam_.curZmp.y;
    gaitParam_.curZmp[2] = m_comPredictParam_.curZmp.z;
    gaitParam_.genCog[0] = m_comPredictParam_.curCog.x;
    gaitParam_.genCog[1] = m_comPredictParam_.curCog.y;
    gaitParam_.genCog[2] = m_comPredictParam_.curCog.z;
    gaitParam_.genCogVel[0] = m_comPredictParam_.curCogVel.x;
    gaitParam_.genCogVel[1] = m_comPredictParam_.curCogVel.y;
    gaitParam_.genCogVel[2] = m_comPredictParam_.curCogVel.z;
    gaitParam_.omega = m_comPredictParam_.omega;
    gaitParam_.l[0] = m_comPredictParam_.l.x;
    gaitParam_.l[1] = m_comPredictParam_.l.y;
    gaitParam_.l[2] = m_comPredictParam_.l.z;
    gaitParam_.dt = m_comPredictParam_.dt;
  }

  // 着地可能な領域を計算
  avoidancePlanner_.calcSafeHulls(gaitParam_.footstepNodesList, gaitParam_.steppable_region, gaitParam_.steppable_height, avoidancePlanner_.steppableHulls, avoidancePlanner_.steppableHeights, avoidancePlanner_.safeHulls);

  // footstepを着地可能な領域になおす
  avoidancePlanner_.updateSafeFootStep(gaitParam_.footstepNodesList, avoidancePlanner_.steppableHulls, avoidancePlanner_.steppableHeights, avoidancePlanner_.safeHulls);

  comCoordsGenerator_.calcZmpTrajectory(gaitParam_, gaitParam_.refZmpTraj);
  
  // write port
  this->m_footStepNodesList_.data.length(gaitParam_.footstepNodesList.size());
  for(int i=0;i<this->m_footStepNodesList_.data.length();i++) {
    this->m_footStepNodesList_.data[i].dstCoords.length(NUM_LEGS);
    this->m_footStepNodesList_.data[i].isSupportPhase.length(NUM_LEGS);
    for(int j=0;j<NUM_LEGS;j++){
      this->m_footStepNodesList_.data[i].dstCoords[j].position.x = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[0];
      this->m_footStepNodesList_.data[i].dstCoords[j].position.y = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[1];
      this->m_footStepNodesList_.data[i].dstCoords[j].position.z = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[2];
      cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam_.footstepNodesList[i].dstCoords[j].linear());
      this->m_footStepNodesList_.data[i].dstCoords[j].orientation.r = rpy[0];
      this->m_footStepNodesList_.data[i].dstCoords[j].orientation.p = rpy[1];
      this->m_footStepNodesList_.data[i].dstCoords[j].orientation.y = rpy[2];
      this->m_footStepNodesList_.data[i].isSupportPhase[j] = gaitParam_.footstepNodesList[i].isSupportPhase[j];
      }
  }

  if(gaitParam_.footstepNodesList.size() > 1 &&
       (gaitParam_.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam_.footstepNodesList[1].isSupportPhase[LLEG]) &&
       ((gaitParam_.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam_.footstepNodesList[0].isSupportPhase[LLEG]) || (!gaitParam_.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam_.footstepNodesList[0].isSupportPhase[LLEG]))){
    this->m_footStepNodesList_.data[0].remainTime = gaitParam_.footstepNodesList[0].remainTime - timer.measure(); // 計算時間を引く．TODO footstepがこのRTCに届くまでの時間．そもそもremainTimeは必要か？
    this->m_footStepNodesListOut_.write();
  }
  return RTC::RTC_OK;
}

static const char* CollisionAvoidance_spec[] = {
    
  "implementation_id", "CollisionAvoidance",
  "type_name",         "CollisionAvoidance",
  "description",       "CollisionAvoidance component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
  void CollisionAvoidanceInit(RTC::Manager* manager) {
    RTC::Properties profile(CollisionAvoidance_spec);
    manager->registerFactory(profile, RTC::Create<CollisionAvoidance>, RTC::Delete<CollisionAvoidance>);
  }
};
