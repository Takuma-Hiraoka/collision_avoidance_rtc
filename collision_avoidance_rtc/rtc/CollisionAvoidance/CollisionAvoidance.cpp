#include "CollisionAvoidance.h"

CollisionAvoidance::CollisionAvoidance(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_steppableRegionIn_("steppableRegionIn", m_steppableRegion_),
  m_footStepNodesListIn_("footStepNodesListIn", m_footStepNodesList_)
{
}

RTC::ReturnCode_t CollisionAvoidance::onInitialize(){
  addInPort("steppableRegionIn", m_steppableRegionIn_);
  addInPort("footStepNodesListIn", m_footStepNodesListIn_);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionAvoidance::onExecute(RTC::UniqueId ec_id){
  std::cerr << "Collision Avoidance rtc onExecute" << std::endl;
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
  
  if(this->m_footStepNodesListIn_.isNew()){
    m_footStepNodesListIn_.read();
    gaitParam_.footstepNodesList.resize(m_footStepNodesList_.data.length());
    for(int i=0;i<gaitParam_.footstepNodesList.size();i++) {
      for(int j=0;j<NUM_LEGS;j++){
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[0] = m_footStepNodesList_.data[i].dstCoords[j].position.x;
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[1] = m_footStepNodesList_.data[i].dstCoords[j].position.y;
	gaitParam_.footstepNodesList[i].dstCoords[j].translation()[2] = m_footStepNodesList_.data[i].dstCoords[j].position.z;
	gaitParam_.footstepNodesList[i].dstCoords[j].linear() = cnoid::rotFromRpy(m_footStepNodesList_.data[i].dstCoords[j].orientation.r, m_footStepNodesList_.data[i].dstCoords[j].orientation.p, m_footStepNodesList_.data[i].dstCoords[j].orientation.y);
	gaitParam_.footstepNodesList[i].isSupportPhase[j] = m_footStepNodesList_.data[i].isSupportPhase[j];
      }
      gaitParam_.footstepNodesList[i].remainTime = m_footStepNodesList_.data[i].remainTime;
    }
  }

  avoidancePlanner_.calcSafeHulls(gaitParam_.footstepNodesList, gaitParam_.steppable_region, gaitParam_.steppable_height, avoidancePlanner_.steppableHulls, avoidancePlanner_.steppableHeights, avoidancePlanner_.safeHulls);
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
