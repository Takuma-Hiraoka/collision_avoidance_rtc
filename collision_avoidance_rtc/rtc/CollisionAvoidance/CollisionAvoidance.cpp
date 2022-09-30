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
