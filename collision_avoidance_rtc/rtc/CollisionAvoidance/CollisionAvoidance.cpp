#include "CollisionAvoidance.h"

CollisionAvoidance::CollisionAvoidance(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t CollisionAvoidance::onInitialize(){
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionAvoidance::onExecute(RTC::UniqueId ec_id){
  std::cerr << "Collision Avoidance rtc onExecute" << std::endl;
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
