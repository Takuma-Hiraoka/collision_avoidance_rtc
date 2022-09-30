#ifndef CollisionAvoidance_H
#define CollisionAvoidance_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

class CollisionAvoidance : public RTC::DataFlowComponentBase{
protected:

public:
  CollisionAvoidance(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void CollisionAvoidanceInit(RTC::Manager* manager);
}

#endif // CollisionAvoidence_H
