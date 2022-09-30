#ifndef CollisionAvoidance_H
#define CollisionAvoidance_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <cnoid/EigenTypes>
#include "auto_stabilizer_msgs/idl/AutoStabilizer.hh"
#include "GaitParam.h"
#include "AvoidancePlanner.h"

class CollisionAvoidance : public RTC::DataFlowComponentBase{
protected:

  auto_stabilizer_msgs::TimedSteppableRegion m_steppableRegion_;
  RTC::InPort <auto_stabilizer_msgs::TimedSteppableRegion> m_steppableRegionIn_;
  auto_stabilizer_msgs::TimedFootStepNodesList m_footStepNodesList_;
  RTC::InPort <auto_stabilizer_msgs::TimedFootStepNodesList> m_footStepNodesListIn_;

public:
  CollisionAvoidance(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  GaitParam gaitParam_;
  AvoidancePlanner avoidancePlanner_;
};

extern "C"
{
  void CollisionAvoidanceInit(RTC::Manager* manager);
}

#endif // CollisionAvoidence_H
