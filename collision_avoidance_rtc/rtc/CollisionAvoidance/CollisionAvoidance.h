#ifndef CollisionAvoidance_H
#define CollisionAvoidance_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <thread>
#include <mutex>
#include <cnoid/EigenTypes>
#include <cnoid/TimeMeasure>
#include <cnoid/Body>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <auto_stabilizer_msgs/idl/AutoStabilizer.hh>
#include <octomap_msgs_rtmros_bridge/idl/Octomap.hh>
#include <moveit/distance_field/propagation_distance_field.h>
#include <octomap_msgs/Octomap.h>
#include "GaitParam.h"
#include "AvoidancePlanner.h"
#include "ComCoordsGenerator.h"
#include "PrioritizedIKSolver.h"
#include "CollisionChecker.h"

class CollisionAvoidance : public RTC::DataFlowComponentBase{
protected:

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedPoint3D m_basePos_;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn_;
  RTC::TimedOrientation3D m_baseRpy_;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn_;
  auto_stabilizer_msgs::TimedSteppableRegion m_steppableRegion_;
  RTC::InPort <auto_stabilizer_msgs::TimedSteppableRegion> m_steppableRegionIn_;
  auto_stabilizer_msgs::TimedFootStepNodesList m_refFootStepNodesList_;
  RTC::InPort <auto_stabilizer_msgs::TimedFootStepNodesList> m_refFootStepNodesListIn_;
  auto_stabilizer_msgs::ComPredictParam m_comPredictParam_;
  RTC::InPort<auto_stabilizer_msgs::ComPredictParam> m_comPredictParamIn_;
  octomap_msgs_rtmros_bridge::TimedOctomapWithPose m_octomap_;
  RTC::InPort<octomap_msgs_rtmros_bridge::TimedOctomapWithPose> m_octomapIn_;

  auto_stabilizer_msgs::TimedFootStepNodesList m_footStepNodesList_;
  RTC::OutPort <auto_stabilizer_msgs::TimedFootStepNodesList> m_footStepNodesListOut_;

public:
  CollisionAvoidance(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void octomapCallback(std::shared_ptr<octomap_msgs::Octomap> octomap, cnoid::Position fieldOrigin);

  GaitParam gaitParam_;
  AvoidancePlanner avoidancePlanner_;
  ComCoordsGenerator comCoordsGenerator_;
  PrioritizedIKSolver iksolver_;
  CollisionChecker collisionChecker_;

  double iter_time = 1;

private:
  cnoid::BodyPtr robot_;
  
  std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > vclipModelMap_;
  std::vector<std::shared_ptr<CollisionChecker::CollisionPair> > selfCollisionPairs_;
  std::vector<std::shared_ptr<CollisionChecker::CollisionPair> > envCollisionPairs_;

  std::shared_ptr<std::thread> thread_;
  bool thread_done_ = true;

  std::shared_ptr<distance_field::PropagationDistanceField> field_;
  cnoid::Position fieldOrigin_ = cnoid::Position::Identity();
  std::vector<cnoid::LinkPtr> targetLinks_;
  std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> > verticesMap_;

};

extern "C"
{
  void CollisionAvoidanceInit(RTC::Manager* manager);
}

#endif // CollisionAvoidence_H
