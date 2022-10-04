#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <cnoid/Body>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <iostream>
#include <unordered_map>
#include <moveit/distance_field/propagation_distance_field.h>


struct CollisionChecker{
public:
  class CollisionPair {
  public:
    cnoid::LinkPtr link1;
    cnoid::LinkPtr link2;
    double distance = 0;
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX();
    cnoid::Vector3 localp1 = cnoid::Vector3::Zero();
    cnoid::Vector3 localp2 = cnoid::Vector3::Zero();
  };

  class boundingBox {
    public:
    cnoid::Position localPose = cnoid::Position::Identity();
    cnoid::LinkPtr parentLink;
    cnoid::Vector3 dimensions = cnoid::Vector3::Zero();

    bool isInside(const cnoid::Vector3f& p) {
      cnoid::Vector3f plocal = worldPoseinv * p;
      return
        (plocal[0] < dimensions[0]/2) &&
        (plocal[1] < dimensions[1]/2) &&
        (plocal[2] < dimensions[2]/2) &&
        (plocal[0] > -dimensions[0]/2) &&
        (plocal[1] > -dimensions[1]/2) &&
        (plocal[2] > -dimensions[2]/2);
    }
    void setParentLinkPose(){
      if(parentLink){
        worldPoseinv = (parentLink->T() * localPose).inverse().cast<float>();
      }
    }
  private:
    Eigen::Affine3f worldPoseinv;
  };

  void checkSelfCollision(std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& collisionPairs, std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> >& vclipModelMap) const;

  void checkEnvCollision(const std::shared_ptr<distance_field::PropagationDistanceField> field, const cnoid::Position fieldOrigin, const std::vector<cnoid::LinkPtr>& targetLinks, std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> >& verticesMap, std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& o_collisionPairs);

  double maxDistance_ = 0.3;
  double minDistance_ = -0.02;
  std::vector<boundingBox > ignoreBoundingBox_;
};

#endif
