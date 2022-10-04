#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <cnoid/Body>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <iostream>
#include <unordered_map>

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

  void checkSelfCollision(std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& collisionPairs, std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> >& vclipModelMap) const;
};

#endif
