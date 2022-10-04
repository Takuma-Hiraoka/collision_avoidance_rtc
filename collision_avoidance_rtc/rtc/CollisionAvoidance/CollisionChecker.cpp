#include "CollisionChecker.h"

void CollisionChecker::checkSelfCollision(std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& collisionPairs, std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> >& vclipModelMap) const{
  // それぞれのcollisionPairsについて最近傍点を計算する．
    for(size_t i=0;i<collisionPairs.size(); i++){
    std::shared_ptr<CollisionPair>& pair = collisionPairs[i];
    cnoid::Vector3 localp1, localp2;
    double distance;
    if(!choreonoid_vclip::computeDistance(vclipModelMap[pair->link1],
                                          pair->link1->p(),
                                          pair->link1->R(),
                                          vclipModelMap[pair->link2],
                                          pair->link2->p(),
                                          pair->link2->R(),
                                          distance,
                                          localp1,
                                          localp2
                                          )){
      std::cerr << "distance between " << pair->link1->name() << " and " << pair->link2->name() <<  " failed" << std::endl;
    }
    if(distance > 1e-6){
      pair->distance = distance;
      pair->direction21 = (pair->link1->T()*localp1 - pair->link2->T()*localp2).normalized();
      pair->localp1 = localp1;
      pair->localp2 = localp2;
    }else{
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      pair->distance = distance;
    }
  }
}

void CollisionChecker::checkEnvCollision(const std::shared_ptr<distance_field::PropagationDistanceField> field, const cnoid::Position fieldOrigin, const std::vector<cnoid::LinkPtr>& targetLinks, std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> >& verticesMap, std::vector<std::shared_ptr<CollisionChecker::CollisionPair>>& o_collisionPairs) {
  
  Eigen::Affine3f fieldOriginInv = fieldOrigin.inverse().cast<float>();
  std::vector<std::shared_ptr<CollisionChecker::CollisionPair>> next_collisionPairs;
  // update ignore bounding box
  for(int i=0;i<this->ignoreBoundingBox_.size();i++) this->ignoreBoundingBox_[i].setParentLinkPose();

  for(int i=0;i<targetLinks.size();i++){
    cnoid::LinkPtr link = targetLinks[i];
    Eigen::Affine3f linkT = link->T().cast<float>();

    double min_dist = this->maxDistance_ + 1;
    cnoid::Vector3f closest_v = cnoid::Vector3f::Zero();
    cnoid::Vector3 closest_point_fieldLocal = cnoid::Vector3::Zero();
    cnoid::Vector3 closest_direction_fieldLocal = cnoid::Vector3::UnitX();

    const std::vector<cnoid::Vector3f>& vertices = verticesMap[link];
    for(int j=0;j<vertices.size();j++){
      cnoid::Vector3f v = linkT * vertices[j];

      bool ignore = false;
      for(int k=0;k<this->ignoreBoundingBox_.size();k++){
	if(this->ignoreBoundingBox_[k].isInside(v)) {
	  ignore = true;
	  break;
	}
      }
      if(ignore) continue;

      cnoid::Vector3f v_fieldLocal = fieldOriginInv * v;

      cnoid::Vector3 grad;
      bool in_bound;
      double dist = field->getDistanceGradient(v_fieldLocal[0],v_fieldLocal[1],v_fieldLocal[2],grad[0],grad[1],grad[2],in_bound);
      if(in_bound && grad.norm() > 0){
	if(dist < min_dist){
	  closest_direction_fieldLocal[0] = (grad[0]/grad.norm());
	  closest_direction_fieldLocal[1] = (grad[1]/grad.norm());
	  closest_direction_fieldLocal[2] = (grad[2]/grad.norm());
	  closest_point_fieldLocal[0] = v_fieldLocal[0]-closest_direction_fieldLocal[0]*dist;
	  closest_point_fieldLocal[1] = v_fieldLocal[1]-closest_direction_fieldLocal[1]*dist;
	  closest_point_fieldLocal[2] = v_fieldLocal[2]-closest_direction_fieldLocal[2]*dist;
	  min_dist = dist;
	  closest_v = vertices[j];
	}
      }
    }

    if(min_dist <= this->maxDistance_ && min_dist >= this->minDistance_){
      cnoid::Vector3 closest_point = fieldOrigin * closest_point_fieldLocal;
      cnoid::Vector3 closest_direction = fieldOrigin.linear() * closest_direction_fieldLocal;

      std::shared_ptr<CollisionChecker::CollisionPair> collisionPair = std::make_shared<CollisionChecker::CollisionPair>();
      collisionPair->link1 = link;
      collisionPair->localp1 = closest_v.cast<double>();
      collisionPair->link2 = nullptr;
      collisionPair->localp2 = closest_point;
      collisionPair->direction21 = closest_direction;
      collisionPair->distance = min_dist;
      next_collisionPairs.push_back(collisionPair);
    }
  }
  o_collisionPairs = next_collisionPairs;
}
