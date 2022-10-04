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
