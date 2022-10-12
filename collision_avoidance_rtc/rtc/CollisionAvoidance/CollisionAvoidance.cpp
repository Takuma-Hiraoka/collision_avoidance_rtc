#include "CollisionAvoidance.h"

#include <cnoid/MeshExtractor>
#include <cnoid/BodyLoader>
#include <octomap_msgs/conversions.h>

static void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
  cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
  const cnoid::Affine3& T = meshExtractor->currentTransform();

  const int vertexIndexTop = model->getOrCreateVertices()->size();

  const cnoid::SgVertexArray& vertices = *mesh->vertices();
  const int numVertices = vertices.size();
  for(int i=0; i < numVertices; ++i){
    const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
    model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
  }

  const int numTriangles = mesh->numTriangles();
  for(int i=0; i < numTriangles; ++i){
    cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
    const int v0 = vertexIndexTop + tri[0];
    const int v1 = vertexIndexTop + tri[1];
    const int v2 = vertexIndexTop + tri[2];
    model->addTriangle(v0, v1, v2);
  }
}

static cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){
  if (!collisionshape) return nullptr;

  std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
  cnoid::SgMeshPtr model = new cnoid::SgMesh;
  if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    model->setName(collisionshape->name());
  }else{
    std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
    return nullptr;
  }

  return model;
}

CollisionAvoidance::CollisionAvoidance(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qIn_("qIn", m_q_),
  m_basePosIn_("basePosIn", m_basePos_),
  m_baseRpyIn_("baseRpyIn", m_baseRpy_),
  m_steppableRegionIn_("steppableRegionIn", m_steppableRegion_),
  m_refFootStepNodesListIn_("refFootStepNodesListIn", m_refFootStepNodesList_),
  m_comPredictParamIn_("comPredictParamIn", m_comPredictParam_),
  m_octomapIn_("octomapIn", m_octomap_),
  m_footStepNodesListOut_("footStepNodesListOut", m_footStepNodesList_),
  m_SequencePlayerServicePort_("SequencePlayerService")
{
}

RTC::ReturnCode_t CollisionAvoidance::onInitialize(){
  addInPort("qIn", this->m_qIn_);
  addInPort("basePosIn", this->m_basePosIn_);
  addInPort("baseRpyIn", this->m_baseRpyIn_);
  addInPort("steppableRegionIn", this->m_steppableRegionIn_);
  addInPort("refFootStepNodesListIn", this->m_refFootStepNodesListIn_);
  addInPort("comPredictParamIn", this->m_comPredictParamIn_);
  addInPort("octomapIn", this->m_octomapIn_);
  addOutPort("footStepNodesListOut", this->m_footStepNodesListOut_);
  this->m_SequencePlayerServicePort_.registerConsumer("service0", "SequencePlayerService", this->m_sequencePlayerService0_);
  addPort(m_SequencePlayerServicePort_);

  // load robot model
  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  
  {
    // load end_effector
    std::string endEffectors;
    if(this->getProperties().hasKey("end_effectors"))endEffectors = std::string(this->getProperties()["end_effectors"]);
    else endEffectors = std::string(this->m_pManager->getConfig()["end_effectors"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] end_effectors: " << endEffectors <<std::endl;
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->robot_->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      this->gaitParam_.push_backEE(name, parentLink, localT);
    }

    // 0番目が右脚. 1番目が左脚. という仮定がある.
    if(this->gaitParam_.eeName.size() < NUM_LEGS || this->gaitParam_.eeName[RLEG] != "rleg" || this->gaitParam_.eeName[LLEG] != "lleg"){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " this->gaitParam_.eeName.size() < 2 || this->gaitParams.eeName[0] != \"rleg\" || this->gaitParam_.eeName[1] != \"lleg\" not holds" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
  }

  // calculate convex hull for all collision mesh
  choreonoid_qhull::convertAllCollisionToConvexHull(this->robot_);
  // generate VClip model
  for(int i=0;i<this->robot_->numLinks();i++){
    std::shared_ptr<Vclip::Polyhedron> vclipmodel = choreonoid_vclip::convertToVClipModel(this->robot_->link(i)->collisionShape());
    this->vclipModelMap_[this->robot_->link(i)] = vclipmodel;
    if(!vclipmodel){
      std::cerr << "[" << m_profile.instance_name << "] " << "vclip model " << this->robot_->link(i)->name() << " failed" << std::endl;
    }
  }

  // collision_pair
  {
    std::string collision_pairProp;
    if(this->getProperties().hasKey("collision_pair")) collision_pairProp = std::string(this->getProperties()["collision_pair"]);
    else collision_pairProp = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] collision_pair: " << collision_pairProp <<std::endl;
    std::istringstream iss(collision_pairProp);
    std::string tmp;
    while (getline(iss, tmp, ' ')) {
      size_t pos = tmp.find_first_of(':');
      std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
      cnoid::LinkPtr link1 = this->robot_->link(name1), link2 = this->robot_->link(name2);
      if ( !link1 ) {
	std::cerr << "[" << this->m_profile.instance_name << "] Could not find robot link " << name1 << std::endl;
	continue;
      }
      if ( !link2 ) {
	std::cerr << "[" << this->m_profile.instance_name << "] Could not find robot link " << name2 << std::endl;
	continue;
      }
      std::cerr << "[" << this->m_profile.instance_name << "] check collisions between " << link1->name() << " and " <<  link2->name() << std::endl;
      std::shared_ptr<CollisionChecker::CollisionPair> pair = std::make_shared<CollisionChecker::CollisionPair>();
      pair->link1 = link1;
      pair->link2 = link2;
      this->selfCollisionPairs_.push_back(pair);
    }
  }

  // env collision mesh
  {
    // get link vertices
    float resolution = 0.01;
    for(int i=0;i<robot_->numLinks();i++){
      cnoid::LinkPtr link = robot_->link(i);
      std::vector<cnoid::Vector3f> vertices; // 同じvertexが2回カウントされている TODO
      cnoid::SgMeshPtr mesh = convertToSgMesh(link->collisionShape());
      if(mesh) {
	mesh->updateBoundingBox();
	cnoid::BoundingBoxf bbx = mesh->boundingBox();
	cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
	std::vector<std::vector<std::vector<bool> > > bin(int(bbxSize[0]/resolution)+1,
							  std::vector<std::vector<bool> >(int(bbxSize[1]/resolution)+1,
											  std::vector<bool>(int(bbxSize[2]/resolution)+1,
													    false)));

	for(int j=0;j<mesh->numTriangles();j++){
	  cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
	  cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
	  cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
	  float l1 = (v1 - v0).norm();
	  float l2 = (v2 - v0).norm();
	  cnoid::Vector3f n1 = (v1 - v0).normalized();
	  cnoid::Vector3f n2 = (v2 - v0).normalized();
	  for(double m=0;m<l1;m+=resolution){
	    for(double n=0;n<l2-l2/l1*m;n+=resolution){
	      cnoid::Vector3f v = v0 + n1 * m + n2 * n;
	      int x = int((v[0] - bbx.min()[0])/resolution);
	      int y = int((v[1] - bbx.min()[1])/resolution);
	      int z = int((v[2] - bbx.min()[2])/resolution);
	      if(!bin[x][y][z]){
		bin[x][y][z] = true;
		vertices.push_back(v);
	      }
	    }
	    double n=l2-l2/l1*m;
	    cnoid::Vector3f v = v0 + n1 * m + n2 * n;
	    int x = int((v[0] - bbx.min()[0])/resolution);
	    int y = int((v[1] - bbx.min()[1])/resolution);
	    int z = int((v[2] - bbx.min()[2])/resolution);
	    if(!bin[x][y][z]){
	      bin[x][y][z] = true;
	      vertices.push_back(v);
	    }
	  }
	  double m = l1;
	  double n= 0;
	  cnoid::Vector3f v = v0 + n1 * m + n2 * n;
	  int x = int((v[0] - bbx.min()[0])/resolution);
	  int y = int((v[1] - bbx.min()[1])/resolution);
	  int z = int((v[2] - bbx.min()[2])/resolution);
	  if(!bin[x][y][z]){
	    bin[x][y][z] = true;
	    vertices.push_back(v);
	  }
	}
      }
      verticesMap_[link] = vertices;
    }

    for(int i=0;i<robot_->numLinks();i++){
      cnoid::LinkPtr link = robot_->link(i);
      if(verticesMap_[link].size() == 0) continue;
      targetLinks_.push_back(link);
    }

  }

  this->iksolver_.init(this->robot_, this->gaitParam_, this->selfCollisionPairs_);
  gaitParam_.orgRobot = this->robot_;
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionAvoidance::onExecute(RTC::UniqueId ec_id){
  std::cerr << "Collision Avoidance rtc onExecute" << std::endl;
  cnoid::TimeMeasure timer;
  timer.begin();
  // TODO 本来はserviceにするべき？

  // read port
  {
    if (this->m_qIn_.isNew()) this->m_qIn_.read();
    if (this->m_basePosIn_.isNew()) this->m_basePosIn_.read();
    if (this->m_baseRpyIn_.isNew()) this->m_baseRpyIn_.read();

    if(this->m_q_.data.length() == this->robot_->numJoints()){
      for ( int i = 0; i < this->robot_->numJoints(); i++ ){
	this->robot_->joint(i)->q() = this->m_q_.data[i];
	gaitParam_.orgRobot->joint(i)->q() = this->m_q_.data[i];
      }
    }
    this->robot_->rootLink()->p()[0] = m_basePos_.data.x;
    this->robot_->rootLink()->p()[1] = m_basePos_.data.y;
    this->robot_->rootLink()->p()[2] = m_basePos_.data.z;
    this->robot_->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy_.data.r, m_baseRpy_.data.p, m_baseRpy_.data.y);
    this->robot_->calcForwardKinematics();

    gaitParam_.orgRobot->rootLink()->p()[0] = m_basePos_.data.x;
    gaitParam_.orgRobot->rootLink()->p()[1] = m_basePos_.data.y;
    gaitParam_.orgRobot->rootLink()->p()[2] = m_basePos_.data.z;
    gaitParam_.orgRobot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy_.data.r, m_baseRpy_.data.p, m_baseRpy_.data.y);
    std::cerr << "act rpy r : " << m_baseRpy_.data.r << " p : " << m_baseRpy_.data.p << " y : " << m_baseRpy_.data.y << std::endl;
  
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
  
    if(this->m_refFootStepNodesListIn_.isNew()){
      m_refFootStepNodesListIn_.read();
      gaitParam_.footstepNodesList.resize(m_refFootStepNodesList_.data.length());
      for(int i=0;i<gaitParam_.footstepNodesList.size();i++) {
	for(int j=0;j<NUM_LEGS;j++){
	  gaitParam_.footstepNodesList[i].dstCoords[j].translation()[0] = m_refFootStepNodesList_.data[i].dstCoords[j].position.x;
	  gaitParam_.footstepNodesList[i].dstCoords[j].translation()[1] = m_refFootStepNodesList_.data[i].dstCoords[j].position.y;
	  gaitParam_.footstepNodesList[i].dstCoords[j].translation()[2] = m_refFootStepNodesList_.data[i].dstCoords[j].position.z;
	  gaitParam_.footstepNodesList[i].dstCoords[j].linear() = cnoid::rotFromRpy(m_refFootStepNodesList_.data[i].dstCoords[j].orientation.r, m_refFootStepNodesList_.data[i].dstCoords[j].orientation.p, m_refFootStepNodesList_.data[i].dstCoords[j].orientation.y);
	  gaitParam_.footstepNodesList[i].isSupportPhase[j] = m_refFootStepNodesList_.data[i].isSupportPhase[j];
	}
	gaitParam_.footstepNodesList[i].remainTime = m_refFootStepNodesList_.data[i].remainTime;
      }
    }

    if(this->m_comPredictParamIn_.isNew()){
      m_comPredictParamIn_.read();
      gaitParam_.curZmp[0] = m_comPredictParam_.curZmp.x;
      gaitParam_.curZmp[1] = m_comPredictParam_.curZmp.y;
      gaitParam_.curZmp[2] = m_comPredictParam_.curZmp.z;
      gaitParam_.genCog[0] = m_comPredictParam_.curCog.x;
      gaitParam_.genCog[1] = m_comPredictParam_.curCog.y;
      gaitParam_.genCog[2] = m_comPredictParam_.curCog.z;
      gaitParam_.genCogVel[0] = m_comPredictParam_.curCogVel.x;
      gaitParam_.genCogVel[1] = m_comPredictParam_.curCogVel.y;
      gaitParam_.genCogVel[2] = m_comPredictParam_.curCogVel.z;
      gaitParam_.omega = m_comPredictParam_.omega;
      gaitParam_.l[0] = m_comPredictParam_.l.x;
      gaitParam_.l[1] = m_comPredictParam_.l.y;
      gaitParam_.l[2] = m_comPredictParam_.l.z;
      gaitParam_.dt = m_comPredictParam_.dt;
    }

    if (this->thread_done_ && this->thread_){
      this->thread_->join();
      this->thread_ = nullptr;
    }
    if (this->m_octomapIn_.isNew()) {
      this->m_octomapIn_.read();
      std::shared_ptr<octomap_msgs::Octomap> octomap = std::make_shared<octomap_msgs::Octomap>();
      octomap->binary = this->m_octomap_.data.octomap.binary;
      octomap->id = this->m_octomap_.data.octomap.id;
      octomap->resolution = this->m_octomap_.data.octomap.resolution;
      octomap->data.resize(this->m_octomap_.data.octomap.data.length());
      for(int i=0;i<octomap->data.size();i++) {
	octomap->data[i] = this->m_octomap_.data.octomap.data[i];
      }
      cnoid::Position fieldOrigin;
      fieldOrigin.translation()[0] = this->m_octomap_.data.origin.position.x;
      fieldOrigin.translation()[1] = this->m_octomap_.data.origin.position.y;
      fieldOrigin.translation()[2] = this->m_octomap_.data.origin.position.z;
      fieldOrigin.linear() = cnoid::rotFromRpy(this->m_octomap_.data.origin.orientation.r,this->m_octomap_.data.origin.orientation.p,this->m_octomap_.data.origin.orientation.y);
      if ( !this->thread_) {
	this->thread_done_ = false;
	this->thread_ = std::make_shared<std::thread>(&CollisionAvoidance::octomapCallback, this, octomap, fieldOrigin);
      }
    }
  } // read port

  bool init_flag = false;
  if (avoidancePlanner_.checkPlanExecute(gaitParam_.footstepNodesList)){    
    // 着地可能な領域を計算
    avoidancePlanner_.calcSafeHulls(gaitParam_.footstepNodesList, gaitParam_.steppable_region, gaitParam_.steppable_height, avoidancePlanner_.steppableHulls, avoidancePlanner_.steppableHeights, avoidancePlanner_.safeHulls);

    for (int i=0;i<this->iter_time;i++){
      // footstepを着地可能な領域になおす
      avoidancePlanner_.updateSafeFootStep(gaitParam_.footstepNodesList, avoidancePlanner_.steppableHulls, avoidancePlanner_.steppableHeights, avoidancePlanner_.safeHulls);

      comCoordsGenerator_.calcZmpTrajectory(gaitParam_, gaitParam_.refZmpTraj);
      comCoordsGenerator_.calcComCoords(gaitParam_, gaitParam_.tgtCog);
      
      for(int j=0;j<NUM_LEGS;j++) {
	gaitParam_.eeTargetPose[j] = gaitParam_.footstepNodesList[0].dstCoords[j];
      }
      if(!init_flag){
	init_flag = true;
	// ただ逆運動学を解くだけでは重心移動分を腕などによって補償してしまうが、実際は腰が十分動いているため正確な予測にはならない。このためとりあえず重心移動分ルートリンクを移動させる。
	this->robot_->rootLink()->p() += gaitParam_.tgtCog - gaitParam_.genCog;
	this->robot_->calcForwardKinematics();
	std::vector<std::shared_ptr<CollisionChecker::CollisionPair> > nan; // はじめは干渉を考えない
	iksolver_.solveFullBodyIK(gaitParam_.dt, gaitParam_, nan, nan, robot_, 1);
      }
      if(this->field_){
	this->robot_->calcForwardKinematics();
	collisionChecker_.checkEnvCollision(this->field_, this->fieldOrigin_, this->targetLinks_, this->verticesMap_, this->envCollisionPairs_);
      }
      collisionChecker_.checkSelfCollision(this->selfCollisionPairs_, this->vclipModelMap_);
      iksolver_.solveFullBodyIK(gaitParam_.dt, gaitParam_, this->selfCollisionPairs_, this->envCollisionPairs_, robot_, 2);
    }

     std::cerr << "execution time : " << timer.measure() << std::endl;
     std::cerr << "out joint angle :";
     for ( int i = 0; i < this->robot_->numJoints(); i++ ){
       std::cerr << " " << this->robot_->joint(i)->q() * 180 / M_PI; // for euslisp debug
     }
    std::cerr << std::endl;
  }
  
  // write port
  {
    this->m_footStepNodesList_.data.length(gaitParam_.footstepNodesList.size());
    for(int i=0;i<this->m_footStepNodesList_.data.length();i++) {
      this->m_footStepNodesList_.data[i].dstCoords.length(NUM_LEGS);
      this->m_footStepNodesList_.data[i].isSupportPhase.length(NUM_LEGS);
      for(int j=0;j<NUM_LEGS;j++){
	this->m_footStepNodesList_.data[i].dstCoords[j].position.x = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[0];
	this->m_footStepNodesList_.data[i].dstCoords[j].position.y = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[1];
	this->m_footStepNodesList_.data[i].dstCoords[j].position.z = gaitParam_.footstepNodesList[i].dstCoords[j].translation()[2];
	cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam_.footstepNodesList[i].dstCoords[j].linear());
	this->m_footStepNodesList_.data[i].dstCoords[j].orientation.r = rpy[0];
	this->m_footStepNodesList_.data[i].dstCoords[j].orientation.p = rpy[1];
	this->m_footStepNodesList_.data[i].dstCoords[j].orientation.y = rpy[2];
	this->m_footStepNodesList_.data[i].isSupportPhase[j] = gaitParam_.footstepNodesList[i].isSupportPhase[j]; // TODO remainTimeは必要か？
      }
    }
    this->m_footStepNodesListOut_.write();

    if (avoidancePlanner_.checkPlanExecute(gaitParam_.footstepNodesList)){    
      if(!CORBA::is_nil(this->m_sequencePlayerService0_._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
	 !this->m_sequencePlayerService0_->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
	OpenHRP::dSequence angles;
	angles.length(this->robot_->numJoints());
	for ( int i = 0; i < this->robot_->numJoints(); i++ ){
	  angles[i] = this->robot_->joint(i)->q(); 
	}
	this->m_sequencePlayerService0_->setJointAngles(angles, gaitParam_.footstepNodesList[0].remainTime);
      }
    }
  } // write port
  
  return RTC::RTC_OK;
}

void CollisionAvoidance::octomapCallback(std::shared_ptr<octomap_msgs::Octomap> octomap, cnoid::Position fieldOrigin){

  std::shared_ptr<octomap::AbstractOcTree> absoctree = std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(*octomap));
  if(!absoctree) {
    this->thread_done_ = true;
    return;
  }

  std::shared_ptr<octomap::OcTree> octree = std::dynamic_pointer_cast<octomap::OcTree>(absoctree);

  if(!octree){
    std::shared_ptr<octomap::ColorOcTree> coloroctree = std::dynamic_pointer_cast<octomap::ColorOcTree>(absoctree);
    if(coloroctree){
      std::stringstream ss;
      coloroctree->writeBinary(ss);
      octree = std::make_shared<octomap::OcTree>(absoctree->getResolution());
      if(!octree->readBinary(ss)) octree = nullptr;
    }
  }

  if(octree){
    double minx,miny,minz; octree->getMetricMin(minx,miny,minz);
    double maxx,maxy,maxz; octree->getMetricMax(maxx,maxy,maxz);
    this->field_ = std::make_shared<distance_field::PropagationDistanceField>(*octree,
                                                                              octomap::point3d(minx,miny,minz),
                                                                              octomap::point3d(maxx,maxy,maxz),
                                                                              collisionChecker_.maxDistance_,
                                                                              true // true: めり込み時に離れる方向を示す. 裏側に行かないよう、minDistanceをある程度大きくせよ
                                                                              );
    this->fieldOrigin_ = fieldOrigin;
    octree->clear(); // destructor of OcTree does not free memory for internal data.
  }else{
    this->field_ = nullptr;
    this->fieldOrigin_ = cnoid::Position::Identity();
  }

  absoctree->clear(); // destructor of OcTree does not free memory for internal data.
  this->thread_done_ = true;
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
