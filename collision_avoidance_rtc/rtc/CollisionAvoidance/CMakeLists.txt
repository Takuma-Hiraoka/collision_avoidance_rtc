add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  auto_stabilizer_msgs
  ik_constraint
  prioritized_inverse_kinematics_solver
  choreonoid_vclip
  choreonoid_qhull
  octomap_msgs_rtmros_bridge
  octomap_msgs
  )
find_package(choreonoid REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  $<TARGET_PROPERTY:OsqpEigen::OsqpEigen,INTERFACE_INCLUDE_DIRECTORIES>
  )

# 相対パスを絶対パスに直す                                                                                                                                                                                  
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(CollisionAvoidance SHARED
  CollisionAvoidance.cpp
  AvoidancePlanner.cpp
  MathUtil.cpp
  ComCoordsGenerator.cpp
  PrioritizedIKSolver.cpp
  CollisionChecker.cpp
  )
target_link_libraries(CollisionAvoidance
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )
set_target_properties(CollisionAvoidance PROPERTIES PREFIX "") # libCollisionAvoidance.so -> CollisionAvoidance.so                                                                           
add_dependencies(CollisionAvoidance RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl                                                                                                        

rtmbuild_add_executable(CollisionAvoidanceComp CollisionAvoidanceComp.cpp)
target_link_libraries(CollisionAvoidanceComp CollisionAvoidance)
