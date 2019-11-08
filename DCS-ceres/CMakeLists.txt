cmake_minimum_required(VERSION 2.8.3)

project(toy_pose_graph_robust_backend)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_CXX_FLAGS "-fpermissive" )

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


set( SRC
  main.cpp
  include/ceres_error.h     src/ceres_error.cpp
  include/g2o_util.h        src/g2o_util.cpp
  include/graph.h           src/graph.cpp
)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable( main ${SRC} )
target_link_libraries( main ${CERES_LIBRARIES})
