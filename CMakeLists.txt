cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(slam_backend)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(GTSAM REQUIRED)
include_directories(include ${GTSAM_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
add_definitions(-std=c++11)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES localization
 CATKIN_DEPENDS roscpp 
 DEPENDS gtsam
)

add_library (localization src/localization.cpp)
add_dependencies(localization
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries (localization gtsam ${catkin_LIBRARIES} ${Boost_LIBRARIES})
