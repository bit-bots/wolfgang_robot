cmake_minimum_required(VERSION 2.8.3)
project(wolfgang_ik)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rotconv
  moveit_ros_planning
  bitbots_docs
)

find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp rotconv moveit_ros_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ik.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

catkin_add_gtest(test_ik tests/test_ik.cpp)
target_link_libraries(test_ik ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

enable_bitbots_docs()
