cmake_minimum_required(VERSION 2.8.3)
project(driving_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  driving_simulator_msgs
)

find_package(Despot CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driving_simulator
  CATKIN_DEPENDS message_runtime driving_simulator_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include/
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


add_executable(MultipolicyMPC src/FORCESNLPsolver_casadi2forces.c
src/FORCESNLPsolver_model_1.c
src/FORCESNLPsolver_model_50.c
        src/Clothoid.cpp
        src/spline.cpp
src/MPC.cpp
)

add_executable(DESPOT
src/Clothoid.cpp
src/spline.cpp
src/POMDPrviz.cpp
src/OnlinePOMDP.cpp)

add_executable(DESPOT1
        src/Clothoid.cpp
        src/spline.cpp
        src/POMDPrviz1.cpp
src/OnlinePOMDP1.cpp)


add_executable(DESPOT2
        src/Clothoid.cpp
        src/spline.cpp
        src/POMDPrviz2.cpp
src/OnlinePOMDP2.cpp)


add_executable(DESPOT3
        src/Clothoid.cpp
        src/spline.cpp
        src/POMDPrviz3.cpp
 src/OnlinePOMDP3.cpp)

target_link_libraries(MultipolicyMPC
${catkin_LIBRARIES}
/home/bdebrito/catkin_ws/src/driving_simulator/src/libFORCESNLPsolver.so)
#
target_link_libraries(DESPOT
${catkin_LIBRARIES}
despot)

target_link_libraries(DESPOT1
${catkin_LIBRARIES}
despot)

target_link_libraries(DESPOT2
${catkin_LIBRARIES}
despot)

target_link_libraries(DESPOT3
${catkin_LIBRARIES}
despot)

add_library(DriverModel
        src/Clothoid.cpp
        src/spline.cpp
        src/DriverModel.cpp)

add_dependencies(DriverModel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(DriverModel ${catkin_LIBRARIES})

add_executable(DriverModel_node src/driver_node.cpp)
add_dependencies(DriverModel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(DriverModel_node DriverModel ${catkin_LIBRARIES} )

### INSTALL ###
install(TARGETS DriverModel_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )