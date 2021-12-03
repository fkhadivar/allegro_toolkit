cmake_minimum_required(VERSION 2.8.3)

project(allegro_toolkit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  # biotac_ros
  # utils
  gazebo_ros
)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
find_package(Eigen3 REQUIRED eigen3) 
find_package(RBDL REQUIRED URDFREADER)
find_package(qpOASES QUIET)


# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_generation #biotac_ros
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR} )
include_directories(${qpOASES_INCLUDE_DIRS})


add_executable(ds_force src/ds_force_node.cpp src/robot/hand_robot.cpp src/control/ds_force_control.cpp src/control/qp_solver.cpp src/control/control_util.cpp src/third_party/Utils.cpp)
target_include_directories(ds_force PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})
target_link_libraries(ds_force ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} ${qpOASES_LIBRARIES})
add_dependencies(ds_force ${PROJECT_NAME}_gencpp)

add_executable(qp_control src/qp_control_node.cpp src/robot/hand_robot.cpp src/control/qp_based_control.cpp src/control/qp_solver.cpp src/control/control_util.cpp src/third_party/Utils.cpp)
target_include_directories(qp_control PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})
target_link_libraries(qp_control ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} ${qpOASES_LIBRARIES})
add_dependencies(qp_control ${PROJECT_NAME}_gencpp)

add_executable(ad_control src/adaptive_control_node.cpp src/robot/hand_robot.cpp src/control/adaptive_control.cpp src/control/qp_solver.cpp src/control/control_util.cpp src/third_party/Utils.cpp)
target_include_directories(ad_control PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})
target_link_libraries(ad_control ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} ${qpOASES_LIBRARIES})
add_dependencies(ad_control ${PROJECT_NAME}_gencpp)

add_executable(hand_ft_controller src/hand_ft_controller_node.cpp src/robot/hand_robot.cpp src/control/hand_ft_controller.cpp src/control/qp_solver.cpp src/control/control_util.cpp src/third_party/Utils.cpp)
target_include_directories(hand_ft_controller PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})
target_link_libraries(hand_ft_controller ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} ${qpOASES_LIBRARIES})
add_dependencies(hand_ft_controller ${PROJECT_NAME}_gencpp)


add_executable(hand_torque_sim src/hand_torque_sim_node.cpp src/robot/hand_robot.cpp src/control/hand_ft_controller.cpp src/control/qp_solver.cpp src/third_party/sg_filter.cpp src/control/control_util.cpp src/third_party/Utils.cpp)
target_include_directories(hand_torque_sim PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS})
target_link_libraries(hand_torque_sim ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} ${qpOASES_LIBRARIES})
add_dependencies(hand_torque_sim ${PROJECT_NAME}_gencpp)

FILE(GLOB SRCS src/control/adaptive_control.cpp src/control/qp_solver.cpp src/control/control_util.cpp src/robot/hand_robot.cpp src/motion_capture.cpp src/third_party/Utils.cpp)
FILE(GLOB INCS include/control/adaptive_control.h include/control/qp_solver.hpp include/control/control_util.h include/robot/hand_robot.hpp include/robot/abs_robot.hpp include/control/abs_control.hpp src/motion_capture.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
