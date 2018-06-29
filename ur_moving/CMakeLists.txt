cmake_minimum_required(VERSION 2.8.3)
project(ur3_moving)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    message_generation
)



find_package(Boost REQUIRED system filesystem date_time thread)



add_message_files(
  FILES
  singleJoint.msg
  allJoints.msg
  endEffector.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  LIBRARIES
    interactivity_utils
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    message_runtime	
)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(ur_setter src/ur_setter.cpp)
target_link_libraries(ur_setter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur_setter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(ur_getter src/ur_getter.cpp)
target_link_libraries(ur_getter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur_getter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

