cmake_minimum_required(VERSION 3.0.2)
project(pp_env_model)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -Wall -g -Wno-sign-compare -Wno-unknown-pragmas -Wno-reorder")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -werror -Wno-sign-compare -Wno-unknown-pragmas -Wno-reorder")


add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pp_common
  hdmap_processing
  derived_object_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  roscpp
  roslib
  message_generation
  path_planning_msgs
  common_msgs
  geometry_msgs
  tf
  lanelet2_extension
  geographic_msgs
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BP_LIB
  CATKIN_DEPENDS   
    pp_common
    hdmap_processing
    derived_object_msgs
    lanelet2_core
    lanelet2_io
    lanelet2_projection
    roscpp
    roslib
    message_generation
    path_planning_msgs
    common_msgs
    geometry_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(BP_LIB
  src/behavioral/ego_state_machine.cpp
  src/behavioral/ego_state.cpp
  src/behavioral/ego_trigger.cpp
  src/behavioral/ego_trigger_factory.cpp
  src/behavioral/HDM_interface.cpp
  src/behavioral/global_command_trigger.cpp
  src/behavioral/global_command_trigger_factory.cpp
  src/behavioral/global_command_state.cpp
  src/behavioral/global_command_state_machine.cpp
  src/behavioral/abstract_state.cpp
  src/behavioral/abstract_state_machine.cpp
  src/behavioral/abstract_trigger_factory.cpp
  src/behavioral/abstract_transition_graph.cpp
  src/behavioral/abstract_trigger.cpp
  src/behavioral/traffic_light_trigger.cpp
  src/behavioral/traffic_light_state.cpp
  src/behavioral/traffic_light_state_machine.cpp
  src/behavioral/traffic_light_trigger_factory.cpp
  src/behavioral/world_state_manager.cpp
  src/behavioral/stop_result.cpp
  src/behavioral/stop_sign_state.cpp
  src/behavioral/stop_sign_trigger.cpp
  src/behavioral/stop_sign_state_machine.cpp
  src/behavioral/stop_sign_trigger_factory.cpp
  src/behavioral/pedestrian_state.cpp
  src/behavioral/pedestrian_trigger.cpp
  src/behavioral/pedestrian_state_machine.cpp
  src/behavioral/pedestrian_trigger_factory.cpp
  src/behavioral/obstacle_state.cpp
  src/behavioral/obstacle_trigger.cpp
  src/behavioral/obstacle_state_machine.cpp
  src/behavioral/obstacle_trigger_factory.cpp
)
add_dependencies(BP_LIB
  ${catkin_EXPORTED_TARGETS}
)

add_executable(lanelet_service
  src/lanelet_service.cpp
)
add_dependencies(lanelet_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanelet_service ${catkin_LIBRARIES} stdc++fs)

add_executable(env_model
  src/env_model.cpp
)
add_dependencies(env_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(env_model ${catkin_LIBRARIES})

add_executable(ego_localization_map_service
  src/ego_localization_map_service.cpp
)
add_dependencies(ego_localization_map_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_localization_map_service ${catkin_LIBRARIES})

install(
  TARGETS BP_LIB
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/behavioral
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

