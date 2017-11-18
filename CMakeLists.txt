cmake_minimum_required(VERSION 2.8.3)
project(hark_audio_capture)

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  dynamic_reconfigure
  hark_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
  cfg/HarkAudioCapture.cfg
)

catkin_package(
  CATKIN_DEPENDS audio_common_msgs dynamic_reconfigure hark_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hark_audio_capture_node.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
