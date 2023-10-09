cmake_minimum_required(VERSION 2.8.3)
project(reemc_controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
  transmission_interface
  ddynamic_reconfigure
  whole_body_state_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reemc_effort_control
#  CATKIN_DEPENDS force_control_reemc roscpp
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#Yaml torque control
add_executable(yaml_effort
  src/yaml_effort.cpp
)

target_link_libraries(yaml_effort
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_library(${PROJECT_NAME} src/effort_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES reemc_effort_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
