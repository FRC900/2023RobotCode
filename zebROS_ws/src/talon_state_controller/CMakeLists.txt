cmake_minimum_required(VERSION 3.20)
project(talon_state_controller)

include("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
	controller_interface
	hardware_interface
  periodic_interval_counter
	pluginlib
	realtime_tools
	roscpp
	sensor_msgs
	std_msgs
	ctre_interfaces
	talon_state_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib
  controller_interface sensor_msgs talon_state_msgs
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  )

include_directories(${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/cancoder_state_controller.cpp
  src/candle_state_controller.cpp
  src/canifier_state_controller.cpp
  src/latency_compensation_state_controller.cpp
  src/talon_config_controller.cpp
  src/talon_state_controller.cpp
  src/talonfxpro_config_controller.cpp
  src/talonfxpro_state_controller.cpp
  src/orchestra_state_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(talon_state_controller_test
                    test/talon_state_controller.test
                    test/talon_state_controller_test.cpp)
  target_link_libraries(talon_state_controller_test ${PROJECT_NAME})
endif()
# Install
#install(DIRECTORY include/${PROJECT_NAME}/
#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES talon_state_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
