cmake_minimum_required(VERSION 3.6.3)
project(gazebo_frcrobot_control)

include ("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

# Through transitive dependencies in the packages above, gazebo_frcrobot_control
# depends on Simbody.  There is a bug in the Ubuntu Artful (17.10) version of
# the Simbody package where it includes /usr/lib/libblas.so and
# /usr/lib/liblapack.so in the CMake list of libraries even though neither of
# those two paths exist (they both really live in /usr/lib/<arch>-linux-gnu).
# We remove these two during build-time on artful below; this works because
# they both will get resolved to the proper paths during runtime linking.
find_program(LSB_RELEASE_EXEC lsb_release)
if(NOT LSB_RELEASE_EXEC STREQUAL "LSB_RELEASE_EXEC-NOTFOUND")
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE OS_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(OS_CODENAME STREQUAL "artful")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/libblas.so")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/liblapack.so")
  endif()
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_frcrobot_control_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
