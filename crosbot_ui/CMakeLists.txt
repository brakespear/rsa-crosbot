cmake_minimum_required(VERSION 2.8.3)
project(crosbot_ui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp crosbot crosbot_map crosbot_fastslam image_transport)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES crosbot_ui
#  CATKIN_DEPENDS crosbot crosbot_fastslam roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/**.hpp)
QT4_ADD_RESOURCES(QT_RESOURCES_CPP resources/resources.qrc)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

## Declare a cpp library
add_library(crosbot_ui
  src/crosbotgui.cpp
  src/opengl.cpp
  src/joystick.cpp
  src/panels/simple.cpp
  src/panels/map.cpp
  src/panels/panelfactory.cpp
  src/panels/pointcloud.cpp
  src/panels/robot.cpp
  src/panels/snap.cpp
  src/panels/snap2.cpp
  src/panels/wasdmouse.cpp
  src/renders/map/map.cpp
  src/renders/map/geotiff.cpp
  src/renders/map/fastslam.cpp
  src/renders/robot/robotwidget.cpp
  src/renders/robot/simple.cpp
  src/renders/robot/image.cpp
  src/renders/robot/joystick.cpp
  src/renders/robot/preset.cpp
  src/renders/robot/jointcontrol.cpp
   
  ${QT_MOC_HPP} ${QT_RESOURCES_CPP}
)

## Declare a cpp executable
add_executable(gui src/nodes/gui.cpp)
add_executable(geo_logger src/nodes/geo_logger.cpp)

## Add dependencies to the executable
# add_dependencies(crosbot_ui_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(crosbot_ui
  ${catkin_LIBRARIES} crosbot crosbot_map crosbot_fastslam ${QT_LIBRARIES} glut GLU phonon
)

target_link_libraries(gui
  ${catkin_LIBRARIES} crosbot_ui
)

target_link_libraries(geo_logger
  ${catkin_LIBRARIES} crosbot_ui
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS crosbot_ui crosbot_ui_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crosbot_ui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
