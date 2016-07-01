cmake_minimum_required(VERSION 2.8.3)
project(crosbot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    geometry_msgs
                    sensor_msgs
                    crosbot_msgs
                    message_runtime
             )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#  DIRECTORY msg
#  FILES
#    xMsg.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
#    geometry_msgs
#    sensor_msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES crosbot
   CATKIN_DEPENDS
       roscpp
       tf
       geometry_msgs
       sensor_msgs
       crosbot_msgs
       message_runtime
#  DEPENDS system_lib
   CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

# Additional (optional) crosbot cmake include
INCLUDE ( cmake/${PROJECT_NAME}-extras.cmake.in )

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/libxml2
)

## Declare a cpp library
# add_library(crosbot
#   src/${PROJECT_NAME}/crosbot.cpp
# )

# This does not automatically trigger CMAKE rebuild if new cpp files are added
#file(GLOB_RECURSE CPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/**.cpp)
#add_library(crosbot ${CPP_SOURCES})

add_library(crosbot
    src/config.cpp
    src/data/image.cpp
    src/data/time.cpp
    src/exceptions.cpp
    src/geometry/pointCloud.cpp
    src/geometry/quaternion.cpp
    src/handle.cpp
    src/image.cpp
    src/jobDispatcher.cpp
    src/jpeg.cpp
    src/serialization.cpp
    src/thread.cpp
)

## Declare a cpp executable
# add_executable(crosbot_node src/crosbot_node.cpp)

## Add dependencies to the executable
#add_dependencies(crosbot ${PROJECT_NAME} crosbot_gencpp)

## WHY IS THIS NEEDED FOR LOCAL BUILDS????
# Make sure crosbot_msgs actually makes the messages before crosbot is built
add_dependencies(crosbot
	crosbot_msgs_generate_messages
)

## Specify libraries to link a library or executable target against
# target_link_libraries(crosbot_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(crosbot 
    ${catkin_LIBRARIES}
    jpeg
    xml2
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
install(TARGETS crosbot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crosbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
