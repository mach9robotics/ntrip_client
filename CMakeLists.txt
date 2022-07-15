cmake_minimum_required(VERSION 3.0.2)
project(ntrip_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mavros_msgs
  nmea_msgs
  message_generation
)

find_package(catkin QUIET COMPONENTS
  rviz
)

## Mark python modules for installation
catkin_python_setup()

## Add service files
add_service_files(
  FILES
  NtripClientConnect.srv
  NtripClientStatus.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    rospy
    std_msgs
    mavros_msgs
    rtcm_msgs
    nmea_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Set QT parameters
if(rviz_FOUND)
    ## This setting causes Qt's "MOC" generation to happen automatically.
    set(CMAKE_AUTOMOC ON)

    ## This plugin includes Qt widgets, so we must include Qt.
    ## We'll use the version that rviz used so they are compatible.
    if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
    else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
    endif()

    ## I prefer the Qt signals and slots to avoid defining "emit", "slots",
    ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
    add_definitions(-DQT_NO_KEYWORDS)
endif()

if(rviz_FOUND)
  add_library(${PROJECT_NAME}
    src/ntrip_panel.cpp
    include/${PROJECT_NAME}/ntrip_panel.hpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
  )
  install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES
    plugins_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/ntrip_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the launch files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)