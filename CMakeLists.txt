cmake_minimum_required(VERSION 2.8.3)
project(see_ego_motion_ukf)


## Specify source files for generation of executables only available for this package
set(EXECUTABLES "src/${PROJECT_NAME}_node.cpp" "src/UnscentedKalmanFilter.cpp" "src/Sensors/GPS.cpp" "src/Sensors/IMU.cpp"
        "src/Sensors/Sensor.cpp" "src/SensorDataHandler.cpp" "src/SensorFusionLogic.cpp" "src/Sensors/SLAM.cpp" "src/Sensors/WheelSpeed.cpp" src/Sensors/SensorNullObject.cpp include/see_ego_motion_ukf/Sensors/SensorNullObject.h)

## Specify catkin packages your project and dependent projects need
set(CATKIN_DEPENDENCIES "roscpp")

## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
## Only customize below this line if you really know what you're doing
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

## Use C++11
add_definitions(-std=c++11 -Wall -Werror -g3)

## Find catkin macros and libraries
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    Eigen3
    message_generation
    geometry_msgs
    message_filters
    gfr_common
    nav_msgs
)

add_message_files(
  FILES Vehicle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: this is necessary if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_DEPENDENCIES}
    message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${EIGEN3_INCLUDE_DIR}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  ${EXECUTABLES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
)

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
