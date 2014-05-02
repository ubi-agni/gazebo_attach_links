cmake_minimum_required(VERSION 2.8.3)
project(agni_tactile_viz)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sr_robot_msgs visualization_msgs agni_utils)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs sr_robot_msgs visualization_msgs agni_utils
    INCLUDE_DIRS 
    LIBRARIES
)

#############################
#  RVIZ MARKER #
#############################
add_executable(agni_tactile_viz src/tactile_viz.cpp)
add_dependencies(agni_tactile_viz ${catkin_EXPORTED_TARGETS})
target_link_libraries(agni_tactile_viz agni_tactile_processing ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS agni_tactile_viz 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp"
#)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
