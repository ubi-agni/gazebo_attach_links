cmake_minimum_required(VERSION 2.8.3)
project(agni_grasp_force_limiter)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pr2_mechanism_msgs)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs pr2_mechanism_msgs
    INCLUDE_DIRS include
    LIBRARIES
)


add_executable(agni_grasp_force_limiter src/agni_grasp_force_limiter.cpp)
add_dependencies(agni_grasp_force_limiter ${catkin_EXPORTED_TARGETS})
target_link_libraries(agni_grasp_force_limiter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS agni_grasp_force_limiter 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

