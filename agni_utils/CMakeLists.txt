cmake_minimum_required(VERSION 2.8.3)
project(agni_utils)
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES agni_tactile_processing
)

add_library(agni_tactile_processing src/tactile_calibration.cpp)
add_dependencies(agni_tactile_processing ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(tactile_calibration_test test/tactile_calibration_test.cpp)
add_dependencies(tactile_calibration_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(tactile_calibration_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread agni_tactile_processing)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS agni_tactile_processing
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



