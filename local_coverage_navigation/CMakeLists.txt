cmake_minimum_required(VERSION 2.8.3)
project(local_coverage_navigation)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        cmake_modules
        costmap_2d
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        rospy
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# actions
add_action_files(
        DIRECTORY
        action
        FILES
        Navigate.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
)



catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(local_coverage_navigator
  src/local_coverage_navigator.cpp
)
target_link_libraries(local_coverage_navigator
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(local_coverage_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_coverage_navigator_node
  src/local_coverage_navigator_node.cpp
)


add_dependencies(local_coverage_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_coverage_navigator_node local_coverage_navigator)
set_target_properties(local_coverage_navigator_node PROPERTIES OUTPUT_NAME local_coverage_navigator)

install(
    TARGETS
        local_coverage_navigator_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        local_coverage_navigator
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)