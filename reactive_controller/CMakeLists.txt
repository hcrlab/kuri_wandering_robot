cmake_minimum_required(VERSION 2.8.3)
project(reactive_controller)

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
add_library(reactive_controller
  src/reactive_controller.cpp
)
target_link_libraries(reactive_controller
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(reactive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(reactive_controller_node
  src/reactive_controller_node.cpp
)


add_dependencies(reactive_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_controller_node reactive_controller)
set_target_properties(reactive_controller_node PROPERTIES OUTPUT_NAME reactive_controller)

install(
    TARGETS
        reactive_controller_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        reactive_controller
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)