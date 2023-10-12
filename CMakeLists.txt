cmake_minimum_required(VERSION 3.5)
project(nav2_complete_coverage_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_complete_coverage_msgs REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
  pluginlib
  nav2_complete_coverage_msgs
  Fields2Cover
  OpenCV
)

include_directories(
  include
)

#add_library(cover_area_navigator SHARED src/cover_area_navigator.cpp)
#ament_target_dependencies(cover_area_navigator ${dependencies})

add_library(nav2_complete_coverage_planner SHARED src/complete_coverage_planner.cpp)
ament_target_dependencies(nav2_complete_coverage_planner ${dependencies})
target_link_libraries(${PROJECT_NAME}
    Fields2Cover
)

#add_library(find_starting_pose SHARED src/find_starting_pose.cpp)
#ament_target_dependencies(find_starting_pose rclcpp behaviortree_cpp_v3)

#pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core complete_coverage_planner.xml)
#pluginlib_export_plugin_description_file(behaviortree_cpp_v3 find_starting_pose.xml)

install(
  TARGETS nav2_complete_coverage_planner #cover_area_navigator my_coverage_planner find_starting_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/
)

#install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
#ament_export_libraries(cover_area_navigator)
ament_export_libraries(nav2_complete_coverage_planner)

ament_export_dependencies(${dependencies})
ament_package()

