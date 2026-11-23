cmake_minimum_required(VERSION 3.16)
project(social_nav_costmap_plugins VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(social_nav_ros_common REQUIRED)

add_library(${PROJECT_NAME} SHARED src/social_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp nav2_costmap_2d nav2_util pluginlib tf2 tf2_ros social_nav_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
  social_nav_ros_common::social_nav_ros_common social_nav_core::social_nav_core)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 並列 colcon test でトピックが混線しないよう ROS_DOMAIN_ID を分ける
  ament_add_gtest(test_social_layer test/test_social_layer.cpp ENV ROS_DOMAIN_ID=73)
  target_link_libraries(test_social_layer ${PROJECT_NAME})
  target_compile_options(test_social_layer PRIVATE -Wall -Wextra)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav2_costmap_2d nav2_util pluginlib tf2 tf2_ros
  social_nav_msgs social_nav_core social_nav_ros_common)
ament_package()
