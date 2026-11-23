cmake_minimum_required(VERSION 3.16)
project(social_nav_mppi_critics VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(social_nav_ros_common REQUIRED)

set(deps rclcpp rclcpp_lifecycle nav2_mppi_controller nav2_costmap_2d pluginlib tf2 tf2_ros
         geometry_msgs social_nav_msgs)

add_library(${PROJECT_NAME} SHARED src/social_critic.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(${PROJECT_NAME} ${deps})
target_link_libraries(${PROJECT_NAME}
  social_nav_core::social_nav_core social_nav_ros_common::social_nav_ros_common)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

pluginlib_export_plugin_description_file(nav2_mppi_controller social_critic_plugins.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_social_critic test/test_social_critic.cpp)
  target_include_directories(test_social_critic PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
  ament_target_dependencies(test_social_critic ${deps})
  target_link_libraries(test_social_critic
    social_nav_core::social_nav_core social_nav_ros_common::social_nav_ros_common)
  target_compile_options(test_social_critic PRIVATE -Wall -Wextra)
endif()

ament_package()
