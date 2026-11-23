cmake_minimum_required(VERSION 3.16)
project(social_nav_controller VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_ros_common REQUIRED)

set(deps rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav_msgs geometry_msgs tf2 tf2_ros pluginlib)

add_library(${PROJECT_NAME} SHARED src/social_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
ament_target_dependencies(${PROJECT_NAME} ${deps})
target_link_libraries(${PROJECT_NAME}
  social_nav_core::social_nav_core social_nav_ros_common::social_nav_ros_common)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_plugin_load test_controller_utils)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} ${PROJECT_NAME})
    ament_target_dependencies(${t} ${deps})
    target_compile_options(${t} PRIVATE -Wall -Wextra)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps} social_nav_core social_nav_msgs social_nav_ros_common)
ament_package()
