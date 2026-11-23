cmake_minimum_required(VERSION 3.16)
project(social_nav_bt_nodes VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_core REQUIRED)
find_package(social_nav_ros_common REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/select_social_behavior.cpp
  src/hold_position.cpp
  src/register_nodes.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
# BT_REGISTER_NODES を dlopen 可能な extern "C" シンボルにする(Nav2 の BT プラグインと同じ)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp behaviortree_cpp nav_msgs nav2_msgs geometry_msgs tf2 tf2_ros social_nav_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
  social_nav_core::social_nav_core social_nav_ros_common::social_nav_ros_common)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_domain 71)
  foreach(t test_hold_position test_select_social_behavior)
    # 並列 colcon test でトピックが混線しないよう、テストごとに ROS_DOMAIN_ID を分ける
    ament_add_gtest(${t} test/${t}.cpp ENV ROS_DOMAIN_ID=${_domain})
    math(EXPR _domain "${_domain} + 1")
    target_link_libraries(${t} ${PROJECT_NAME})
    target_compile_options(${t} PRIVATE -Wall -Wextra)
    target_compile_definitions(${t} PRIVATE
      SOCIAL_NAV_BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}>")
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp behaviortree_cpp nav_msgs nav2_msgs geometry_msgs tf2 tf2_ros
  social_nav_msgs social_nav_core social_nav_ros_common)
ament_package()
