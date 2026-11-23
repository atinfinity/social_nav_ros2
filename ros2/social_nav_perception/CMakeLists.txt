cmake_minimum_required(VERSION 3.16)
project(social_nav_perception VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_ros_common REQUIRED)
find_package(social_nav_core REQUIRED)

add_library(${PROJECT_NAME} src/people_tracker_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_sensor_msgs social_nav_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC social_nav_ros_common::social_nav_ros_common social_nav_core::social_nav_core)

add_executable(people_tracker_node src/main.cpp)
target_compile_options(people_tracker_node PRIVATE -Wall -Wextra)
target_link_libraries(people_tracker_node ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS people_tracker_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  foreach(t test_perception_utils test_people_tracker_node)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} ${PROJECT_NAME})
    target_compile_options(${t} PRIVATE -Wall -Wextra)
  endforeach()
  ament_target_dependencies(test_people_tracker_node tf2_geometry_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs
  social_nav_msgs social_nav_ros_common social_nav_core)
ament_package()
