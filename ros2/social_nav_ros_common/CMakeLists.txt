cmake_minimum_required(VERSION 3.16)
project(social_nav_ros_common VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(social_nav_core REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp social_nav_msgs)
target_link_libraries(${PROJECT_NAME} INTERFACE social_nav_core::social_nav_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_conversions test_people_subscriber test_robot_model_params)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} ${PROJECT_NAME})
    target_compile_options(${t} PRIVATE -Wall -Wextra)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp social_nav_msgs social_nav_core)
ament_package()
