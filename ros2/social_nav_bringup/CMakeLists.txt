cmake_minimum_required(VERSION 3.16)
project(social_nav_bringup VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY params behavior_trees launch maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/smoke_launch.sh DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup_files test/test_bringup_files.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
