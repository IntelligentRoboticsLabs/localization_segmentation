cmake_minimum_required(VERSION 3.5)
project(octomap_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(octomap_slam_msgs REQUIRED)

set(dependencies
  rclcpp
  octomap
  octomap_msgs
  octomap_slam_msgs
  tf2
  tf2_ros
)

include_directories(include/)

add_library(${PROJECT_NAME} SHARED
  src/octomap_slam/OctomapServer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(octomap_server src/octomap_server.cpp)
ament_target_dependencies(octomap_server ${dependencies})
target_link_libraries(octomap_server ${PROJECT_NAME} octomap)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  octomap_server
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
