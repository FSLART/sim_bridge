cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(lart_msgs REQUIRED)

add_executable(sim_bridge src/sim_bridge.cpp)
ament_target_dependencies(sim_bridge
  rclcpp
  eufs_msgs
  ackermann_msgs
  lart_msgs
)

install(TARGETS
  sim_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
