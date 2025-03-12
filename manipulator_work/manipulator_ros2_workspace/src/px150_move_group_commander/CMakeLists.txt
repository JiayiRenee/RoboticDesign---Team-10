cmake_minimum_required(VERSION 3.8)
project(px150_move_group_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interbotix_moveit_interface_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)

set(
  MOVEIT_INTERFACE_DEPENDENCIES
    interbotix_moveit_interface_msgs
    moveit_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
)

include_directories(
  include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(px150_move_group_commander_node src/px150_move_group_commander_node.cpp src/moveit_interface_obj.cpp)
target_include_directories(px150_move_group_commander_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(px150_move_group_commander_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  px150_move_group_commander_node
  ${MOVEIT_INTERFACE_DEPENDENCIES}
)



install(TARGETS px150_move_group_commander_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
