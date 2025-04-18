# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.8)
   message(FATAL_ERROR "CMake >= 2.8.0 required")
endif()
if(CMAKE_VERSION VERSION_LESS "2.8.3")
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.8.3...3.23)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_cmake_targets_defined "")
set(_cmake_targets_not_defined "")
set(_cmake_expected_targets "")
foreach(_cmake_expected_target IN ITEMS moveit_task_constructor_core::moveit_task_constructor_core_stages moveit_task_constructor_core::moveit_task_constructor_core_stage_plugins moveit_task_constructor_core::moveit_task_constructor_core)
  list(APPEND _cmake_expected_targets "${_cmake_expected_target}")
  if(TARGET "${_cmake_expected_target}")
    list(APPEND _cmake_targets_defined "${_cmake_expected_target}")
  else()
    list(APPEND _cmake_targets_not_defined "${_cmake_expected_target}")
  endif()
endforeach()
unset(_cmake_expected_target)
if(_cmake_targets_defined STREQUAL _cmake_expected_targets)
  unset(_cmake_targets_defined)
  unset(_cmake_targets_not_defined)
  unset(_cmake_expected_targets)
  unset(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT _cmake_targets_defined STREQUAL "")
  string(REPLACE ";" ", " _cmake_targets_defined_text "${_cmake_targets_defined}")
  string(REPLACE ";" ", " _cmake_targets_not_defined_text "${_cmake_targets_not_defined}")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_cmake_targets_defined_text}\nTargets not yet defined: ${_cmake_targets_not_defined_text}\n")
endif()
unset(_cmake_targets_defined)
unset(_cmake_targets_not_defined)
unset(_cmake_expected_targets)


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target moveit_task_constructor_core::moveit_task_constructor_core_stages
add_library(moveit_task_constructor_core::moveit_task_constructor_core_stages SHARED IMPORTED)

set_target_properties(moveit_task_constructor_core::moveit_task_constructor_core_stages PROPERTIES
  INTERFACE_LINK_LIBRARIES "moveit_task_constructor_core::moveit_task_constructor_core;moveit_core::collision_detector_bullet_plugin;moveit_core::moveit_butterworth_filter;moveit_core::moveit_butterworth_parameters;moveit_core::moveit_collision_distance_field;moveit_core::moveit_collision_detection;moveit_core::moveit_collision_detection_fcl;moveit_core::moveit_collision_detection_bullet;moveit_core::moveit_dynamics_solver;moveit_core::moveit_constraint_samplers;moveit_core::moveit_distance_field;moveit_core::moveit_exceptions;moveit_core::moveit_kinematics_base;moveit_core::moveit_kinematic_constraints;moveit_core::moveit_kinematics_metrics;moveit_core::moveit_planning_interface;moveit_core::moveit_planning_scene;moveit_core::moveit_planning_request_adapter;moveit_core::moveit_robot_model;moveit_core::moveit_robot_state;moveit_core::moveit_robot_trajectory;moveit_core::moveit_smoothing_base;moveit_core::moveit_test_utils;moveit_core::moveit_trajectory_processing;moveit_core::moveit_transforms;moveit_core::moveit_utils;geometry_msgs::geometry_msgs__rosidl_generator_c;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_c;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c;geometry_msgs::geometry_msgs__rosidl_typesupport_c;geometry_msgs::geometry_msgs__rosidl_generator_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_cpp;geometry_msgs::geometry_msgs__rosidl_generator_py;moveit_ros_planning::moveit_rdf_loader;moveit_ros_planning::moveit_kinematics_plugin_loader;moveit_ros_planning::moveit_robot_model_loader;moveit_ros_planning::moveit_constraint_sampler_manager_loader;moveit_ros_planning::moveit_planning_pipeline;moveit_ros_planning::moveit_trajectory_execution_manager;moveit_ros_planning::moveit_plan_execution;moveit_ros_planning::moveit_planning_scene_monitor;moveit_ros_planning::moveit_collision_plugin_loader;moveit_ros_planning::moveit_default_planning_request_adapter_plugins;moveit_ros_planning::moveit_cpp;moveit_ros_planning_interface::moveit_common_planning_interface_objects;moveit_ros_planning_interface::moveit_planning_scene_interface;moveit_ros_planning_interface::moveit_move_group_interface;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_fastrtps_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_fastrtps_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_introspection_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_introspection_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_py;rclcpp::rclcpp;tf2_eigen::tf2_eigen;visualization_msgs::visualization_msgs__rosidl_generator_c;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_c;visualization_msgs::visualization_msgs__rosidl_generator_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_c;visualization_msgs::visualization_msgs__rosidl_typesupport_c;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_cpp;visualization_msgs::visualization_msgs__rosidl_generator_py"
)

# Create imported target moveit_task_constructor_core::moveit_task_constructor_core_stage_plugins
add_library(moveit_task_constructor_core::moveit_task_constructor_core_stage_plugins SHARED IMPORTED)

set_target_properties(moveit_task_constructor_core::moveit_task_constructor_core_stage_plugins PROPERTIES
  INTERFACE_LINK_LIBRARIES "moveit_task_constructor_core::moveit_task_constructor_core_stages"
)

# Create imported target moveit_task_constructor_core::moveit_task_constructor_core
add_library(moveit_task_constructor_core::moveit_task_constructor_core SHARED IMPORTED)

set_target_properties(moveit_task_constructor_core::moveit_task_constructor_core PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "moveit_core::collision_detector_bullet_plugin;moveit_core::moveit_butterworth_filter;moveit_core::moveit_butterworth_parameters;moveit_core::moveit_collision_distance_field;moveit_core::moveit_collision_detection;moveit_core::moveit_collision_detection_fcl;moveit_core::moveit_collision_detection_bullet;moveit_core::moveit_dynamics_solver;moveit_core::moveit_constraint_samplers;moveit_core::moveit_distance_field;moveit_core::moveit_exceptions;moveit_core::moveit_kinematics_base;moveit_core::moveit_kinematic_constraints;moveit_core::moveit_kinematics_metrics;moveit_core::moveit_planning_interface;moveit_core::moveit_planning_scene;moveit_core::moveit_planning_request_adapter;moveit_core::moveit_robot_model;moveit_core::moveit_robot_state;moveit_core::moveit_robot_trajectory;moveit_core::moveit_smoothing_base;moveit_core::moveit_test_utils;moveit_core::moveit_trajectory_processing;moveit_core::moveit_transforms;moveit_core::moveit_utils;geometry_msgs::geometry_msgs__rosidl_generator_c;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_c;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c;geometry_msgs::geometry_msgs__rosidl_typesupport_c;geometry_msgs::geometry_msgs__rosidl_generator_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_cpp;geometry_msgs::geometry_msgs__rosidl_generator_py;moveit_ros_planning::moveit_rdf_loader;moveit_ros_planning::moveit_kinematics_plugin_loader;moveit_ros_planning::moveit_robot_model_loader;moveit_ros_planning::moveit_constraint_sampler_manager_loader;moveit_ros_planning::moveit_planning_pipeline;moveit_ros_planning::moveit_trajectory_execution_manager;moveit_ros_planning::moveit_plan_execution;moveit_ros_planning::moveit_planning_scene_monitor;moveit_ros_planning::moveit_collision_plugin_loader;moveit_ros_planning::moveit_default_planning_request_adapter_plugins;moveit_ros_planning::moveit_cpp;moveit_ros_planning_interface::moveit_common_planning_interface_objects;moveit_ros_planning_interface::moveit_planning_scene_interface;moveit_ros_planning_interface::moveit_move_group_interface;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_fastrtps_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_fastrtps_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_introspection_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_c;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_introspection_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_typesupport_cpp;moveit_task_constructor_msgs::moveit_task_constructor_msgs__rosidl_generator_py;rclcpp::rclcpp;rviz_marker_tools::rviz_marker_tools;visualization_msgs::visualization_msgs__rosidl_generator_c;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_c;visualization_msgs::visualization_msgs__rosidl_generator_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_c;visualization_msgs::visualization_msgs__rosidl_typesupport_c;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_cpp;visualization_msgs::visualization_msgs__rosidl_generator_py"
)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
file(GLOB _cmake_config_files "${CMAKE_CURRENT_LIST_DIR}/moveit_task_constructor_coreTargetsExport-*.cmake")
foreach(_cmake_config_file IN LISTS _cmake_config_files)
  include("${_cmake_config_file}")
endforeach()
unset(_cmake_config_file)
unset(_cmake_config_files)

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(_cmake_target IN LISTS _cmake_import_check_targets)
  foreach(_cmake_file IN LISTS "_cmake_import_check_files_for_${_cmake_target}")
    if(NOT EXISTS "${_cmake_file}")
      message(FATAL_ERROR "The imported target \"${_cmake_target}\" references the file
   \"${_cmake_file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_cmake_file)
  unset("_cmake_import_check_files_for_${_cmake_target}")
endforeach()
unset(_cmake_target)
unset(_cmake_import_check_targets)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
