# Install script for directory: /home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/src/mtc_tutorial

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/install/mtc_tutorial")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial" TYPE EXECUTABLE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/mtc_tutorial")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial"
         OLD_RPATH "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/install/moveit_task_constructor_core/lib:/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/install/rviz_marker_tools/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/install/moveit_task_constructor_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mtc_tutorial/mtc_tutorial")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE DIRECTORY FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/src/mtc_tutorial/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/mtc_tutorial")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/mtc_tutorial")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial/environment" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial/environment" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_index/share/ament_index/resource_index/packages/mtc_tutorial")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial/cmake" TYPE FILE FILES
    "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_core/mtc_tutorialConfig.cmake"
    "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/ament_cmake_core/mtc_tutorialConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mtc_tutorial" TYPE FILE FILES "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/src/mtc_tutorial/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_workspace/build/mtc_tutorial/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
