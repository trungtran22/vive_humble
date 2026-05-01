cmake_minimum_required(VERSION 3.8)
project(vive_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# OpenVR is built outside the ROS workspace.
# Override on the colcon command line with:
#   --cmake-args -DOPENVR=/path/to/openvr
if(NOT DEFINED OPENVR)
  if(DEFINED ENV{OPENVR})
    set(OPENVR "$ENV{OPENVR}")
  else()
    set(OPENVR "$ENV{HOME}/libraries/openvr")
  endif()
endif()
message(STATUS "Using OpenVR from: ${OPENVR}")

add_compile_definitions(LINUX LINUX64 POSIX)
add_compile_definitions(OPENVR_PATH="${OPENVR}")

find_library(OPENVR_LIBRARIES
  NAMES openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES linux64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
if(NOT OPENVR_LIBRARIES)
  message(FATAL_ERROR "Could not locate libopenvr_api.so under ${OPENVR}/{bin,lib}/linux64. "
                      "Build OpenVR (https://github.com/ValveSoftware/openvr) and pass "
                      "-DOPENVR=/path/to/openvr to colcon.")
endif()
message(STATUS "Found OpenVR library: ${OPENVR_LIBRARIES}")

set(OPENVR_INCLUDE_DIRS
  ${OPENVR}/headers
  ${OPENVR}/samples
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${OPENVR_INCLUDE_DIRS}
)

# vive_node: HMD/controller/tracker poses + joystick state
add_executable(vive_node
  src/vive_node.cpp
  src/vr_interface.cpp
)
ament_target_dependencies(vive_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(vive_node
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# vive_pose: minimal absolute-pose publisher
add_executable(vive_pose
  src/vive_pose.cpp
)
ament_target_dependencies(vive_pose
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)
target_link_libraries(vive_pose
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

install(TARGETS
  vive_node
  vive_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_servervr.sh
  scripts/check_vr.sh
  scripts/close_servervr.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES 60-HTC-Vive-perms.rules
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
