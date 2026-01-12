cmake_minimum_required(VERSION 3.22)
project(mrosuam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_TESTING OFF)

#cmake_policy(SET CMP0054 NEW)

# Find which machine this is - MACHINE_OBC is true if this is the on-board computer
set(HOST_NAME $ENV{NAME})
if (NOT HOST_NAME)
  cmake_host_system_information(RESULT HOST_NAME QUERY HOSTNAME)
endif()
message("Hostname is: ${HOST_NAME}")
if (HOST_NAME STREQUAL "halaena")
  set(MACHINE_OBC 1)
elseif(HOST_NAME STREQUAL "Matthew-PC")
  set(MACHINE_OBC 0)
else()
  message(FATAL_ERROR "Hostname wasn't halaena or Matthew-PC")
endif()


if (MACHINE_OBC)
  message("Machine is OBC")
else()
  message("Machine is GCS")
  add_subdirectory(extern/imgui)
  add_subdirectory(extern/implot)
  add_subdirectory(extern/jpeg-compressor)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(camera_ros REQUIRED)


if(NOT MACHINE_OBC)
  add_subdirectory(src/gui)
endif()
add_subdirectory(src/resource_monitor)

ament_package()