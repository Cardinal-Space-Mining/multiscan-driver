cmake_minimum_required(VERSION 3.15)
project(multiscan_driver)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(csm_metrics REQUIRED)

# --- Presets ------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(SYSTEM)

# --- Scansegmentxd ------------------------------------------------------------
add_library(scansegment_xd STATIC
  "src/sick_scan_xd/msgpack11/msgpack11.cpp"
  "src/sick_scan_xd/compact_parser.cpp"
  "src/sick_scan_xd/msgpack_parser.cpp"
  "src/sick_scan_xd/scansegment_parser_output.cpp"
  "src/sick_scan_xd/sick_scan_common_nw.cpp"
  "src/sick_scan_xd/sick_scan_common_tcp.cpp"
  "src/sick_scan_xd/sopas_services.cpp"
  "src/sick_scan_xd/softwarePLL.cpp"
  "src/sick_scan_xd/udp_receiver.cpp"
  "src/sick_scan_xd/udp_sockets.cpp"
  "src/sick_scan_xd/tcp/colaa.cpp"
  "src/sick_scan_xd/tcp/colab.cpp"
  "src/sick_scan_xd/tcp/errorhandler.cpp"
  "src/sick_scan_xd/tcp/Mutex.cpp"
  "src/sick_scan_xd/tcp/SickThread.cpp"
  "src/sick_scan_xd/tcp/SopasBase.cpp"
  "src/sick_scan_xd/tcp/tcp.cpp"
  "src/sick_scan_xd/tcp/Time.cpp"
  "src/sick_scan_xd/tcp/toolbox.cpp"
  "src/sick_scan_xd/tcp/wsa_init.cpp")
ament_target_dependencies(scansegment_xd)

# --- Executables --------------------------------------------------------------
add_executable(multiscan_driver
  "src/multiscan_driver.cpp" )
target_link_libraries(multiscan_driver
  scansegment_xd
  csm_metrics::stats
  csm_metrics::profiling
  Eigen3::Eigen )
target_include_directories(multiscan_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>" )
ament_target_dependencies(multiscan_driver
  rclcpp
  sensor_msgs
  tf2_ros
  csm_metrics )

# --- Export -------------------------------------------------------------------
install(TARGETS
  multiscan_driver
  DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY include/
  DESTINATION include )

ament_export_include_directories(
  include )
ament_export_dependencies(
  rclcpp
  sensor_msgs
  tf2_ros
  csm_metrics )
ament_package()
