cmake_minimum_required(VERSION 3.15)
project(multiscan_driver)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(csm_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(csm_metrics REQUIRED)

standard_compile_options()

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
auto_target_dependencies(multiscan_driver
    csm_utils
    rclcpp
    sensor_msgs
    tf2_ros
    csm_metrics )

# --- Tests --------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ZLIB REQUIRED)

    # Copy of the current data path on the vendored parser, used as the
    # reference the rewrite is checked against (see test/oracle/).
    add_library(multiscan_test_oracle STATIC
        "test/oracle/legacy_pipeline.cpp" )
    target_include_directories(multiscan_test_oracle PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/test"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/include" )
    target_link_libraries(multiscan_test_oracle PUBLIC
        scansegment_xd
        ZLIB::ZLIB
        Eigen3::Eigen )
    target_compile_definitions(multiscan_test_oracle PUBLIC
        MS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data" )

    ament_add_gtest(test_compact_packet "test/test_compact_packet.cpp")
    target_link_libraries(test_compact_packet multiscan_test_oracle)

    ament_add_gtest(test_legacy_oracle "test/test_legacy_oracle.cpp")
    target_link_libraries(test_legacy_oracle
        multiscan_test_oracle
        ${sensor_msgs_TARGETS} )
endif()

# --- Export -------------------------------------------------------------------
install(TARGETS
    multiscan_driver
    DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY include/
    DESTINATION include )

ament_export_include_directories(
    include )
ament_export_dependencies(
    csm_utils
    rclcpp
    sensor_msgs
    tf2_ros
    csm_metrics )
ament_package()
