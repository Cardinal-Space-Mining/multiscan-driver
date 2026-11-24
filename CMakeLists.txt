cmake_minimum_required(VERSION 3.15)
project(multiscan_driver)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(csm_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(csm_metrics REQUIRED)
find_package(ZLIB REQUIRED)

standard_compile_options()


# --- Core (ROS-free decoding) ----------------------------------------------------
add_library(multiscan_core STATIC
  "src/compact/decoder.cpp"
  "src/compact/point_layout.cpp"
  "src/frame_assembler.cpp"
  "src/ghost/ghost_filter.cpp"
  "src/net/udp_receiver.cpp"
  "src/sopas/cola_client.cpp"
  "src/sopas/sensor_session.cpp"
  "src/sopas/sensor_settings.cpp"
  "src/time_sync.cpp")
target_include_directories(multiscan_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_link_libraries(multiscan_core PUBLIC
    ZLIB::ZLIB )

# --- Executables --------------------------------------------------------------
add_executable(multiscan_driver
    "src/multiscan_driver.cpp" )
target_link_libraries(multiscan_driver
    multiscan_core
    csm_metrics::stats
    csm_metrics::profiling )
target_include_directories(multiscan_driver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>" )
auto_target_dependencies(multiscan_driver
    csm_utils
    rclcpp
    sensor_msgs
    geometry_msgs
    diagnostic_msgs
    tf2_ros
    csm_metrics )

# --- Tests --------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # captures and recordings under test/data, test/support helpers
    add_library(multiscan_test_support INTERFACE)
    target_include_directories(multiscan_test_support INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/test" )
    target_link_libraries(multiscan_test_support INTERFACE
        multiscan_core
        ZLIB::ZLIB )
    target_compile_definitions(multiscan_test_support INTERFACE
        MS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data" )

    ament_add_gtest(test_compact_packet "test/test_compact_packet.cpp")
    target_link_libraries(test_compact_packet multiscan_test_support)

    ament_add_gtest(test_decoder "test/test_decoder.cpp")
    target_link_libraries(test_decoder multiscan_test_support)

    ament_add_gtest(test_frame_assembler "test/test_frame_assembler.cpp")
    target_link_libraries(test_frame_assembler multiscan_test_support)

    ament_add_gtest(test_udp_receiver "test/test_udp_receiver.cpp")
    target_link_libraries(test_udp_receiver multiscan_core)

    ament_add_gtest(test_sopas "test/test_sopas.cpp")
    target_link_libraries(test_sopas multiscan_test_support)

    ament_add_gtest(test_time_sync "test/test_time_sync.cpp")
    target_link_libraries(test_time_sync multiscan_core)

    ament_add_gtest(test_ghost_filter "test/test_ghost_filter.cpp")
    target_link_libraries(test_ghost_filter multiscan_test_support)

    # not a test: decoder timing on the captures
    add_executable(bench_decoder "test/bench_decoder.cpp")
    target_link_libraries(bench_decoder multiscan_test_support)

    # not a test: ghost filter timing (bench_ghost_filter MODEL [CAPTURE] [--paced])
    add_executable(bench_ghost_filter "test/bench_ghost_filter.cpp")
    target_link_libraries(bench_ghost_filter multiscan_test_support)
endif()

# --- Export -------------------------------------------------------------------
install(TARGETS
    multiscan_driver
    DESTINATION lib/${PROJECT_NAME} )

install(PROGRAMS
    "tools/ms_geometry.py"
    DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY include/
    DESTINATION include )

ament_export_include_directories(
    include )
ament_export_dependencies(
    csm_utils
    rclcpp
    sensor_msgs
    geometry_msgs
    diagnostic_msgs
    tf2_ros
    csm_metrics )
ament_package()
