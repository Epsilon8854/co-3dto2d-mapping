cmake_minimum_required(VERSION 3.5)
project(co_3dto2d_mapping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(occupancy_mapper src/occupancy_mapper.cpp)
target_compile_features(occupancy_mapper PUBLIC cxx_std_17)
target_include_directories(occupancy_mapper PRIVATE include)
ament_target_dependencies(
  occupancy_mapper
  builtin_interfaces
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(
  TARGETS occupancy_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    co_3dto2d_mapping/pointcloud_rear_sector_filter.py
    co_3dto2d_mapping/pointcloud_frame_republisher.py
    co_3dto2d_mapping/imu_frame_republisher.py
    co_3dto2d_mapping/initial_xy_icp_alignment.py
    co_3dto2d_mapping/rerun_mapping_node.py
    scripts/run_livox_robot.sh
    scripts/run_two_mid360_2d_mapping.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Keep the public executable name used by existing launch files, while the
# implementation extends the original republisher with temporal merged-map
# fusion driven by current-frame local observations.
install(
  PROGRAMS co_3dto2d_mapping/record_republisher_temporal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME record_republisher.py
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(
    test_occupancy_grid_utils
    test/test_occupancy_grid_utils.cpp
  )
  target_compile_features(test_occupancy_grid_utils PUBLIC cxx_std_17)
  target_include_directories(test_occupancy_grid_utils PRIVATE include)

  ament_add_gtest(
    test_occupancy_point_filter
    test/test_occupancy_point_filter.cpp
  )
  target_compile_features(test_occupancy_point_filter PUBLIC cxx_std_17)
  target_include_directories(test_occupancy_point_filter PRIVATE include)

  ament_add_gtest(
    test_temporal_occupancy
    test/test_temporal_occupancy.cpp
  )
  target_compile_features(test_temporal_occupancy PUBLIC cxx_std_17)
  target_include_directories(test_temporal_occupancy PRIVATE include)

  ament_add_gtest(
    test_planar_icp
    test/test_planar_icp.cpp
  )
  target_compile_features(test_planar_icp PUBLIC cxx_std_17)
  target_include_directories(test_planar_icp PRIVATE include)

  ament_add_pytest_test(
    test_record_republisher
    test/test_record_republisher.py
    PYTHON_EXECUTABLE /usr/bin/python3
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_temporal_fusion
    test/test_temporal_fusion.py
    PYTHON_EXECUTABLE /usr/bin/python3
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_record_republisher_temporal
    test/test_record_republisher_temporal.py
    PYTHON_EXECUTABLE /usr/bin/python3
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_launch_isolation
    test/test_launch_isolation.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )

  ament_add_pytest_test(
    test_two_live_bag_runner
    test/test_two_live_bag_runner.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )

  ament_add_pytest_test(
    test_visualization_utils
    test/test_visualization_utils.py
    PYTHON_EXECUTABLE /usr/bin/python3
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_workspace_layout
    test/test_workspace_layout.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )
endif()

ament_package()
