cmake_minimum_required(VERSION 3.8)
project(myrobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# ---- 实车底盘驱动:麦轮解算 + 4 轮速度 PID + 串口协议(对应固件 host_link.c)----
add_executable(chassis_driver src/chassis_driver_node.cpp)
target_include_directories(chassis_driver PRIVATE src)
target_compile_features(chassis_driver PUBLIC cxx_std_17)
ament_target_dependencies(chassis_driver
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_msgs)
install(TARGETS chassis_driver DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_chassis test/test_chassis.cpp)
  target_include_directories(test_chassis PRIVATE src)
endif()

install(
  PROGRAMS scripts/cmd_vel_guard.py scripts/scan_filter.py scripts/vel_track_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
