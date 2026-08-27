cmake_minimum_required(VERSION 3.8)
project(myrobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY src/robot config
  DESTINATION share/${PROJECT_NAME}
)

# launch 脚本实际位于源码树 config/launch/,仍安装到 share 的 launch/ 下,
# 供 ros2 launch 与 myrobot_bringup 中的 IncludeLaunchDescription 引用
install(
  DIRECTORY config/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES field.world
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
