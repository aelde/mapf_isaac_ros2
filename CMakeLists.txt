cmake_minimum_required(VERSION 3.8)
project(mapf_isaac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TbJob.srv"
  "msg/TbTask.msg"
  "msg/TbPathtoGoal.msg"
  "msg/Tbpose2D.msg"
  DEPENDENCIES
  geometry_msgs std_msgs
)

ament_python_install_package(scripts)
install(PROGRAMS
  scripts/ros_pathfollow_short_plus.py
  scripts/ros_tbJob_service_server.py
  scripts/ros_planner_v1.py
  scripts/ros_run_isaacsim.py
  scripts/ros_pathfollow_timestep.py
  scripts/ros_pathfol.py
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()
