cmake_minimum_required(VERSION 3.5)
project(rif-ros2-utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Curses)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(${CURSES_FOUND})
  add_executable(keyboard_teleop src/keyboard_teleop.cpp)
  ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs)
  target_link_libraries(keyboard_teleop -lncurses)
  target_include_directories(keyboard_teleop PUBLIC
    ${CURSES_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS keyboard_teleop
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "libncurses5-dev library NOT found. NOT building keyboard_teleop node.")
endif()

# # uncomment to allow colcon to find the launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
