cmake_minimum_required(VERSION 3.5)
project(rif_ros2_utils)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Curses)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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


add_executable(compress_image
  src/compress_image.cpp)
target_include_directories(compress_image
  PUBLIC include)
ament_target_dependencies(compress_image
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  )

install(TARGETS compress_image
  DESTINATION lib/${PROJECT_NAME}
  )


# # uncomment to install launch and/or rviz directories
# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
#   )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/filter-tf-msgs.py
  scripts/msg-to-action.py
  DESTINATION lib/${PROJECT_NAME}
)

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
