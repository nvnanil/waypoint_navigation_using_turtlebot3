cmake_minimum_required(VERSION 3.8)
project(group13_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include/group13_final)
# broadcaster node
add_executable(broadcaster_demo src/broadcaster_demo.cpp)
ament_target_dependencies(broadcaster_demo rclcpp tf2_geometry_msgs tf2_ros geometry_msgs mage_msgs)
install(TARGETS broadcaster_demo DESTINATION lib/${PROJECT_NAME})
# listener node node and navigation
add_executable(aruco_listen src/aruco_listener.cpp)
ament_target_dependencies(aruco_listen rclcpp rclcpp_action rclcpp_components nav2_msgs tf2_geometry_msgs tf2_ros geometry_msgs mage_msgs ros2_aruco_interfaces)
install(TARGETS aruco_listen DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include launch config DESTINATION share/${PROJECT_NAME})

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
