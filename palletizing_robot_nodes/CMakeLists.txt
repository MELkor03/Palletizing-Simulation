cmake_minimum_required(VERSION 3.8)
project(palletizing_robot_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(palletizing_robot_interfaces REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(move_robot src/move_robot.cpp)
add_executable(palletization_service src/palletization_service.cpp)
add_executable(spawner src/spawner.cpp)
add_executable(pick_n_place src/pick_n_place.cpp)
target_include_directories(pick_n_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_robot
  "moveit_ros_planning_interface"
  "rclcpp"
)
target_compile_features(pick_n_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pick_n_place
  "linkattacher_msgs"
  "moveit_ros_planning_interface"
  "rclcpp"
)
target_compile_features(spawner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  spawner
  "rclcpp" 
  "gazebo_msgs" 
  "geometry_msgs" 
  "moveit_ros_planning_interface" 
  "ament_index_cpp"
  "palletizing_robot_interfaces"
)
target_compile_features(palletization_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  palletization_service
  "linkattacher_msgs"
  "moveit_ros_planning_interface"
  "moveit_ros_planning_interface" 
  "rclcpp"
  "palletizing_robot_interfaces"
)

install(TARGETS move_robot pick_n_place spawner palletization_service
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
