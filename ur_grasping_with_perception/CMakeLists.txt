cmake_minimum_required(VERSION 3.8)
project(ur_grasping_with_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(grasping_with_perception_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(find_apriltag_server SHARED
  src/find_apriltag_server.cpp)
target_include_directories(find_apriltag_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(find_apriltag_server
  PRIVATE "ACTION_FIND_APRILTAG_BUILDING_DLL")
ament_target_dependencies(find_apriltag_server
  "grasping_with_perception_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "apriltag_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros")

rclcpp_components_register_node(find_apriltag_server PLUGIN "ur_grasping_with_perception::FindApriltagActionServer" EXECUTABLE Findapriltag_action_server)
install(TARGETS
find_apriltag_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#add_library(pick_and_place_with_perception SHARED
#  src/pick_and_place_with_perception.cpp)
#target_include_directories(pick_and_place_with_perception PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_definitions(pick_and_place_with_perception
#  PRIVATE "ACTION_FIND_APRILTAG_BUILDING_DLL")
#ament_target_dependencies(pick_and_place_with_perception
#  "grasping_with_perception_interfaces"
#  "rclcpp"
#  "rclcpp_action"
#  "rclcpp_components")
#rclcpp_components_register_node(pick_and_place_with_perception PLUGIN "ur_grasping_with_perception::FindApriltagActionClient" EXECUTABLE Findapriltag_action_client)
#install(TARGETS
#pick_and_place_with_perception
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)


add_executable(pick_place_with_perception
  src/pick_and_place_with_perception.cpp)

target_include_directories(pick_place_with_perception
  PUBLIC include)
ament_target_dependencies(pick_place_with_perception
  ament_cmake
  grasping_with_perception_interfaces
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs
  grasping_msgs )

install(TARGETS 
	pick_place_with_perception
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

# Install the config file
install(DIRECTORY 
	config
    DESTINATION share/${PROJECT_NAME}
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

# Rajouts projet <!> <---------------------
add_executable(mathys src/mathys.cpp)

target_include_directories(mathys
  PUBLIC include)

ament_target_dependencies(mathys
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  grasping_with_perception_interfaces
  interactive_markers
  control_msgs
  grasping_msgs
)

install(TARGETS
  mathys
  DESTINATION lib/${PROJECT_NAME})

