cmake_minimum_required(VERSION 3.8)
project(a_star_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(astar_planner src/astar_planner.cpp)
# add_executable(astar_planner_ros src/astar_planner_ros.cpp)
add_executable(astar_planning_node src/astar_planning_node.cpp)
target_include_directories(astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(astar_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  astar_planner
  "rclcpp"
  "nav2_costmap_2d"
  "nav2_core"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "pluginlib"
)
ament_target_dependencies(
  astar_planning_node
  "rclcpp"
  "nav2_costmap_2d"
  "nav2_core"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "pluginlib"
)
# ament_target_dependencies(
#   astar_planner_ros
#   "rclcpp"
#   "nav2_costmap_2d"
#   "nav2_core"
#   "nav_msgs"
#   "geometry_msgs"
#   "tf2_ros"
#   "pluginlib"
# )
install(TARGETS astar_planner
  astar_planning_node
  # astar_planner_ros
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
  ${ament_INCLUDE_DIRS}
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
