cmake_minimum_required(VERSION 3.8)
project(astar_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(astar_planning_node src/astar_planning_node.cpp)
target_include_directories(astar_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(astar_planning_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  astar_planning_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
)

install(TARGETS astar_planning_node
  DESTINATION lib/${PROJECT_NAME})

# add_executable(map_sub_node src/map_sub_node.cpp)
# target_include_directories(map_sub_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(map_sub_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   map_sub_node
#   "rclcpp"
#   "geometry_msgs"
#   "nav_msgs"
# )

# install(TARGETS map_sub_node
#   DESTINATION lib/${PROJECT_NAME})


# add_executable(goal_sub_node src/goal_sub_node.cpp)
# target_include_directories(goal_sub_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(goal_sub_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   goal_sub_node
#   "rclcpp"
#   "geometry_msgs"
#   "nav_msgs"
# )
  
# install(TARGETS goal_sub_node
#   DESTINATION lib/${PROJECT_NAME})
  
# add_executable(init_sub_node src/init_sub_node.cpp)
# target_include_directories(init_sub_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(init_sub_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   init_sub_node
#   "rclcpp"
#   "geometry_msgs"
#   "nav_msgs"
# )

# install(TARGETS init_sub_node
#   DESTINATION lib/${PROJECT_NAME})
    
      
  
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
