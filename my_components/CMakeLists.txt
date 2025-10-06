cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

# ------------- Component library -------------  

# build library  
add_library(pre_approach_component SHARED
  src/pre_approach.cpp
)

# attach include
include_directories(include)

# export symbols
target_compile_definitions(pre_approach_component PRIVATE
  "COMPOSITION_BUILDING_DLL"
)

# link depencencies
ament_target_dependencies(pre_approach_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
# register plugin automaica / manual
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")

set(node_plugins "${node_plugins}my_components::PreApproach; $<TARGET_FILE:pre_approach_component>\n")

# install library 
install(TARGETS
  pre_approach_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install RViz and lunach files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()