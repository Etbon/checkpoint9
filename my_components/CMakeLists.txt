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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# service interface checkpoint 9
find_package(attach_shelf REQUIRED)

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
add_library(attach_server_component SHARED
  src/attach_server.cpp
)
add_library(attach_client_component SHARED
  src/attach_client.cpp

)

# attach include
include_directories(include)

# export symbols
target_compile_definitions(pre_approach_component PRIVATE
  "COMPOSITION_BUILDING_DLL"
)

target_compile_definitions(attach_server_component PRIVATE
  "COMPOSITION_BUILDING_DLL"
)

target_compile_definitions(attach_client_component PRIVATE
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

ament_target_dependencies(attach_server_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  attach_shelf
)

ament_target_dependencies(attach_client_component
  rclcpp
  rclcpp_components
  attach_shelf

)

# register plugin automaica / manual
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")

set(node_plugins "${node_plugins}my_components::PreApproach; $<TARGET_FILE:pre_approach_component>\n")
set(node_plugins "${node_plugins}my_components::AttachServer; $<TARGET_FILE:attach_server_component>\n")
set(node_plugins "${node_plugins}my_components::AttachClient; $<TARGET_FILE:attach_client_component>\n")

# ------------- Manual Executabel -------------
add_executable(attach_server_manual 
	src/attach_server_manual.cpp
)

target_include_directories(attach_server_manual 
	PRIVATE include
)

target_link_libraries(attach_server_manual
	attach_server_component
)

ament_target_dependencies(attach_server_manual
	rclcpp
	geometry_msgs
	sensor_msgs
	tf2
	tf2_geometry_msgs
)

# install library 
install(TARGETS
  pre_approach_component
  attach_server_component
  attach_server_manual
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install RViz and lunach files
install(DIRECTORY
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()