cmake_minimum_required(VERSION 3.8)
project(behavior_tree_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)

# Other Executables
add_executable(bt_node src/correct_direction_node.cpp)
target_include_directories(bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_node rclcpp behaviortree_cpp_v3 std_msgs)

# Install libraries and executables
install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# Remove the following block if you don't have a trees directory
# install(DIRECTORY trees/
#   DESTINATION share/${PROJECT_NAME}/trees
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
