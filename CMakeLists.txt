cmake_minimum_required(VERSION 3.5)
project(autoware_v2x_trajectory)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
find_package(ament_cmake_auto REQUIRED)
set(VANETZA_INSTALL ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
find_package(Vanetza REQUIRED)
find_package(GeographicLib 1.37 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)

include_directories(
  include
)

ament_auto_add_library(autoware_v2x_trajectory SHARED
  src/v2x_node.cpp
  src/v2x_app.cpp
  src/application.cpp
  src/cpm_application.cpp
  src/ethernet_device.cpp
  src/link_layer.cpp
  src/raw_socket_link.cpp
  src/router_context.cpp
  src/dcc_passthrough.cpp
  src/time_trigger.cpp
  src/positioning.cpp
  src/security.cpp
  src/trajectory_visualizer_node.cpp
)

target_link_libraries(autoware_v2x_trajectory
  Vanetza::vanetza
  ${GeographicLib_LIBRARIES}
  Boost::thread sqlite3
)

# 新しいビジュアライザノードの実行可能ファイルを追加
add_executable(trajectory_visualizer_node src/trajectory_visualizer_node.cpp)

# 依存パッケージとのリンク設定
ament_target_dependencies(trajectory_visualizer_node
  rclcpp
  autoware_auto_planning_msgs
  visualization_msgs
)

install(TARGETS
  autoware_v2x_trajectory
  trajectory_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(autoware_v2x_trajectory
  PLUGIN "v2x_trajectory::V2XNode"
  EXECUTABLE autoware_v2x_trajectory_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)