cmake_minimum_required(VERSION 3.14)
project(crane_rrt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GantryCraneState.msg"
  "msg/CollisionStatus.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

include_directories(
  include
)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===============================
# 1. Nonlinear Crane Simulator Node
# ===============================
add_executable(simulator_node
  src/simulator_node_main.cpp
)
ament_target_dependencies(simulator_node
  rclcpp
  ament_index_cpp
  std_msgs
  yaml-cpp
)
target_link_libraries(simulator_node
  yaml-cpp
  Threads::Threads
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

# ===============================
# 2. Crane RViz2 Visualizer Node
# ===============================
add_executable(visualizer_node
  src/visualizer_node_main.cpp
)
ament_target_dependencies(visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
)
target_link_libraries(visualizer_node
  yaml-cpp
  Threads::Threads
  ${cpp_typesupport_target}
)

# ===============================
# 3. Arena RViz2 Visualizer Node
# ===============================
add_executable(arena_visualizer_node
  src/arena_visualizer_node_main.cpp
)
ament_target_dependencies(arena_visualizer_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  visualization_msgs
  std_msgs
)
target_link_libraries(arena_visualizer_node
  yaml-cpp
  Threads::Threads
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

# ===============================
# 4. Interactive RRT Exploration Node
# ===============================
add_executable(rrt_exploration_node
  src/rrt_exploration_node_main.cpp
)
ament_target_dependencies(rrt_exploration_node
  rclcpp
  ament_index_cpp
  std_msgs
  geometry_msgs
  visualization_msgs
  yaml-cpp
)
target_link_libraries(rrt_exploration_node
  yaml-cpp
  Threads::Threads
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

# ===============================
# 5. ECC 2027 Standalone Benchmark Suite
# ===============================
add_executable(ecc_benchmark
  src/ecc_benchmark.cpp
)
ament_target_dependencies(ecc_benchmark
  rclcpp
  ament_index_cpp
  std_msgs
  yaml-cpp
)
target_link_libraries(ecc_benchmark
  yaml-cpp
  Threads::Threads
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

install(TARGETS
  simulator_node
  visualizer_node
  arena_visualizer_node
  rrt_exploration_node
  ecc_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/crane_keyboard_gui.py
  scripts/rrt_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  arenas
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
