cmake_minimum_required(VERSION 3.5)
project(pid)

# Specify C++11 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ament and other required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# # Add dynamic reconfigure configuration
# find_package(dynamic_reconfigure REQUIRED)
# generate_dynamic_reconfigure_options(
#     cfg/Pid.cfg
# )

# Include directories
include_directories(include)

# Create executables
add_executable(controller src/controller.cpp src/pid.cpp)
# add_executable(plant_sim src/plant_sim.cpp)
# add_executable(setpoint_node src/setpoint_node.cpp)
# add_executable(sim_time src/sim_time.cpp)
# add_executable(autotune src/autotune.cpp)

# Link libraries
ament_target_dependencies(controller rclcpp std_msgs Boost)
# ament_target_dependencies(plant_sim rclcpp std_msgs dynamic_reconfigure Boost)
# ament_target_dependencies(setpoint_node rclcpp std_msgs dynamic_reconfigure Boost)
# ament_target_dependencies(sim_time rclcpp std_msgs dynamic_reconfigure Boost)
# ament_target_dependencies(autotune rclcpp std_msgs dynamic_reconfigure Boost)

# Install targets
install(TARGETS controller # plant_sim setpoint_node sim_time autotune
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS pid/wheelchair_pid_helper.py # plant_sim setpoint_node sim_time autotune
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch pid
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY scripts/
#   DESTINATION share/${PROJECT_NAME}
# )

# Export package for ament
ament_package()
