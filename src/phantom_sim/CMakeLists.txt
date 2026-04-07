cmake_minimum_required(VERSION 3.16)
project(phantom_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(phantom_model REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Simulator node -----------------------------------------------------------
add_executable(simulator_node src/simulator_node.cpp)
target_link_libraries(simulator_node phantom_model::robot_model Eigen3::Eigen)
ament_target_dependencies(simulator_node
    rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros ament_index_cpp)

# --- Test executable (not installed) ------------------------------------------
add_executable(test_model src/test_model.cpp)
target_link_libraries(test_model phantom_model::robot_model Eigen3::Eigen)

# --- Install ------------------------------------------------------------------
install(TARGETS simulator_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz plotjuggler
        DESTINATION share/${PROJECT_NAME})

ament_package()
