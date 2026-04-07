cmake_minimum_required(VERSION 3.16)
project(phantom_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(phantom_model REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Vanilla BaseController executable ---------------------------------------
# `BaseController` is header-only (see include/phantom_controllers/base_controller.hpp).
# Each executable that uses it just includes the header and links the
# transitive dependencies directly — no shared library is needed.  Toggle
# gravity compensation via the `gravity_compensation` ROS parameter.
add_executable(base_controller_node src/base_controller_node.cpp)
target_include_directories(base_controller_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(base_controller_node
    phantom_model::robot_model
    Eigen3::Eigen
)
ament_target_dependencies(base_controller_node
    rclcpp std_msgs sensor_msgs ament_index_cpp)

# --- Install ------------------------------------------------------------------
install(TARGETS base_controller_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

# Header-only — let downstream packages (and future controllers in this
# package) get the include dir + transitive deps via find_package().
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    ament_index_cpp
    phantom_model
    Eigen3
)

ament_package()
