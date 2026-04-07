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

# --- Base controller library --------------------------------------------------
add_library(base_controller SHARED
    src/base_controller.cpp
)
target_include_directories(base_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(base_controller
    phantom_model::robot_model
    Eigen3::Eigen
)
ament_target_dependencies(base_controller
    rclcpp std_msgs sensor_msgs ament_index_cpp)

# --- Gravity compensation controller -----------------------------------------
add_executable(gravity_comp_node src/gravity_comp_node.cpp)
target_link_libraries(gravity_comp_node base_controller)
ament_target_dependencies(gravity_comp_node rclcpp)

# --- Install ------------------------------------------------------------------
install(TARGETS gravity_comp_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS base_controller
        DESTINATION lib
        EXPORT phantom_controllers_targets)

install(DIRECTORY include/
        DESTINATION include)

install(EXPORT phantom_controllers_targets
        FILE phantom_controllersTargets.cmake
        NAMESPACE phantom_controllers::
        DESTINATION lib/cmake/${PROJECT_NAME})

ament_package()
