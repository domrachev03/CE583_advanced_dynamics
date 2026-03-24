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
find_package(yaml-cpp REQUIRED)

# --- CasADi ------------------------------------------------------------------
# Try config-mode first (works when casadi ships its own .cmake files).
# Fall back to manual header + library search (e.g. conda-forge builds).
find_package(casadi QUIET CONFIG)
if(NOT casadi_FOUND)
    find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
              HINTS $ENV{CONDA_PREFIX}/include)
    find_library(CASADI_LIBRARY casadi
                 HINTS $ENV{CONDA_PREFIX}/lib)
    if(CASADI_INCLUDE_DIR AND CASADI_LIBRARY)
        message(STATUS "CasADi found manually: ${CASADI_LIBRARY}")
        add_library(casadi::casadi SHARED IMPORTED)
        set_target_properties(casadi::casadi PROPERTIES
            IMPORTED_LOCATION "${CASADI_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CASADI_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "CasADi not found. Install via: pixi add casadi")
    endif()
endif()

# --- Robot model library ------------------------------------------------------
add_library(robot_model SHARED
    src/robot_model.cpp
)
target_include_directories(robot_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_model
    casadi::casadi
    yaml-cpp::yaml-cpp
)

# --- Simulator node -----------------------------------------------------------
add_executable(simulator_node src/simulator_node.cpp)
target_link_libraries(simulator_node robot_model)
ament_target_dependencies(simulator_node
    rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros ament_index_cpp)

# --- Test executable (not installed) ------------------------------------------
add_executable(test_model src/test_model.cpp)
target_link_libraries(test_model robot_model)

# --- Install ------------------------------------------------------------------
install(TARGETS simulator_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS robot_model
        DESTINATION lib
        EXPORT phantom_sim_targets)

install(DIRECTORY config launch rviz
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

# Export robot_model so other packages (phantom_controllers) can link against it
install(EXPORT phantom_sim_targets
        FILE phantom_simTargets.cmake
        NAMESPACE phantom_sim::
        DESTINATION lib/cmake/${PROJECT_NAME})

ament_export_targets(phantom_sim_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(casadi yaml-cpp)
ament_export_include_directories(include)

ament_package()
