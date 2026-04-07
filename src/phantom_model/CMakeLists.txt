cmake_minimum_required(VERSION 3.16)
project(phantom_model)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# --- CasADi ------------------------------------------------------------------
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

# --- Robot model library (no ROS2 dependency) ---------------------------------
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
    Eigen3::Eigen
)

# --- Install ------------------------------------------------------------------
install(TARGETS robot_model
        DESTINATION lib
        EXPORT phantom_model_targets)

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

install(EXPORT phantom_model_targets
        FILE phantom_modelTargets.cmake
        NAMESPACE phantom_model::
        DESTINATION lib/cmake/${PROJECT_NAME})

ament_export_targets(phantom_model_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(casadi yaml-cpp Eigen3)
ament_export_include_directories(include)

ament_package()
