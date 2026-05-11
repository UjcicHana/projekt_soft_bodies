cmake_minimum_required(VERSION 3.31)
project(projekt_soft_bodies)

set(CMAKE_CXX_STANDARD 20)

# OPENGL
add_library(glad src/glad.c)
target_include_directories(glad PUBLIC external)

# GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# IMGUI
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)

set(IMGUI_SOURCES
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

include_directories(
        include
        ${GLFW_INCLUDE_DIRS}
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)


add_executable(projekt_soft_bodies
        src/main.cpp
        src/Particle.h
        src/Constraint.h
        src/Constraint.cpp
        src/Object.h
        src/Object.cpp
        src/SimulatedObject.h
        src/SimulatedObject.cpp
        ${IMGUI_SOURCES}
        src/Renderer.cpp
        src/Renderer.h
        src/Solver.cpp
        src/Solver.h)

target_link_libraries(projekt_soft_bodies
        glad
        ${GLFW_LIBRARIES}
        dl
        GL
        m
)



