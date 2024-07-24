cmake_minimum_required(VERSION 3.12)
project(pyembree)

# Use pybind11 from github source
include(FetchContent)
message("-- download and build pybind11 from source")
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY "https://github.com/pybind/pybind11.git"
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pybind11"
)
FetchContent_MakeAvailable(pybind11)

# Declare our python embree module
pybind11_add_module(pyembree
    src/bindings.cpp
    #pyembree.cpp
    #intersect_callback_guard.cpp
    #rtcore_common.cpp
    #rtcore_device.cpp
    #rtcore_buffer.cpp
    #rtcore_ray.cpp
    #rtcore_geometry.cpp
    #rtcore_scene.cpp
    #rtcore_builder.cpp
    #rtcore_quaternion.cpp
)