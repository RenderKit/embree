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
    #src/bindings.cpp
    src/pyembree.cpp
    src/intersect_callback_guard.cpp
    src/rtcore_common.cpp
    src/rtcore_device.cpp
    src/rtcore_buffer.cpp
    src/rtcore_ray.cpp
    src/rtcore_geometry.cpp
    src/rtcore_scene.cpp
    src/rtcore_builder.cpp
    src/rtcore_quaternion.cpp
)


# Make sure we have embree
if (NOT embree_DIR)
    message("-- building embree from source")
    option(EMBREE_INSTALL_DEPENDENCIES "always on for pyembree" ON)
    option(EMBREE_TUTORIALS "alway off for pyembree" OFF)
    set(embree_BINARY_DIR ${CMAKE_BINARY_DIR}/_deps/embree-build)
    message("bin dir    ${embree_BINARY_DIR}")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${embree_BINARY_DIR})

    message("EMBREE LIBS:   ${EMBREE_LIBRARIES}")
else()
    message("-- using embree from ${embree_DIR}")
    set(embree_BINARY_DIR "${embree_DIR}/../../../bin")
endif()

target_include_directories(pyembree PRIVATE ${CMAKE_INSTALL_PREFIX}/include)
target_link_libraries(pyembree PRIVATE embree)