set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
message("-- download and build pybind11 from source")
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY "https://github.com/pybind/pybind11.git"
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pybind11"
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(pyembree
    pyembree.cpp
    intersect_callback_guard.cpp
    rtcore_common.cpp
    rtcore_device.cpp
    rtcore_buffer.cpp
    rtcore_ray.cpp
    rtcore_geometry.cpp
    rtcore_scene.cpp
    rtcore_builder.cpp
    rtcore_quaternion.cpp
)

if (NOT embree_DIR)
    message("-- building embree from source")
    option(EMBREE_INSTALL_DEPENDENCIES ON)
    option(EMBREE_TUTORIALS OFF)
    set(embree_BINARY_DIR ${CMAKE_BINARY_DIR}/_deps/embree-build)
    message("bin dir    ${embree_BINARY_DIR}")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. ${embree_BINARY_DIR})
else()
    message("-- using embree from ${embree_DIR}")
    set(embree_BINARY_DIR "${embree_DIR}/../../../bin")
endif()


message("install prefix    ${CMAKE_INSTALL_PREFIX}")

target_include_directories(pyembree PRIVATE ${CMAKE_INSTALL_PREFIX}/include)
target_link_libraries(pyembree PRIVATE embree)

message("CMAKE_BINARY_DIR           ${CMAKE_BINARY_DIR}")
message("CMAKE_CURRENT_BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}")

if (WIN32)
    add_custom_command(TARGET pyembree POST_BUILD
        COMMAND ${CMAKE_COMMAND}  -Dembree_BINARY_DIR=${embree_BINARY_DIR} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -P ${CMAKE_CURRENT_LIST_DIR}/pack_pip.cmake
    )
endif()