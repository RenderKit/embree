set(CMAKE_CXX_STANDARD 17)

include(ExternalProject)

# Make sure we have embree
if (NOT embree_DIR)
    message("-- building embree from source")
    option(EMBREE_INSTALL_DEPENDENCIES "always on for pyembree" ON)
    option(EMBREE_TUTORIALS "alway off for pyembree" OFF)
    set(embree_BINARY_DIR ${CMAKE_BINARY_DIR}/_deps/embree-build)
    message("bin dir    ${embree_BINARY_DIR}")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. ${embree_BINARY_DIR})

    message("EMBREE LIBS:   ${EMBREE_LIBRARIES}")
else()
    message("-- using embree from ${embree_DIR}")
    set(embree_BINARY_DIR "${embree_DIR}/../../../bin")
endif()


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

target_include_directories(pyembree PRIVATE ${CMAKE_INSTALL_PREFIX}/include)
target_link_libraries(pyembree PRIVATE embree)

if(EMBREE_SYCL_SUPPORT)
    add_compile_definitions(EMBREE_SYCL_SUPPORT=1)
    message("${CMAKE_MODULE_PATH}")
    message("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake")
    set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake" ${CMAKE_MODULE_PATH})
    message("${CMAKE_MODULE_PATH}")
    include(dpcpp)
endif()

install(TARGETS pyembree
        COMPONENT python
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_BINDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

message("install prefix    ${CMAKE_INSTALL_PREFIX}")