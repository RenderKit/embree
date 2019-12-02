
project("Test SYCL with shared libraries" CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsycl")

add_library(mylib SHARED lib.cpp)

add_executable(myapp main.cpp)

target_link_libraries(myapp mylib)
