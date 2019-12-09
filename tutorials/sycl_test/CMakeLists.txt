
ADD_LIBRARY(sycl_test_static_lib STATIC sycl_test_static_lib.cpp)

ADD_EXECUTABLE(sycl_test sycl_test.cpp)
TARGET_LINK_LIBRARIES(sycl_test sycl_test_static_lib)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -foffload-static-lib=../../libsycl_test_static_lib.a")
#SET_TARGET_PROPERTIES(sycl_test PROPERTIES COMPILE_FLAGS "${FLAGS_SSE42}")
