## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(embree_tests ../../kernels/embree.rc
  embree_tests.cpp
  kernels/kernels_tests.cpp
  common/common_tests.cpp)
TARGET_LINK_LIBRARIES(embree_tests tasking sys math embree)

if (EMSCRIPTEN)
    # Use "-s ALLOW_MEMORY_GROWTH=1" to allow the WASM heap to grow.

    # Use "-s PROXY_TO_PTHREAD=1" to move program execution to a worker thread, leaving the main
    # thread available to respond to requests for more worker threads. Without this flag, we'd need
    # to prepopulate a thread pool with enough threads for every unit test (with something like
    # "-s PTHREAD_POOL_SIZE=40"), otherwise the main thread would block as soon another thread is
    # launched. See https://emscripten.org/docs/porting/pthreads.html#additional-flags and
    # https://github.com/emscripten-core/emscripten/blob/main/src/settings.js#L1019.

    # Use "-s EXIT_RUNTIME=1" to exit the Node.js process when the main thread completes. Otherwise,
    # any worker threads (even completed threads) will keep the process alive. See
    # https://github.com/emscripten-core/emscripten/blob/main/src/settings.js#L91.

    # Use "-s NODERAWFS=1" to allow tests running on Node.js to access the system's files (through
    # Emscripten's "raw filesystem" backend). This is used by several unit tests to read test data.
    # See https://github.com/emscripten-core/emscripten/blob/main/src/settings.js#L898.

    # Use "-Wno-pthreads-mem-growth" to silence the warning "USE_PTHREADS + ALLOW_MEMORY_GROWTH may
    # run non-wasm code slowly, see https://github.com/WebAssembly/design/issues/1271". Unit tests
    # don't run much (if any) non-wasm code.

    TARGET_LINK_OPTIONS(embree_tests PUBLIC
    "SHELL:-s ASSERTIONS=1"
    "SHELL:-s ALLOW_MEMORY_GROWTH=1"
    "SHELL:-s PROXY_TO_PTHREAD=1"
    "SHELL:-s EXIT_RUNTIME=1"
    -Wno-pthreads-mem-growth
  )
endif()

SET_PROPERTY(TARGET embree_tests PROPERTY FOLDER tutorials)
SET_PROPERTY(TARGET embree_tests APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_tests DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(embree_tests)

IF (BUILD_TESTING)

  ADD_TEST(NAME embree_tests COMMAND embree_tests)
  SET_TESTS_PROPERTIES(embree_tests PROPERTIES TIMEOUT 7000)

ENDIF()
