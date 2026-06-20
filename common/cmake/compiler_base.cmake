## Copyright 2009-2022 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# Common compiler configuration macros shared between clang.cmake, dpcpp.cmake and gnu.cmake

MACRO(_SET_IF_EMPTY VAR VALUE)
  IF(NOT ${VAR})
    SET(${VAR} "${VALUE}")
  ENDIF()
ENDMACRO()

# Common security and visibility flags for non-Windows platforms
MACRO(APPLY_COMMON_COMPILER_FLAGS)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")                       # enables most warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat -Wformat-security")  # enables string format vulnerability warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")               # treat char as signed on all processors, including ARM
  IF (NOT APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")                     # enables support for more secure position independent execution
  ENDIF()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")                       # generate position independent code suitable for shared libraries
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")                       # generate position independent code suitable for shared libraries
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")         # makes all symbols hidden by default
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility-inlines-hidden") # makes all inline symbols hidden by default
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")        # disables strict aliasing rules
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-tree-vectorize")         # disable auto vectorizer
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")         # perform extra security checks for some standard library calls
ENDMACRO()

# Stack protector configuration
MACRO(CONFIGURE_STACK_PROTECTOR)
  IF (EMBREE_STACK_PROTECTOR)
    IF (WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GS")           # protects against return address overrides
    ELSEIF (NOT WIN32)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector") # protects against return address overrides
    ELSEIF (WIN32)
      # MinGW or Clang on Windows - use GNU-style flag
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
    ENDIF()
  ELSEIF (WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GS-")          # do not protect against return address overrides
  ENDIF()
ENDMACRO()

MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
  IF (EMBREE_STACK_PROTECTOR)
    IF (WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
      SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "/GS-")
    ELSE ()
      SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "-fno-stack-protector")
    ENDIF()
  ENDIF()
ENDMACRO()

# Debug/Release optimization flags (non-Windows)
MACRO(APPLY_OPTIMIZATION_FLAGS)
  SET(CMAKE_CXX_FLAGS_DEBUG "")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")              # generate debug information
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")         # enable assertions
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTBB_USE_DEBUG") # configure TBB in debug mode
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")             # enable full optimizations

  SET(CMAKE_CXX_FLAGS_RELEASE "")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")     # disable assertions
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")          # enable full optimizations

  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")              # generate debug information
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DNDEBUG")        # disable assertions
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")             # enable full optimizations
ENDMACRO()

# macOS-specific linker flags
MACRO(APPLY_MACOS_FLAGS)
  IF (APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7")   # makes sure code runs on older MacOSX versions
    # Only use libc++ for Clang-based compilers; GCC on macOS doesn't support it
    IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")             # link against libc++ which supports C++11 features
    ENDIF()
  ENDIF()
ENDMACRO()

# Linux/ELF-specific linker flags (not for Windows or macOS)
MACRO(APPLY_LINUX_LINKER_FLAGS)
  IF (NOT APPLE AND NOT WIN32 AND NOT EMBREE_ADDRESS_SANITIZER AND NOT EMSCRIPTEN)
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined") # issues link error for undefined symbols in shared library
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                     # enables position independent execution for executable
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z relro -z now")    # re-arranges data sections to increase security
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack")     # we do not need an executable stack
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z relro -z now")          # re-arranges data sections to increase security
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")           # we do not need an executable stack
  ENDIF()
ENDMACRO()
