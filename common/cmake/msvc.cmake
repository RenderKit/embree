## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(FLAGS_SSE2  "/D__SSE__ /D__SSE2__")
SET(FLAGS_SSE42 "${FLAGS_SSE2} /D__SSE3__ /D__SSSE3__ /D__SSE4_1__ /D__SSE4_2__")
SET(FLAGS_AVX   "${FLAGS_SSE42} /arch:AVX")
SET(FLAGS_AVX2  "${FLAGS_SSE42} /arch:AVX2")
SET(FLAGS_AVX512  "${FLAGS_AVX2} /arch:AVX512")
SET(FLAGS_APX  "${FLAGS_AVX512} /arch:AVX10.2 /vlen=512 /feature:APX")

SET(COMMON_CXX_FLAGS "")
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /EHsc")        # catch C++ exceptions only and extern "C" functions never throw a C++ exception
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /MP")          # compile source files in parallel
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GR")          # enable runtime type information (on by default)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /Gy")          # package individual functions
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /W4")          # enable highest practical warning level
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /WX")          # treat warnings as errors
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4100")      # disable: unreferenced formal parameter (intentional in template/virtual code)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4127")      # disable: conditional expression is constant (intentional in template/SIMD code)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4201")      # disable: nonstandard extension used: nameless struct/union (intentional for SIMD vec types)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4244")      # disable: conversion from X to Y, possible loss of data (intentional in SIMD/geometry math)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4267")      # disable: conversion from size_t to smaller type (intentional in index arithmetic)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4324")      # disable: structure was padded due to alignment specifier (intentional for SIMD alignment)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4512")      # disable: assignment operator could not be generated (C++03 compat, types with const members)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4714")      # disable: function marked __forceinline not inlined (compiler decision, not an error)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4702")      # disable: unreachable code (false-positive in heavily-inlined/templated code)
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4800")      # disable: implicit conversion from int to bool (intentional in flag/mask code)
IF (EMBREE_STACK_PROTECTOR)
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS")          # protects against return address overrides
ELSE()
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS-")          # do not protect against return address overrides
ENDIF()
MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
  IF (EMBREE_STACK_PROTECTOR)
    SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "/GS-")
  ENDIF()
ENDMACRO()

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEPENDENTLOADFLAG:0x2000")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /DEPENDENTLOADFLAG:0x2000")

# Remove CMake-injected /W3 so our /W4 is unambiguous
STRING(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COMMON_CXX_FLAGS}")
STRING(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})         # disable native runtime checks
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DDEBUG")                     # enables assertions
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DTBB_USE_DEBUG")             # configures TBB in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Ox")                         # enable full optimizations
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")                         # inline intrinsic functions
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")        # generate debug information
SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG")  # generate debug information

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")                       # enable full optimizations
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi")                       # inline intrinsic functions

SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COMMON_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Ox")                      # enable full optimizations
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Oi")                      # inline intrinsic functions
SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")        # generate debug information
SET(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")  # generate debug information

SET(SECURE_LINKER_FLAGS "")
SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /NXCompat")    # compatible with data execution prevention (on by default)
SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /DynamicBase") # random rebase of executable at load time
IF (CMAKE_SIZEOF_VOID_P EQUAL 4)
  SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /SafeSEH")     # invoke known exception handlers (Win32 only, x64 exception handlers are safe by design)
ENDIF()
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")

SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4217")  # locally defined symbol XXX imported in function YYY (happens as the ISPC API layer uses exported library functions)
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /IGNORE:4049")  # warning LNK4049: locally defined symbol _rtcOccluded1M imported

INCLUDE(msvc_post)

