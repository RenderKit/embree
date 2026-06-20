## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(compiler_base)

MACRO(_SET_IF_EMPTY VAR VALUE)
  IF(NOT ${VAR})
    SET(${VAR} "${VALUE}")
  ENDIF()
ENDMACRO()

IF (EMBREE_ARM)
  SET(FLAGS_SSE2 "-D__SSE__ -D__SSE2__")
  SET(FLAGS_SSE42 "-D__SSE4_2__  -D__SSE4_1__")
  SET(FLAGS_AVX "-D__AVX__ -D__SSE4_2__  -D__SSE4_1__  -D__BMI__ -D__BMI2__ -D__LZCNT__")
  SET(FLAGS_AVX2 "-D__AVX2__ -D__AVX__ -D__SSE4_2__  -D__SSE4_1__  -D__BMI__ -D__BMI2__ -D__LZCNT__")
ELSE ()
  _SET_IF_EMPTY(FLAGS_SSE2  "-msse2")
  _SET_IF_EMPTY(FLAGS_SSE42 "-msse4.2")
  _SET_IF_EMPTY(FLAGS_AVX   "-mavx")
  _SET_IF_EMPTY(FLAGS_AVX2  "-mf16c -mavx2 -mfma -mlzcnt -mbmi -mbmi2")
  _SET_IF_EMPTY(FLAGS_AVX512 "-march=skylake-avx512")
ENDIF ()

OPTION(EMBREE_IGNORE_CMAKE_CXX_FLAGS "When enabled Embree ignores default CMAKE_CXX_FLAGS." ON)
IF (EMBREE_IGNORE_CMAKE_CXX_FLAGS)
  SET(CMAKE_CXX_FLAGS "")
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

IF (EMBREE_ARM)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flax-vector-conversions")  # allow lax vector type conversions
ENDIF (EMBREE_ARM)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-class-memaccess")        # disables clearing an object of type 'XXX' with no trivial copy-assignment; use assignment or value-initialization instead

# these prevent compile to optimize away security checks
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-overflow")            # assume that signed overflow occurs
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-delete-null-pointer-checks") # keep all checks for NULL pointers
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fwrapv")                         # this option instructs the compiler to assume that signed arithmetic overflow warps around.

APPLY_COMMON_COMPILER_FLAGS()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")                  # enables C++11 features
CONFIGURE_STACK_PROTECTOR()

APPLY_OPTIMIZATION_FLAGS()
APPLY_MACOS_FLAGS()

IF (NOT APPLE)
  IF (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined") # issues link error for undefined symbols in shared library
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack")     # we do not need an executable stack
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z relro -z now")    # re-arranges data sections to increase security
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")           # we do not need an executable stack
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z relro -z now")          # re-arranges data sections to increase security
  ENDIF ()
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                       # enables position independent execution for executable
ENDIF()
