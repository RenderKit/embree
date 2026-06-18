## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(compiler_base)

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
ENDIF()

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                     # enables position independent execution for executable
