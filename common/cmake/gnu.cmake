## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

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
  _SET_IF_EMPTY(FLAGS_AVX512 "-mavx512f -mavx512dq -mavx512cd -mavx512bw -mavx512vl -mf16c -mavx2 -mfma -mlzcnt -mbmi -mbmi2 -mprefer-vector-width=256")
ENDIF ()

OPTION(EMBREE_IGNORE_CMAKE_CXX_FLAGS "When enabled Embree ignores default CMAKE_CXX_FLAGS." ON)
IF (EMBREE_IGNORE_CMAKE_CXX_FLAGS)
  SET(CMAKE_CXX_FLAGS "")
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

IF (EMBREE_ARM)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")             # treat 'char' as 'signed char'
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flax-vector-conversions")  # allow lax vector type conversions
ENDIF (EMBREE_ARM)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")                       # enables most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat -Wformat-security")  # enables string format vulnerability warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-class-memaccess")        # disables clearing an object of type ‘XXX’ with no trivial copy-assignment; use assignment or value-initialization instead

# these prevent compile to optimize away security checks
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-overflow")            # assume that signed overflow occurs
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-delete-null-pointer-checks") # keep all checks for NULL pointers
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fwrapv")                         # this option instructs the compiler to assume that signed arithmetic overflow warps around.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")                   # treat char as signed on all processors, including ARM

IF (NOT WIN32)
  IF (NOT APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")                   # enables support for more secure position independent execution
  ENDIF()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")                     # generate position independent code suitable for shared libraries
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")                     # generate position independent code suitable for shared libraries
ENDIF ()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")                  # enables C++11 features
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")         # makes all symbols hidden by default
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility-inlines-hidden") # makes all inline symbols hidden by default
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")        # disables strict aliasing rules
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-tree-vectorize")         # disable auto vectorizer
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")         # perform extra security checks for some standard library calls
IF (EMBREE_STACK_PROTECTOR)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")           # protects against return address overrides
ENDIF()
MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
  IF (EMBREE_STACK_PROTECTOR)
    SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "-fno-stack-protector")
  ENDIF()
ENDMACRO()

SET(CMAKE_CXX_FLAGS_DEBUG "")
IF (EMBREE_ARM)
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsigned-char")             # treat 'char' as 'signed char'
ENDIF (EMBREE_ARM)
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")              # generate debug information
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")         # enable assertions
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTBB_USE_DEBUG") # configure TBB in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")             # enable full optimizations

SET(CMAKE_CXX_FLAGS_RELEASE "")
IF (EMBREE_ARM)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fsigned-char")             # treat 'char' as 'signed char'
ENDIF (EMBREE_ARM)
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")     # disable assertions
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")          # enable full optimizations

SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "")
IF (EMBREE_ARM)
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fsigned-char")             # treat 'char' as 'signed char'
ENDIF (EMBREE_ARM)
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")              # generate debug information
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DNDEBUG")        # disable assertions
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")             # enable full optimizations

IF (APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7")   # makes sure code runs on older MacOSX versions
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")             # link against libc++ which supports C++11 features
ELSE(APPLE)
  IF (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined") # issues link error for undefined symbols in shared library
    IF(NOT MINGW)
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack")     # we do not need an executable stack
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z relro -z now")    # re-arranges data sections to increase security
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")           # we do not need an executable stack
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z relro -z now")          # re-arranges data sections to increase security
    ENDIF ()
  ENDIF ()
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                       # enables position independent execution for executable
ENDIF(APPLE)

