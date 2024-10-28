## Copyright 2009-2022 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

MACRO(_SET_IF_EMPTY VAR VALUE)
  IF(NOT ${VAR})
    SET(${VAR} "${VALUE}")
  ENDIF()
ENDMACRO()

_SET_IF_EMPTY(FLAGS_SSE2  "-msse2")
_SET_IF_EMPTY(FLAGS_SSE42 "-msse4.2")
_SET_IF_EMPTY(FLAGS_AVX   "-mavx")
_SET_IF_EMPTY(FLAGS_AVX2  "-mf16c -mavx2 -mfma -mlzcnt -mbmi -mbmi2")
_SET_IF_EMPTY(FLAGS_AVX512 "-march=skx")

IF (NOT WIN32)
  OPTION(EMBREE_IGNORE_CMAKE_CXX_FLAGS "When enabled Embree ignores default CMAKE_CXX_FLAGS." ON)
  IF (EMBREE_IGNORE_CMAKE_CXX_FLAGS)
    SET(CMAKE_CXX_FLAGS "")
  ENDIF()
ENDIF()

GET_FILENAME_COMPONENT(SYCL_COMPILER_DIR ${CMAKE_CXX_COMPILER} PATH)
GET_FILENAME_COMPONENT(SYCL_COMPILER_NAME ${CMAKE_CXX_COMPILER} NAME_WE)
IF (NOT SYCL_COMPILER_NAME STREQUAL "clang++")
  SET(SYCL_ONEAPI TRUE)
  IF (SYCL_COMPILER_NAME STREQUAL "icx" OR SYCL_COMPILER_NAME STREQUAL "icpx")
    SET(SYCL_ONEAPI_ICX TRUE)
  ELSE()
    SET(SYCL_ONEAPI_ICX FALSE)
  ENDIF()
  SET(STORE_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  SET(STORE_CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS})
  IF (NOT EMBREE_SYCL_SUPPORT)
    # if EMBREE_SYCL_SUPPORT is off we don't want the -fsycl flags
    SET(CMAKE_CXX_FLAGS ${STORE_CMAKE_CXX_FLAGS})
    SET(CMAKE_CXX_LINK_FLAGS ${STORE_CMAKE_CXX_LINK_FLAGS})
  ENDIF()
ELSE()
  SET(SYCL_ONEAPI FALSE)
  ADD_DEFINITIONS(-D__INTEL_LLVM_COMPILER)
ENDIF()

IF (EMBREE_SYCL_SUPPORT)

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sycl")   # makes dpcpp compiler compatible with clang++
  
  SET(CMAKE_CXX_FLAGS_SYCL "-fsycl -fsycl-unnamed-lambda -Xclang -fsycl-allow-func-ptr")
  SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -Wno-mismatched-tags -Wno-pessimizing-move -Wno-reorder -Wno-unneeded-internal-declaration -Wno-delete-non-abstract-non-virtual-dtor -Wno-dangling-field -Wno-unknown-pragmas -Wno-logical-op-parentheses")
  
  IF (SYCL_ONEAPI_ICX AND WIN32)
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} /debug:none")    # FIXME: debug information generation takes forever in SYCL
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} /DNDEBUG")    # FIXME: debug information generation takes forever in SYCL
  ELSE()
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -g0")            # FIXME: debug information generation takes forever in SYCL
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -UDEBUG -DNDEBUG") # FIXME: assertion still not working in SYCL
  ENDIF()
  
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning
  SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning

  IF (WIN32)
    SET(SYCL_COMPILER_LIB_DIR "${SYCL_COMPILER_DIR}/../lib")
    IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
      file(GLOB SYCL_LIB RELATIVE ${SYCL_COMPILER_LIB_DIR}
           ${SYCL_COMPILER_LIB_DIR}/sycld.lib
           ${SYCL_COMPILER_LIB_DIR}/sycl[0-9]d.lib
           ${SYCL_COMPILER_LIB_DIR}/sycl[0-9][0-9]d.lib)
    ELSE()
      file(GLOB SYCL_LIB RELATIVE ${SYCL_COMPILER_LIB_DIR}
           ${SYCL_COMPILER_LIB_DIR}/sycl.lib
           ${SYCL_COMPILER_LIB_DIR}/sycl[0-9].lib
           ${SYCL_COMPILER_LIB_DIR}/sycl[0-9][0-9].lib)
    ENDIF()
    LIST(GET SYCL_LIB 0 SYCL_LIB)
    GET_FILENAME_COMPONENT(SYCL_LIB_NAME ${SYCL_LIB} NAME_WE)
  ELSE()
    SET(SYCL_LIB_NAME "sycl")
  ENDIF()

  SET(CMAKE_LINK_FLAGS_SYCL "-fsycl")
  
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableOCLNoInlineAttr=0")                                # enabled __noinline
  #LIST(APPEND CMAKE_IGC_OPTIONS "ControlKernelTotalSize=0")
  #LIST(APPEND CMAKE_IGC_OPTIONS "SubroutineThreshold=110000")        # Minimal kernel size to enable subroutines
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableUnmaskedFunctions=1")         # enables unmasked functions
  #LIST(APPEND CMAKE_IGC_OPTIONS "ByPassAllocaSizeHeuristic=64")      # puts small arrays into registers
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableIndirectCallOptimization=0")  # Enables inlining indirect calls by comparing function addresses
  #LIST(APPEND CMAKE_IGC_OPTIONS "FunctionControl=0")                 # 0 = default, 1 = inline, 2 = subroutine, 3 = stackcall, 4 = keep indirect calls
  #LIST(APPEND CMAKE_IGC_OPTIONS "forceGlobalRA=1")                   # "force global register allocator
  #LIST(APPEND CMAKE_IGC_OPTIONS "TotalGRFNum=128")                   # Total GRF used for register allocation
  #LIST(APPEND CMAKE_IGC_OPTIONS "GRFNumToUse=64")                   # "Set the number of general registers to use (64 to totalGRFNum)
  #LIST(APPEND CMAKE_IGC_OPTIONS "ReplaceIndirectCallWithJmpi=1")     # Replace indirect call with jmpi instruction (HW WA)
  #LIST(APPEND CMAKE_IGC_OPTIONS "DisableUniformAnalysis=1")          # Setting this to 1/true adds a compiler switch to disable uniform_analysis
  #LIST(APPEND CMAKE_IGC_OPTIONS "DisableLoopUnroll=1")               # Setting this to 1/true adds a compiler switch to disable loop unrolling
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableStatelessToStatefull=0")      #  Enable Stateless To Statefull transformation for global and constant address space in OpenCL kernels
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableRecursionOpenCL=1")           # Enable recursion with OpenCL user functions
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnableAdvMemOpt=0")                 # Enable advanced memory optimization
  #LIST(APPEND CMAKE_IGC_OPTIONS "UniformMemOptLimit=512")            # "Limit of uniform memory optimization in bits
  #LIST(APPEND CMAKE_IGC_OPTIONS "EnablePreemption=0")                 # Enable generating preeemptable code (SKL+)
  #LIST(APPEND CMAKE_IGC_OPTIONS "AllowSubroutineAndInirectdCalls=1")  # Allow subroutine in the presence of indirect calls
  #LIST(APPEND CMAKE_IGC_OPTIONS "AllocaRAPressureThreshold=0")        # The threshold for the register pressure potential (this reduces amount of spilling!)
  #LIST(APPEND CMAKE_IGC_OPTIONS "AssumeInt64Support=0")               # Architecture with partial int64 still promote uniform arrays to registers
  LIST(APPEND CMAKE_IGC_OPTIONS "VISAOptions=-scratchAllocForStackInKB 128 ")  # this works around some IGC bug in spill compression
  
  IF (CMAKE_BUILD_TYPE STREQUAL "Debug") # to allow printf inside indirectly callable function
    LIST(APPEND CMAKE_IGC_OPTIONS "ForceInlineStackCallWithImplArg=0")
    LIST(APPEND CMAKE_IGC_OPTIONS "EnableGlobalStateBuffer=1")   
  ENDIF()
  
  STRING(REPLACE ";" "," CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS}")

  IF (EMBREE_SYCL_AOT_DEVICE_REVISION GREATER 0)
    SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -revision_id ${EMBREE_SYCL_AOT_DEVICE_REVISION}")        # Enable this to override the stepping/RevId
  ENDIF()
  
  SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -cl-intel-greater-than-4GB-buffer-required")      # enables support for buffers larger than 4GB
  IF (EMBREE_SYCL_LARGEGRF)
    SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -internal_options -cl-intel-256-GRF-per-thread")          # large GRF mode
  ENDIF()
  SET(CMAKE_OCL_OTHER_OPTIONS "${CMAKE_OCL_OTHER_OPTIONS} -cl-intel-force-global-mem-allocation -cl-intel-no-local-to-generic")
  #SET(CMAKE_OCL_OTHER_OPTIONS "${CMAKE_OCL_OTHER_OPTIONS} -cl-intel-private-memory-minimal-size-per-thread 8192")
  
  IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
    SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64")
  ELSE()
    SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64,spir64_gen")
  ENDIF()
  
  SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64 \"${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
  
  IF (NOT EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
    SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_LINK_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64_gen \"-device ${EMBREE_SYCL_AOT_DEVICES} ${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
  ENDIF()
 
  IF (EMBREE_SYCL_DBG)
    SET(CMAKE_CXX_FLAGS_SYCL_AOT "-g")
  ENDIF()

  SET(CMAKE_CXX_FLAGS_SYCL  "${CMAKE_CXX_FLAGS_SYCL}  ${CMAKE_CXX_FLAGS_SYCL_AOT}")
  SET(CMAKE_LINK_FLAGS_SYCL "${CMAKE_LINK_FLAGS_SYCL} ${CMAKE_LINK_FLAGS_SYCL_AOT}")


  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pessimizing-move") # disabled: warning: moving a temporary object prevents copy elision [-Wpessimizing-move]

  IF (SYCL_ONEAPI_ICX AND WIN32)
    IF (${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL 2024.0)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I\"${SYCL_COMPILER_DIR}/../opt/compiler/include/sycl\" -I\"${SYCL_COMPILER_DIR}/../opt/compiler/include/sycl/sycl\"")       # disable warning from SYCL header
    ENDIF()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I\"${SYCL_COMPILER_DIR}/../include/sycl\" -I\"${SYCL_COMPILER_DIR}/../include/\"")       # disable warning from SYCL header
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qstd=c++17")
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    IF (SYCL_ONEAPI_ICX AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL 2024.0)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem \"${SYCL_COMPILER_DIR}/../opt/compiler/include/sycl\" -isystem \"${SYCL_COMPILER_DIR}/../opt/compiler/include/sycl/sycl\"")       # disable warning from SYCL header
    ENDIF()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem \"${SYCL_COMPILER_DIR}/../include/sycl\" -isystem \"${SYCL_COMPILER_DIR}/../include/\"")       # disable warning from SYCL header
  ENDIF()

  # enable C++17 features
  IF (SYCL_ONEAPI_ICX AND WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qstd=c++17")
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  ENDIF()
ENDIF(EMBREE_SYCL_SUPPORT)

IF(SYCL_ONEAPI_ICX)
  IF (WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Qno-intel-lib")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Qno-intel-lib")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Qimf-use-svml:false")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Qimf-use-svml:false")
    SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} /Qno-intel-lib")
    SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} /Qno-intel-lib")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /Qoption,link,/DEPENDENTLOADFLAG:0x2000")
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /Qoption,link,/DEPENDENTLOADFLAG:0x2000")
  ELSE()
    SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -static-intel")
    SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -static-intel")
    #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fimf-use-svml=false")
    #SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fimf-use-svml=false")
    IF (NOT EMBREE_SYCL_SUPPORT)
      SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -no-intel-lib")
      SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -no-intel-lib")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-intel-lib")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -no-intel-lib")
    ENDIF()
  ENDIF()
ENDIF()

IF (EMBREE_STACK_PROTECTOR)
  IF (SYCL_ONEAPI_ICX AND WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GS")           # protects against return address overrides
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector") # protects against return address overrides
  ENDIF()
ENDIF()
MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
  IF (EMBREE_STACK_PROTECTOR)
    IF (SYCL_ONEAPI_ICX AND WIN32)
      SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "/GS-")
    ELSE()
      SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "-fno-stack-protector")
    ENDIF()
  ENDIF()
ENDMACRO()

IF (SYCL_ONEAPI_ICX AND WIN32)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")   # makes dpcpp compiler compatible with clang++
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")        # catch C++ exceptions only and extern "C" functions never throw a C++ exception
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR")          # enable runtime type information (on by default)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -fcxx-exceptions") # enable C++ exceptions in Clang
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Gy")          # package individual functions
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")         # makes all symbols hidden by default
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility-inlines-hidden") # makes all inline symbols hidden by default
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")        # disables strict aliasing rules
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-tree-vectorize")         # disable auto vectorizer
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")         # perform extra security checks for some standard library calls
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")               # treat char as signed on all processors, including ARM
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")                       # enables most warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat -Wformat-security")  # enables string format vulnerability warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-model=precise")   # makes dpcpp compiler compatible with clang++
ENDIF()

IF (WIN32)

  IF (NOT EMBREE_SYCL_SUPPORT)
    IF (SYCL_ONEAPI_ICX)
      IF (${MSVC_VERSION} VERSION_GREATER_EQUAL 1916)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qstd=c++14")
      ELSE()
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qstd=c++11")
      ENDIF()
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Oi")
    ELSE()
      IF (${MSVC_VERSION} VERSION_GREATER_EQUAL 1916)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
      ELSE()
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      ENDIF()
    ENDIF()
  ENDIF()

  INCLUDE(msvc_post)

  # workaround for file encoding problems of kernels/embree.rc found here https://gitlab.kitware.com/cmake/cmake/-/issues/18311
  set(CMAKE_NINJA_CMCLDEPS_RC OFF)

ELSE()

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")                     # enables support for more secure position independent execution
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")                       # generate position independent code suitable for shared libraries
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")                       # generate position independent code suitable for shared libraries

  OPTION(EMBREE_ADDRESS_SANITIZER "Enabled CLANG address sanitizer." OFF)

  IF (EMBREE_ADDRESS_SANITIZER)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer -fno-optimize-sibling-calls")
  ENDIF()

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

  IF(EMBREE_SYCL_SUPPORT)
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "")
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "${CMAKE_CXX_FLAGS_RELWITHASSERT} -DDEBUG")         # enable assertions
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "${CMAKE_CXX_FLAGS_RELWITHASSERT} -O3")             # enable full optimizations
  ENDIF(EMBREE_SYCL_SUPPORT)

  IF (NOT EMBREE_ADDRESS_SANITIZER) # for address sanitizer this causes link errors
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined") # issues link error for undefined symbols in shared library
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack")     # we do not need an executable stack
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z relro -z now")    # re-arranges data sections to increase security
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")           # we do not need an executable stack
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z relro -z now")          # re-arranges data sections to increase security
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                     # enables position independent execution for executable
  ENDIF()

ENDIF()
