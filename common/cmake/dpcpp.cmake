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
_SET_IF_EMPTY(FLAGS_NEON   "-D__SSE__ -D__SSE2__")

IF (WIN32)

  SET(COMMON_CXX_FLAGS "")
  #SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /EHsc")        # catch C++ exceptions only and extern "C" functions never throw a C++ exception
#  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /MP")          # compile source files in parallel
  #SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GR")          # enable runtime type information (on by default)
  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Xclang -fcxx-exceptions") # enable C++ exceptions in Clang
  #SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /w")          # disable all warnings
  #SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /Gy")          # package individual functions
  #IF (EMBREE_STACK_PROTECTOR)
  #  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS")          # protects against return address overrides
  #ELSE()
  #  SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /GS-")          # do not protect against return address overrides
  #ENDIF()
  MACRO(DISABLE_STACK_PROTECTOR_FOR_FILE file)
    IF (EMBREE_STACK_PROTECTOR)
      SET_SOURCE_FILES_PROPERTIES(${file} PROPERTIES COMPILE_FLAGS "/GS-")
    ENDIF()
  ENDMACRO()
  
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COMMON_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DDEBUG")                     # enables assertions
  #SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DTBB_USE_DEBUG")             # configures TBB in debug mode
  #SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Ox")                         # enable full optimizations
  #SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")                         # inline intrinsic functions
  #SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")        # generate debug information
  #SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG")  # generate debug information

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COMMON_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")                       # enable full optimizations
  #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi")                       # inline intrinsic functions
  
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COMMON_CXX_FLAGS}")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Ox")                      # enable full optimizations
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Oi")                      # inline intrinsic functions
  SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")        # generate debug information
  SET(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG")  # generate debug information
  
  SET(SECURE_LINKER_FLAGS "")
  #SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /NXCompat")    # compatible with data execution prevention (on by default)
  #SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /DynamicBase") # random rebase of executable at load time
  IF (CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(SECURE_LINKER_FLAGS "${SECURE_LINKER_FLAGS} /SafeSEH")     # invoke known exception handlers (Win32 only, x64 exception handlers are safe by design)
  ENDIF()
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${SECURE_LINKER_FLAGS}")

  IF (EMBREE_DPCPP_SUPPORT)

    GET_FILENAME_COMPONENT(SYCL_COMPILER_DIR ${CMAKE_CXX_COMPILER} PATH)
    GET_FILENAME_COMPONENT(SYCL_COMPILER_NAME ${CMAKE_CXX_COMPILER} NAME)

    IF (NOT SYCL_COMPILER_NAME STREQUAL "dpcpp")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning
    ENDIF()
    
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") # enables C++17 features
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sycl")   # makes dpcpp compiler compatible with clang++

    SET(CMAKE_CXX_FLAGS_SYCL "-Wno-mismatched-tags -Wno-pessimizing-move -Wno-reorder -Wno-unneeded-internal-declaration -Wno-delete-non-abstract-non-virtual-dtor -Wno-dangling-field -Wno-unknown-pragmas -Wno-logical-op-parentheses -fsycl -fsycl-unnamed-lambda -Xclang -fsycl-allow-func-ptr")

    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -g0")              # FIXME: debug information generation takes forever in SYCL
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -UDEBUG -DNDEBUG") # FIXME: assertion still not working in SYCL

    SET(CMAKE_LINK_FLAGS_SYCL "-lsycl -fsycl")

      SET(CMAKE_IGC_OPTIONS "EnableOCLNoInlineAttr=0")                                # enabled __noinline
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ControlKernelTotalSize=0")
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},SubroutineThreshold=110000")        # Minimal kernel size to enable subroutines
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableUnmaskedFunctions=1")         # enables unmasked functions
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ByPassAllocaSizeHeuristic=64")      # puts small arrays into registers
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableIndirectCallOptimization=0")  # Enables inlining indirect calls by comparing function addresses
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},FunctionControl=0")                 # 0 = default, 1 = inline, 2 = subroutine, 3 = stackcall, 4 = keep indirect calls
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},forceGlobalRA=1")                   # "force global register allocator
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},TotalGRFNum=128")                   # Total GRF used for register allocation
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},GRFNumToUse=64")                   # "Set the number of general registers to use (64 to totalGRFNum)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ReplaceIndirectCallWithJmpi=1")     # Replace indirect call with jmpi instruction (HW WA)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},DisableUniformAnalysis=1")          # Setting this to 1/true adds a compiler switch to disable uniform_analysis
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},DisableLoopUnroll=1")               # Setting this to 1/true adds a compiler switch to disable loop unrolling
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableStatelessToStatefull=0")      #  Enable Stateless To Statefull transformation for global and constant address space in OpenCL kernels
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableRecursionOpenCL=1")           # Enable recursion with OpenCL user functions
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableAdvMemOpt=0")                 # Enable advanced memory optimization
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},UniformMemOptLimit=512")            # "Limit of uniform memory optimization in bits
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnablePreemption=0")                 # Enable generating preeemptable code (SKL+)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AllowSubroutineAndInirectdCalls=1")  # Allow subroutine in the presence of indirect calls
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AllocaRAPressureThreshold=0")        # The threshold for the register pressure potential (this reduces amount of spilling!)

      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AssumeInt64Support=0")               # Architecture with partial int64 still promote uniform arrays to registers
      SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},VISAOptions=-scratchAllocForStackInKB 128 -nospillcompression")  # this works around some IGC bug in spill compression

      IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
        SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ForceInlineStackCallWithImplArg=0,EnableGlobalStateBuffer=1")   # to allow printf inside indirectly callable function
      ENDIF()
            
      IF (EMBREE_DPCPP_AOT_DEVICE_REVISION GREATER 0)
        SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -revision_id ${EMBREE_DPCPP_AOT_DEVICE_REVISION}")        # Enable this to override the stepping/RevId, default is a0 = 0, b0 = 1, c0 = 2, so on...        
      ENDIF()
      
      # all the sycl libraries that we have to link against
      if (EXISTS "${SYCL_COMPILER_DIR}/../lib/sycl.lib")
        set(SYCL_LIBS "-L${SYCL_COMPILER_DIR}/../lib -lsycl")
      else()
        message(FATAL_ERROR "Library sycl.lib not found in ${SYCL_COMPILER_DIR}/../lib")
      endif()

      SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -cl-intel-greater-than-4GB-buffer-required")      # enables support for buffers larger than 4GB
      #SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -ze-opt-large-register-file")                     # large GRF mode
      SET(CMAKE_OCL_OTHER_OPTIONS "${CMAKE_OCL_OTHER_OPTIONS} -cl-intel-force-global-mem-allocation -cl-intel-no-local-to-generic")

      IF (EMBREE_DPCPP_AOT_DEVICES STREQUAL "none")
        SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64 \"${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
      ELSE()
        #SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64,spir64_gen")
        SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64_gen")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT}")
      ENDIF()

      #SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64 \"${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
            
      IF (NOT EMBREE_DPCPP_AOT_DEVICES STREQUAL "none")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_LINK_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64_gen \"-device ${EMBREE_DPCPP_AOT_DEVICES} ${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
      ENDIF()

      SET(CMAKE_CXX_FLAGS_SYCL  "${CMAKE_CXX_FLAGS_SYCL}  ${CMAKE_CXX_FLAGS_SYCL_AOT}")
      SET(CMAKE_LINK_FLAGS_SYCL "${CMAKE_LINK_FLAGS_SYCL} ${CMAKE_LINK_FLAGS_SYCL_AOT}")

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pessimizing-move -isystem \"${SYCL_COMPILER_DIR}/../include/sycl\" -isystem \"${SYCL_COMPILER_DIR}/../include/\"")       # disable warning from SYCL header
  ENDIF(EMBREE_DPCPP_SUPPORT)

  GET_FILENAME_COMPONENT(COMPILER_NAME ${CMAKE_CXX_COMPILER} NAME_WE)
  message("compiler name: ${COMPILER_NAME}")

  IF (COMPILER_NAME STREQUAL "icx")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")   # makes dpcpp compiler compatible with clang++
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-model=precise")   # makes dpcpp compiler compatible with clang++
  ENDIF()

  INCLUDE(msvc_post)

  # workaround for file encoding problems of kernels/embree.rc found here https://gitlab.kitware.com/cmake/cmake/-/issues/18311
  set(CMAKE_NINJA_CMCLDEPS_RC OFF)

ELSE()

  OPTION(EMBREE_IGNORE_CMAKE_CXX_FLAGS "When enabled Embree ignores default CMAKE_CXX_FLAGS." ON)
  OPTION(EMBREE_ADDRESS_SANITIZER "Enabled CLANG address sanitizer." OFF)
  IF (EMBREE_IGNORE_CMAKE_CXX_FLAGS)
    SET(CMAKE_CXX_FLAGS "")
  ENDIF()

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") 
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")                       # enables most warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat -Wformat-security")  # enables string format vulnerability warnings
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")               # treat char as signed on all processors, including ARM
  IF (NOT APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")                     # enables support for more secure position independent execution
  ENDIF()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")                       # generate position independent code suitable for shared libraries
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")                       # generate position independent code suitable for shared libraries
  IF (EMBREE_DPCPP_SUPPORT)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")                  # enables C++17 features
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")                  # enables C++11 features    
  ENDIF()
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

  IF (EMBREE_ADDRESS_SANITIZER)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer -fno-optimize-sibling-calls")
  ENDIF()

  IF (EMBREE_DPCPP_SUPPORT)

    GET_FILENAME_COMPONENT(SYCL_COMPILER_DIR ${CMAKE_CXX_COMPILER} PATH)
    GET_FILENAME_COMPONENT(SYCL_COMPILER_NAME ${CMAKE_CXX_COMPILER} NAME)

    IF (NOT SYCL_COMPILER_NAME STREQUAL "dpcpp")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning
    ENDIF()
    
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") # enables C++17 features
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sycl")  # makes dpcpp compiler compatible with clang++

    SET(CMAKE_CXX_FLAGS_SYCL "-Wno-mismatched-tags -Wno-pessimizing-move -Wno-reorder -Wno-unneeded-internal-declaration -Wno-delete-non-abstract-non-virtual-dtor -Wno-dangling-field -Wno-unknown-pragmas -Wno-logical-op-parentheses -fsycl -fsycl-unnamed-lambda -Xclang -fsycl-allow-func-ptr")

    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -g0")              # FIXME: debug information generation takes forever in SYCL
    SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -UDEBUG -DNDEBUG") # FIXME: assertion still not working in SYCL

    SET(CMAKE_LINK_FLAGS_SYCL "-fsycl")

      SET(CMAKE_IGC_OPTIONS "EnableOCLNoInlineAttr=0")                                # enabled __noinline
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ControlKernelTotalSize=0")
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},SubroutineThreshold=110000")        # Minimal kernel size to enable subroutines
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableUnmaskedFunctions=1")         # enables unmasked functions
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ByPassAllocaSizeHeuristic=64")      # puts small arrays into registers
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableIndirectCallOptimization=0")  # Enables inlining indirect calls by comparing function addresses
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},FunctionControl=0")                 # 0 = default, 1 = inline, 2 = subroutine, 3 = stackcall, 4 = keep indirect calls
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},forceGlobalRA=1")                   # "force global register allocator
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},TotalGRFNum=128")                   # Total GRF used for register allocation
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},GRFNumToUse=64")                   # "Set the number of general registers to use (64 to totalGRFNum)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ReplaceIndirectCallWithJmpi=1")     # Replace indirect call with jmpi instruction (HW WA)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},DisableUniformAnalysis=1")          # Setting this to 1/true adds a compiler switch to disable uniform_analysis
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},DisableLoopUnroll=1")               # Setting this to 1/true adds a compiler switch to disable loop unrolling
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableStatelessToStatefull=0")      #  Enable Stateless To Statefull transformation for global and constant address space in OpenCL kernels
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableRecursionOpenCL=1")           # Enable recursion with OpenCL user functions
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnableAdvMemOpt=0")                 # Enable advanced memory optimization
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},UniformMemOptLimit=512")            # "Limit of uniform memory optimization in bits
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},EnablePreemption=0")                 # Enable generating preeemptable code (SKL+)
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AllowSubroutineAndInirectdCalls=1")  # Allow subroutine in the presence of indirect calls
      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AllocaRAPressureThreshold=0")        # The threshold for the register pressure potential (this reduces amount of spilling!)

      #SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},AssumeInt64Support=0")               # Architecture with partial int64 still promote uniform arrays to registers
      SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},VISAOptions=-scratchAllocForStackInKB 128 -nospillcompression")  # this works around some IGC bug in spill compression

      IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
        SET(CMAKE_IGC_OPTIONS "${CMAKE_IGC_OPTIONS},ForceInlineStackCallWithImplArg=0,EnableGlobalStateBuffer=1")   # to allow printf inside indirectly callable function
      ENDIF()
            
      IF (EMBREE_DPCPP_AOT_DEVICE_REVISION GREATER 0)
        SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -revision_id ${EMBREE_DPCPP_AOT_DEVICE_REVISION}")        # Enable this to override the stepping/RevId, default is a0 = 0, b0 = 1, c0 = 2, so on...        
      ENDIF()
      
      # all the sycl libraries that we have to link against
      set(SYCL_LIBS "${SYCL_COMPILER_DIR}/../lib/libsycl-glibc.o")
      if (EXISTS "${SYCL_COMPILER_DIR}/../lib/libsycl-fallback-cassert.o")
        list(APPEND SYCL_LIBS "${SYCL_COMPILER_DIR}/../lib/libsycl-fallback-cassert.o")
      endif()
      string(REPLACE ";" "  " SYCL_LIBS "${SYCL_LIBS}")

      SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -cl-intel-greater-than-4GB-buffer-required")      # enables support for buffers larger than 4GB
      #SET(CMAKE_OCL_OPTIONS "${CMAKE_OCL_OPTIONS} -ze-opt-large-register-file")                     # large GRF mode
      SET(CMAKE_OCL_OTHER_OPTIONS "${CMAKE_OCL_OTHER_OPTIONS} -cl-intel-force-global-mem-allocation -cl-intel-no-local-to-generic")

      IF (EMBREE_DPCPP_AOT_DEVICES STREQUAL "none")
        SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64 \"${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
      ELSE()
        #SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64,spir64_gen")
        SET(CMAKE_CXX_FLAGS_SYCL_AOT "-fsycl-targets=spir64_gen")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT}")
      ENDIF()

      #SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_CXX_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64 \"${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
      
      IF (NOT EMBREE_DPCPP_AOT_DEVICES STREQUAL "none")
        SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_LINK_FLAGS_SYCL_AOT} -Xsycl-target-backend=spir64_gen \"-device ${EMBREE_DPCPP_AOT_DEVICES} ${CMAKE_OCL_OPTIONS} -options \\\"${CMAKE_OCL_OTHER_OPTIONS} -igc_opts='${CMAKE_IGC_OPTIONS}'\\\"\"")
      ENDIF()

      #SET(CMAKE_LINK_FLAGS_SYCL_AOT "${CMAKE_LINK_FLAGS_SYCL_AOT} ${SYCL_LIBS}")

      SET(CMAKE_CXX_FLAGS_SYCL  "${CMAKE_CXX_FLAGS_SYCL}  ${CMAKE_CXX_FLAGS_SYCL_AOT}")
      SET(CMAKE_LINK_FLAGS_SYCL "${CMAKE_LINK_FLAGS_SYCL} ${CMAKE_LINK_FLAGS_SYCL_AOT}")

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pessimizing-move -isystem ${SYCL_COMPILER_DIR}/../include/sycl -isystem ${SYCL_COMPILER_DIR}/../include/")       # disable warning from SYCL header
  ENDIF(EMBREE_DPCPP_SUPPORT)

  #SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-intel") # links intel runtime statically
  #SET(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -static-intel") # links intel runtime statically
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-model=precise")        # dpcpp has fp-model fast as default

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

  IF(EMBREE_DPCPP_SUPPORT)
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "")
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "${CMAKE_CXX_FLAGS_RELWITHASSERT} -DDEBUG")         # enable assertions
    SET(CMAKE_CXX_FLAGS_RELWITHASSERT "${CMAKE_CXX_FLAGS_RELWITHASSERT} -O3")             # enable full optimizations
  ENDIF(EMBREE_DPCPP_SUPPORT)

  IF (APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7")   # makes sure code runs on older MacOSX versions
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")             # link against libc++ which supports C++11 features
  ELSE(APPLE)
    IF (NOT EMBREE_ADDRESS_SANITIZER) # for address sanitizer this causes link errors
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined") # issues link error for undefined symbols in shared library
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack")     # we do not need an executable stack
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z relro -z now")    # re-arranges data sections to increase security
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")           # we do not need an executable stack
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z relro -z now")          # re-arranges data sections to increase security
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")                     # enables position independent execution for executable
    ENDIF()
  ENDIF(APPLE)

ENDIF()