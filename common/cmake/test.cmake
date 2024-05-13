## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0
if (BUILD_TESTING OR EMBREE_TESTING_INSTALL_TESTS)

  INCLUDE(CTest)

  if (EMBREE_TESTING_INSTALL_TESTS)
    SET(EMBREE_INSTALL_CTESTTESTFILE "${CMAKE_CURRENT_BINARY_DIR}/embree-addtests.cmake")
    file(WRITE "${EMBREE_INSTALL_CTESTTESTFILE}" "")
  endif()

  if (NOT EMBREE_TESTING_PACKAGE_TEST_PROJECT)
    IF (WIN32)
        IF("${CMAKE_CXX_COMPILER_ID}" MATCHES "MSVC")
          SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
        ELSE()
          SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
        ENDIF()
    ELSE()
        SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    ENDIF()
  else()
    SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${EMBREE_INSTALL_BINDIR}")
  endif()

  if (NOT EMBREE_TESTING_PACKAGE_TEST_PROJECT)
    SET(EMBREE_TESTING_INTENSITY 1 CACHE STRING "Intensity of testing (0 = no testing, 1 = verify and tutorials, 2 = light testing, 3 = intensive testing, 4 = very intensive testing.")
    SET_PROPERTY(CACHE EMBREE_TESTING_INTENSITY PROPERTY STRINGS 0 1 2 3 4)
    SET(EMBREE_TESTING_ONLY_SYCL_TESTS OFF CACHE BOOL "Run only tests with the sycl support.")
    SET(EMBREE_TESTING_MEMCHECK OFF CACHE BOOL "Turns on memory checking for some tests.")
    SET(EMBREE_TESTING_BENCHMARK OFF CACHE BOOL "Turns benchmarking on.")
    SET(EMBREE_TESTING_BENCHMARK_DATABASE "${PROJECT_BINARY_DIR}" CACHE PATH "Path to database for benchmarking.")
    SET(EMBREE_TESTING_PACKAGE OFF CACHE BOOL "Packages release as test.")
    SET(EMBREE_TESTING_KLOCWORK OFF CACHE BOOL "Runs Kocwork as test.")
    SET(EMBREE_TESTING_SDE OFF CACHE STRING "Uses SDE to run tests for specified CPU.")
    SET_PROPERTY(CACHE EMBREE_TESTING_SDE PROPERTY STRINGS OFF pnr nhm wsm snb ivb hsw bdw knl skl skx cnl)
  endif()

  FUNCTION (SET_EMBREE_TEST_PROPERTIES testname)
    SET(variants "_cpp;_ispc;_sycl")
    foreach(v ${variants})
      if (v STREQUAL "_cpp")
        SET(v "")
      endif()
      set(testnamedef "EMBREE_TEST_${testname}${v}_DEFINED")
      if(${testnamedef})
        SET_TESTS_PROPERTIES(${testname}${v} ${ARGN})
      endif()
    endforeach()

    if (EMBREE_TESTING_INSTALL_TESTS)
      file(APPEND "${EMBREE_INSTALL_CTESTTESTFILE}" "SET_EMBREE_TEST_PROPERTIES(${testname} ${ARGN}) \n")
    endif()
  ENDFUNCTION()

  MACRO (ADD_EMBREE_GENERIC_TEST testname executable)  
    ADD_TEST(NAME ${testname}
             WORKING_DIRECTORY "${MY_PROJECT_BINARY_DIR}"
             COMMAND ${executable} ${ARGN})
    SET(testnamedef EMBREE_TEST_${testname}_DEFINED)
    SET(${testnamedef} "1" CACHE INTERNAL "")
  ENDMACRO()

  MACRO (ADD_EMBREE_GENERIC_CPP_TEST testname executable)  
    if((NOT ${EMBREE_SYCL_SUPPORT}) OR (NOT ${EMBREE_TESTING_ONLY_SYCL_TESTS}))
      ADD_TEST(NAME ${testname}
               WORKING_DIRECTORY "${MY_PROJECT_BINARY_DIR}"
               COMMAND ${executable} ${ARGN})
      SET(testnamedef EMBREE_TEST_${testname}_DEFINED)
      SET(${testnamedef} "1" CACHE INTERNAL "")
    endif()
  ENDMACRO()

  MACRO (ADD_EMBREE_GENERIC_ISPC_TEST testname executable)  
    if((NOT ${EMBREE_SYCL_SUPPORT}) OR (NOT ${EMBREE_TESTING_ONLY_SYCL_TESTS}))
      IF (EMBREE_ISPC_SUPPORT AND EMBREE_RAY_PACKETS)
        ADD_TEST(NAME ${testname}_ispc
                 WORKING_DIRECTORY "${MY_PROJECT_BINARY_DIR}"
                 COMMAND ${executable}_ispc ${ARGN})
        SET(testnamedef EMBREE_TEST_${testname}_ispc_DEFINED)
        SET(${testnamedef} "1" CACHE INTERNAL "")
      ENDIF()       
    endif()
  ENDMACRO()

  MACRO (ADD_EMBREE_GENERIC_SYCL_TEST testname executable)  
    IF (EMBREE_SYCL_SUPPORT)
      ADD_TEST(NAME ${testname}_sycl
               WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
               COMMAND ${executable}_sycl ${ARGN})
      SET(testnamedef EMBREE_TEST_${testname}_sycl_DEFINED)
      SET(${testnamedef} 1 CACHE INTERNAL "")
      SET_TESTS_PROPERTIES(${testname}_sycl PROPERTIES TIMEOUT 50)
    ENDIF()
  ENDMACRO()

  

















  # Checks if the current cmake configuration is compatible with <condition>
  # condition may be a triple of CMAKE_VARIABLE_NAME op VALUE
  # supported operators for op are: ==, !=, <, <=, >, >=
  FUNCTION (EMBREE_TESTING_CHECK_OPTION out condition)  
    # parse condition into list
    string(REGEX MATCHALL "([^\ ]+\ |[^\ ]+$)" tokens "${condition}")
    LIST(LENGTH tokens token_count)
    IF (NOT ${token_count} EQUAL 3)
      message(FATAL_ERROR "illegal embree_opitons condition: ${condition}")
    ENDIF()

    # we require every condition to follow the scheme "variable_name op value"
    LIST(GET tokens 0 option)
    LIST(GET tokens 1 comp)
    LIST(GET tokens 2 value)

    STRING(STRIP ${option} option)
    STRING(STRIP ${comp} comp)
    STRING(STRIP ${value} value)

    SET(${out} 0 PARENT_SCOPE)
    if ("${comp}" STREQUAL "==")
      if ("${${option}}" STREQUAL "${value}")
        SET(${out} 1 PARENT_SCOPE)
      endif()
    elseif ("${comp}" STREQUAL "!=")
      if (NOT ("${${option}}" STREQUAL "${value}"))
        SET(${out} 1 PARENT_SCOPE)
      endif()
    elseif ("${comp}" STREQUAL ">")
      if ("${${option}}" GREATER "${value}")
        SET(${out} 1 PARENT_SCOPE)
      endif()
    elseif ("${comp}" STREQUAL ">=")
      if ("${${option}}" GREATER_EQUAL "${value}")
        SET(${out} 1 PARENT_SCOPE)
      endif()
    elseif ("${comp}" STREQUAL "<")
      if ("${${option}}" LESS "${value}")
        SET(${out} 1 PARENT_SCOPE)
      endif()
    elseif ("${comp}" STREQUAL "<=")
      if ("${${option}}" LESS_EQUAL "${value}")
        SET(${out} 1 PARENT_SCOPE)
      endif()
    else()
      message(FATAL_ERROR "Could not parse embree_option condition: ${condition}")
    endif()

  ENDFUNCTION()

  # Checks multiple options from a list with EMBREE_TESTING_CHECK_OPTION
  FUNCTION (EMBREE_TESTING_CHECK_OPTIONS_LIST out conditions)  
    SET(${out} 1 PARENT_SCOPE)
    FOREACH (c ${conditions})
      EMBREE_TESTING_CHECK_OPTION(myout ${c})

      IF (myout EQUAL 0)
        SET(${out} 0 PARENT_SCOPE)
        BREAK()
      ENDIF()
    ENDFOREACH()
  ENDFUNCTION()


  # looks for ifile in multiple possible locations and outputs a file with absolute path in ofile
  FUNCTION (EMBREE_FIND_TEST_FILE ifile ofile errmsgflavor)
    if (EXISTS "${ifile}")                                  # abs path, use get_filename_component because it could also be relative to cwd
      get_filename_component(absifile "${ifile}" ABSOLUTE)
      SET(${ofile} ${absifile} PARENT_SCOPE)
    elseif(EXISTS "${PROJECT_SOURCE_DIR}/tests/${ifile}")   # testing dir
      set(${ofile} "${PROJECT_SOURCE_DIR}/tests/${ifile}" PARENT_SCOPE)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${ifile}")   # relative to source folder
      set(${ofile} "${CMAKE_CURRENT_SOURCE_DIR}/${ifile}" PARENT_SCOPE)
    else()
      set(${ofile} "" PARENT_SCOPE)
      if (errmsgflavor)
        message(FATAL_ERROR
          "Could not find ${errmsgflavor} \"${ifile}\"\n"
          "looked for:\n"
          "  ${inputfile}\n"
          "  ${PROJECT_SOURCE_DIR}/tests/${ifile}\n"
          "  ${CMAKE_CURRENT_SOURCE_DIR}/${ifile}\n"
        )
      endif()
    endif()
  ENDFUNCTION()


  FUNCTION (EMBREE_ADD_TEST_PARSE_SUBLIST args keyword sublist)
    SET(myargs ${args})
    SET(mysublist "")
    SET(keywords "ECS;XML;NO_REFERENCE;REFERENCE;REFERENCE_SUFFIX;INTENSITY;CONDITION_FILE;CONDITION;ARGS;NO_CPP;NO_ISPC;NO_SYCL;GEN_REFERENCE;")

    list(FIND nargs ${keyword} istart)
    if (NOT(istart EQUAL -1))
      list(LENGTH myargs iend)
      foreach(k ${keywords})
        list(FIND myargs ${k} i)
        if (NOT(i EQUAL -1) AND (i GREATER istart) AND (i LESS iend))
          SET(iend ${i})
        endif()
      endforeach()

      MATH(EXPR i "${istart}+1")
      while (i LESS iend)
        list(GET myargs ${i} elem)
        list(APPEND mysublist ${elem})
        MATH(EXPR i "${i}+1")
      endwhile()
    endif()

    SET(${sublist} ${mysublist} PARENT_SCOPE)
  ENDFUNCTION()

  FUNCTION (EMBREE_ADD_TEST_PARSE_FLAG args keyword value)
    SET(myargs ${args})
    SET(${value} OFF PARENT_SCOPE)
    list(FIND nargs ${keyword} i)
    if (NOT(i EQUAL -1))
      SET(${value} ON PARENT_SCOPE)
    endif()
  ENDFUNCTION()

    # ADD_EMBREE_TEST_ECS(testname exename [ECS <file> | XML <file>] [NO_REFERENCE | REFERENCE <path> | REFERENCE_SUFFIX <suffix>] [INTENSITY <i>] [CONDITION <conds>] [ARGS <args>] [GEN_REFERENCE])
    # [ECS <file> | XML <inputfile> | OPTIONS <inputfile>]
    #   - looks for file and calls the test command with either "-c <inputfile>.ecs" or "-i <inputfile>.xml"
    #
    # [NO_REFERENCE | REFERENCE <path> | REFERENCE_SUFFIX <suffix>]
    #   - if not found a reference is will be expected in the same folder as *.ecs with name *.ecs.exename.exr
    #   - NO_REFERENCE: don't look for a reference (no --compare in test command)
    #   - REFERENCE <path>: use the reference located in <path>. Same rules aplly as for finding ecs files: absolute or relative to CMAKE_CURRENT_SOURCE_DIR path, must not be located outside the embree root dir.
    #   - REFERENCE_SUFFIX <suffix>: use the default reference location and name with a suffix before the last file extension, e.g. *.ecs.exename<suffix>.exr
    #   - if this argument is not specified, looks for 
    #       1. <inputfile>.exename.exr, or
    #       2. <testname>.exr, if no <inputfile> was given
    #
    # [INTENSITY <i>] 
    #   - default i = 1
    #   - sets the intensity level for the test, test is only run if ${EMBREE_TESTING_INTENSITY} GREATER_EQUAL i
    #   - could be done with an *.embree_options file, but this is more flexible, e.g. you can easier share reference images without specifying an absolute path
    #   - DOES NOT overrite EMBREE_TESTING_INTENSITY, if specified in *.embree_options
    #
    # [CONDITION_FILE <file>]
    #   - file containing additional conditions
    #   - conditions are specified linewise in form of: EMBREE_OPTION op VALUE, where EMBREE_OPTION is a cmake variable used during embree configuration and op is one of ==, !=, <, <=, >, >=
    #   - if this argument is not specified, looks for 
    #       1. <inputfile>.embree_options, or
    #       2. <testname>.embree_options, if no <inputfile> was given
    #
    # [CONDITION <conds>]
    #   - cmake list of additional conditions, specified the same way as in an embree_options file
    #
    # [ARGS <args>]
    #   - additional arguments for the test command
    #
    # [GEN_REFERENCE]
    #   - writes the reference to the expected location
    #   - could also be done with the ARGS parameter, but this way we don't have to deal with paths
    #
    # EXAMPLES
    #
    # all optional arguments default, 
    #  - looks for points.ecs.embree_options to filter out test by configured options, if not found no restrictions
    #  - runs for EMBREE_TESTING_INTENSITY >= 1
    #  - reference will be expected in the same folder as points.ecs with name points.ecs.embree_viewer.exr
    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "models/xxx/points.ecs")                                  # 1) 
    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "${PROJECT_SOURCE_DIR}/models/xxx/points.ecs")            # 2) same as 1) but with absolute path to ecs
    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "points.ecs")                                             # 3) same as 1) but looks for points.ecs in the ${CMAKE_CURRENT_SOURCE_DIR} i.e. <embree_root>/tutorials/viewer/points.ecs

    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "models/xxx/points.ecs" ARGS --coherent INTENSITY 2)      # 4) same as 1) but runs only at EMBREE_TESTING_INTENSITY >= 2
    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "models/xxx/points.ecs" REFERENCE_SUFFIX "_quads" ARGS 
    #  --coherent 
    #  --convert-triangles-to-quads INTENSITY 2)                                                               # 5) same as 4) but expects reference name points.ecr.embree_viewer_quads.ecs
    #ADD_EMBREE_TEST_ECS(viewer_points embree_viewer "models/xxx/points.ecs" REFERENCE "special.exr" ARGS 
    #  --coherent 
    #  --convert-triangles-to-quads INTENSITY 2)                                                               # 5) same as 4) but expects reference <embree-root>/tutorials/viewer/special.exr
    ## 1) - 5) all share the same base input arguments defined in points.ecs as well as the points.ecs.embree_options

    #ADD_EMBREE_TEST_ECS(point_geometry embree_point_geometry)                                                 # 6) no ecs file, uses point_geometry.embree_options if exists, expects reference ${CMAKE_CURRENT_SOURCE_DIR}/point_geometry.exr
    #ADD_EMBREE_TEST_ECS(verify verify NO_REFERENCE)                                                           # 7) no ecs file, uses verify.embree_options if exists, no reference
    #ADD_EMBREE_TEST_ECS(verify_memcheck verify NO_REFERENCE CONDITIONS "EMBREE_TESTING_MEMCHECK == ON")       # 8) same as 7) but with extra condition

  FUNCTION (ADD_EMBREE_TEST_ECS testname executable)
    if (EMBREE_TESTING_FILTER_TESTNAMES)
      list(FIND EMBREE_TESTING_FILTER_TESTNAMES "${testname}" i)
      if (${i} EQUAL -1)
        return()
      endif()
    endif()

    SET(nargs ${ARGN})

    # disable everything
    SET(testnamedef EMBREE_TEST_${testname}_DEFINED)
    SET(${testnamedef} "0" CACHE INTERNAL "")
    SET(testnamedef EMBREE_TEST_${testname}_ispc_DEFINED)
    SET(${testnamedef} "0" CACHE INTERNAL "")
    SET(testnamedef EMBREE_TEST_${testname}_sycl_DEFINED)
    SET(${testnamedef} "0" CACHE INTERNAL "")

    # parsing input file
    #   ECS mode -> single parameter with filename after ECS keyword
    list(FIND nargs "ECS" i)
    if (NOT(i EQUAL -1))
      SET(inputtype "-c")
      MATH(EXPR i "${i}+1")
      list(GET nargs ${i} out)
      EMBREE_FIND_TEST_FILE("${out}" inputfile "test file")
    endif()
    #   XML mode -> single parameter with filename after XML keyword
    list(FIND nargs "XML" i)
    if (NOT(i EQUAL -1))
      SET(inputtype "-i")
      MATH(EXPR i "${i}+1")
      list(GET nargs ${i} out)
      EMBREE_FIND_TEST_FILE("${out}" inputfile "test file")
    endif()
    #   no filetype keyword -> look for <testname>.ecs/xml
    #   does not need to be specified
    if (NOT inputtype)
      EMBREE_FIND_TEST_FILE("${testname}.ecs" inputfile "")
      if(inputfile)
        SET(inputtype "-c")
      endif()
      EMBREE_FIND_TEST_FILE("${testname}.ecs" inputfile "")
      if(inputfile)
        SET(inputtype "-i")
      endif()
    endif()

    # parsing refernce image
    #   no reference image mode -> no additional parameter to parse
    list(FIND nargs "NO_REFERENCE" i)
    if (NOT(i EQUAL -1))
      SET(no_reference ON)
    endif()
    #   reference suffix mode -> singe parameter with suffix to default reference file name
    list(FIND nargs "REFERENCE_SUFFIX" i)
    if (NOT(i EQUAL -1))
      MATH(EXPR i "${i}+1")
      list(GET nargs ${i} out)
      EMBREE_FIND_TEST_FILE("${inputfile}.${executable}${out}.exr" referencefile "reference image")
    endif()
    #   reference mode -> singe parameter with absolute path to reference image
    list(FIND nargs "REFERENCE" i)
    if (NOT(i EQUAL -1))
      MATH(EXPR i "${i}+1")
      list(GET nargs ${i} out)
      EMBREE_FIND_TEST_FILE("${out}" referencefile "reference image")
    endif()
    #   no reference keyword -> look for <testname>.exr and <inputfile>.<executable>.exr respectively
    if ((NOT no_reference) AND (NOT referencefile))
      if (NOT inputfile)
        EMBREE_FIND_TEST_FILE("${testname}.exr" referencefile "reference image")
      else()
        EMBREE_FIND_TEST_FILE("${inputfile}.${executable}.exr" referencefile "reference image")
      endif()
    endif()

    # parsing intensity
    #   single integer parameter
    SET(intensity 1)
    list(FIND nargs "INTENSITY" i)
    if (NOT(i EQUAL -1))
      MATH(EXPR i "${i}+1")
      list(GET nargs ${i} intensity)
    endif()

    # parsing condition
    SET(conditions "")
    SET(conditionsfile)
    list(FIND nargs "CONDITION_FILE" i)
    #   condition file -> single parameter to absolute path of conditions file
    if (NOT(i EQUAL -1))
      list(GET nargs ${i} conditionsfile)
      EMBREE_FIND_TEST_FILE("${conditionsfile}" conditionsfile "")
    #   no <inputfile> specified -> look for <testname>.embree_options
    elseif (NOT inputtype)
      EMBREE_FIND_TEST_FILE("${testname}.embree_options" conditionsfile "")
    #   <inputfile> specified -> look for <inputfile>.embree_options
    else()
      EMBREE_FIND_TEST_FILE("${inputfile}.embree_options" conditionsfile "")
    endif()
    if (conditionsfile)
      file(READ "${conditionsfile}" lines)
      string(REGEX REPLACE "\n" ";" conditions "${lines}")
    endif()

    # parsing additional conditions
    EMBREE_ADD_TEST_PARSE_SUBLIST("${nargs}" "CONDITION" extra_conditions)
    list(APPEND conditions ${extra_conditions})

    # parsing extra args
    EMBREE_ADD_TEST_PARSE_SUBLIST("${nargs}" "ARGS" extraargs)

    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "MEMCHECK"      memcheck)
    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "NO_CPP"        no_cpp)
    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "NO_ISPC"       no_ispc)
    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "NO_SYCL"       no_sycl)
    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "NO_POSTFIX"    no_postfix)
    EMBREE_ADD_TEST_PARSE_FLAG("${nargs}" "GEN_REFERENCE" gen_reference)

    # add the test, if the conditions are met
    SET(matchconditions ON)
    if (conditions)
      EMBREE_TESTING_CHECK_OPTIONS_LIST(out "${conditions}")
      if (out EQUAL 0)
        SET(matchconditions OFF)
      endif()
    endif()

    # LESS_EQUAL not supported on all CI runners
    if (matchconditions AND ((intensity LESS EMBREE_TESTING_INTENSITY) OR (intensity EQUAL EMBREE_TESTING_INTENSITY)))
      SET(args "")
      if (inputfile)
        list(APPEND args "${inputtype} ${inputfile}")
      endif()
      if (NOT no_reference)
        list(APPEND args "--compare ${referencefile}")
      endif()
      if (gen_reference)
        list(APPEND args "-o ${referencefile}")
      endif()
      foreach(a ${extraargs})
        list(APPEND args "${a}")
      endforeach()



      if (memcheck)
        list(PREPEND args "${MY_PROJECT_BINARY_DIR}/${executable}")
        list(PREPEND args "${EMBREE_MEMORYCHECK_COMMAND_OPTIONS}")
        if (no_postfix)
          ADD_EMBREE_GENERIC_TEST(${testname} ${EMBREE_MEMORYCHECK_COMMAND} ${args})
        else()
          if((NOT ${EMBREE_SYCL_SUPPORT}) OR (NOT ${EMBREE_TESTING_ONLY_SYCL_TESTS}))
            if (NOT(no_cpp))
              ADD_EMBREE_GENERIC_CPP_TEST(${testname} ${EMBREE_MEMORYCHECK_COMMAND} ${args})
            endif()
            if (NOT(no_ispc))
              ADD_EMBREE_GENERIC_ISPC_TEST(${testname} ${EMBREE_MEMORYCHECK_COMMAND} ${args})
            endif()
          endif()
          if (NOT(no_sycl))
            ADD_EMBREE_GENERIC_SYCL_TEST(${testname} ${EMBREE_MEMORYCHECK_COMMAND} ${args}) 
          endif()
        endif()
      else()
        if (no_postfix)
          ADD_EMBREE_GENERIC_TEST(${testname} ${executable} ${args})
        else()
          if (NOT(no_cpp))
            ADD_EMBREE_GENERIC_CPP_TEST(${testname} ${executable} ${args})
          endif()
          if (NOT(no_ispc))
            ADD_EMBREE_GENERIC_ISPC_TEST(${testname} ${executable} ${args})
          endif()
          if (NOT(no_sycl))
            ADD_EMBREE_GENERIC_SYCL_TEST(${testname} ${executable} ${args}) 
          endif()
        endif()
      endif()
    endif()



    if (EMBREE_TESTING_INSTALL_TESTS)
      if (inputfile)
        get_filename_component(inputpath ${inputfile} DIRECTORY)
        STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" inputpath "${inputpath}")
        INSTALL(FILES "${inputfile}"
                DESTINATION "${CMAKE_INSTALL_TESTDIR}/${inputpath}"
                COMPONENT testing)
      endif()

      if (conditionsfile)
        get_filename_component(conditionspath ${conditionsfile} DIRECTORY)
        STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" conditionspath "${conditionspath}")
        INSTALL(FILES "${conditionsfile}"
                DESTINATION "${CMAKE_INSTALL_TESTDIR}/${conditionspath}"
                COMPONENT testing)
      endif()

      if (referencefile)
        get_filename_component(referencepath ${referencefile} DIRECTORY)
        STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" referencepath "${referencepath}")
        INSTALL(FILES "${referencefile}"
                DESTINATION "${CMAKE_INSTALL_TESTDIR}/${referencepath}"
                COMPONENT testing)
      endif()

      SET(testcall "ADD_EMBREE_TEST_ECS(${testname} ${executable}")
      if (inputfile)
        STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" inputfile "${inputfile}")
        if (inputtype STREQUAL "-c")
          SET(testcall "${testcall} \n    ECS ${inputfile}")
        elseif (inputtype STREQUAL "-i")
          SET(testcall "${testcall} \n    XML ${inputfile}")
        endif()
      endif()

      if (no_reference)
        SET(testcall "${testcall} \n    NO_REFERENCE")
      else()
        STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" referencefile "${referencefile}")
        SET(testcall "${testcall} \n    REFERENCE ${referencefile}")
      endif()

      SET(testcall "${testcall} \n    INTENSITY ${intensity}")
      if (${memcheck})
        SET(testcall "${testcall} \n    MEMCHECK")
      endif()
      if (${no_cpp})
        SET(testcall "${testcall} \n    NO_CPP")
      endif()
      if (${no_ispc})
        SET(testcall "${testcall} \n    NO_ISPC")
      endif()
      if (${no_sycl})
        SET(testcall "${testcall} \n    NO_SYCL")
      endif()
      if (${no_postfix})
        SET(testcall "${testcall} \n    NO_POSTFIX")
      endif()
      STRING(REPLACE "${PROJECT_SOURCE_DIR}/" "" conditionsfile "${conditionsfile}")
      SET(testcall "${testcall} \n    CONDITION_FILE ${conditionsfile}")
      SET(testcall "${testcall} \n    CONDITION ")
      foreach(c ${conditions})
        SET(testcall "${testcall} \"${c}\"")
      endforeach()
      SET(testcall "${testcall} \n    ARGS ")
      foreach(a ${extraargs})
        SET(testcall "${testcall} ${a}")
      endforeach()

      SET(testcall "${testcall})\n\n")
      file(APPEND "${EMBREE_INSTALL_CTESTTESTFILE}" "${testcall}")
    endif()
  ENDFUNCTION()

else()
  FUNCTION(ADD_EMBREE_TEST_ECS testname executable)
  ENDFUNCTION()
  FUNCTION(ADD_EMBREE_GENERIC_TEST testname executable)
  ENDFUNCTION()
  FUNCTION(ADD_EMBREE_GENERIC_CPP_TEST testname executable)
  ENDFUNCTION()
  FUNCTION(ADD_EMBREE_GENERIC_SYCL_TEST testname executable)
  ENDFUNCTION()
  FUNCTION(ADD_EMBREE_GENERIC_ISPC_TEST testname executable)
  ENDFUNCTION()
  FUNCTION(SET_EMBREE_TEST_PROPERTIES testname)
  ENDFUNCTION()
endif()