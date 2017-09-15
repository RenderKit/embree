
message("CTEST_BUILD_OPTIONS = ${CTEST_BUILD_OPTIONS}")

# project and repository to clone from
SET(CTEST_PROJECT_NAME "Embree")
SET(TEST_REPOSITORY "https://github.com/embree/embree.git")

SET(TEST_ROOT_DIRECTORY ${CTEST_SCRIPT_DIRECTORY})
STRING(REPLACE ":/" ":\\" TEST_ROOT_DIRECTORY1 "${TEST_ROOT_DIRECTORY}") # cygwin git has issues with c:/dir paths 
SET(TEST_MODELS_DIRECTORY ${TEST_ROOT_DIRECTORY}/dependencies/embree-models)

# set source and build directory
set(CTEST_SOURCE_DIRECTORY "${TEST_ROOT_DIRECTORY1}/embree/${TEST_TRACK}/${TEST_NAME}/code")
set(CTEST_BINARY_DIRECTORY "${TEST_ROOT_DIRECTORY1}/embree/${TEST_TRACK}/${TEST_NAME}/build")
set(CTEST_BENCHMARK_DATABASE "${TEST_ROOT_DIRECTORY1}/embree/${TEST_TRACK}/${TEST_NAME}/benchmark")
file(MAKE_DIRECTORY ${CTEST_BENCHMARK_DATABASE})

# update external model repository
FIND_PROGRAM(CTEST_GIT_COMMAND NAMES git)

# macro that updates the test models
MACRO(update_test_models)
  IF(NOT EXISTS "${TEST_MODELS_DIRECTORY}")
    MESSAGE("cloning test models ...")
    EXECUTE_PROCESS(
      COMMAND ${CTEST_GIT_COMMAND} "clone" "git@git.sdvis.org:embree-models.git" dependencies/embree-models
      WORKING_DIRECTORY ${TEST_ROOT_DIRECTORY}
    )
  ELSE()
    MESSAGE("updating test models ...")
    EXECUTE_PROCESS(
      COMMAND ${CTEST_GIT_COMMAND} "fetch"
      WORKING_DIRECTORY ${TEST_MODELS_DIRECTORY}
    )
  ENDIF()
  IF (NOT TEST_MODELS_HASH)
    SET(TEST_MODELS_HASH master)
  ENDIF()
  MESSAGE("checking out test models: ${TEST_MODELS_HASH}")
  EXECUTE_PROCESS(
      COMMAND ${CTEST_GIT_COMMAND} "checkout" ${TEST_MODELS_HASH}
      WORKING_DIRECTORY ${TEST_MODELS_DIRECTORY}
  )
ENDMACRO()

# increase default output sizes for test outputs
IF (NOT DEFINED CTEST_CUSTOM_MAXIMUM_PASSED_TEST_OUTPUT_SIZE)
  SET(CTEST_CUSTOM_MAXIMUM_PASSED_TEST_OUTPUT_SIZE 100000)
ENDIF()
IF(NOT DEFINED CTEST_CUSTOM_MAXIMUM_FAILED_TEST_OUTPUT_SIZE)
  SET(CTEST_CUSTOM_MAXIMUM_FAILED_TEST_OUTPUT_SIZE 800000)
ENDIF()

# enable testing in Embree
SET (CTEST_BUILD_OPTIONS "${CTEST_BUILD_OPTIONS} -D BUILD_TESTING:BOOL=ON -D EMBREE_TESTING_MODEL_DIR:PATH=${TEST_MODELS_DIRECTORY}")

# enable benchmarking
IF (TEST_BENCHMARK)
 SET (CTEST_BUILD_OPTIONS "${CTEST_BUILD_OPTIONS} -D EMBREE_TESTING_BENCHMARK=ON -D EMBREE_TESTING_BENCHMARK_DATABASE:PATH=${CTEST_BENCHMARK_DATABASE}")
ENDIF()

# set site based on this machine's hostname
SITE_NAME(HOSTNAME)
set(CTEST_SITE "${HOSTNAME}")

# drop location
set(CTEST_DROP_METHOD "http")
IF(NOT CTEST_DROP_SITE)
#  set(CTEST_DROP_SITE "10.123.110.146")
   set(CTEST_DROP_SITE "10.123.110.90")
ENDIF()
set(CTEST_DROP_LOCATION "/CDash/submit.php?project=${CTEST_PROJECT_NAME}")
set(CTEST_DROP_SITE_CDASH TRUE)

# get OS and CPU information
find_program(UNAME NAMES uname)
macro(getuname name flag)
  exec_program("${UNAME}" ARGS "${flag}" OUTPUT_VARIABLE "${name}")
endmacro(getuname)

getuname(osname -s)
getuname(osrel  -r)
getuname(cpu    -m)

# initial git checkout if needed
set(initial_checkout FALSE)

if(NOT EXISTS "${CTEST_SOURCE_DIRECTORY}")
  exec_program("${CTEST_GIT_COMMAND} clone ${TEST_REPOSITORY} -b ${TEST_BRANCH} ${CTEST_SOURCE_DIRECTORY}")
  set(initial_checkout TRUE)
endif()

# get git branch name (should match ${TEST_BRANCH}!)
macro(getgitbranch name)
  exec_program("${CTEST_GIT_COMMAND}" ARGS "--git-dir=${CTEST_SOURCE_DIRECTORY}/.git rev-parse --symbolic-full-name --abbrev-ref HEAD" OUTPUT_VARIABLE "${name}")
endmacro(getgitbranch)

getgitbranch(branch)

if (NOT "${branch}" STREQUAL "${TEST_BRANCH}")
  message(FATAL_ERROR "branch is ${branch} and expected to be ${TEST_BRANCH}")
endif()

# build using as many processes as we have processors
include(ProcessorCount)
ProcessorCount(numProcessors)
if(numProcessors EQUAL 0)
  SET(numProcessors 1)
endif()

# set build name
set(CTEST_BUILD_NAME "${TEST_NAME}-${osname}-${cpu}")
set(CTEST_CMAKE_GENERATOR "Unix Makefiles")
IF (WIN32)
  set(CTEST_BUILD_COMMAND "${CMAKE_COMMAND} --build . --config ${CTEST_CONFIGURATION_TYPE}")
ELSE()
  set(CTEST_BUILD_COMMAND "make -j ${numProcessors}")
ENDIF()

###################
# execute the test
###################

# requires CMake 2.8 or higher for git!!!
set(CTEST_UPDATE_COMMAND "${CTEST_GIT_COMMAND}")

set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE:STRING=${CTEST_BUILD_CONFIGURATION}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} ${CTEST_BUILD_OPTIONS}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_SOURCE_DIRECTORY}\"")

# start the test
if (${TEST_TYPE} STREQUAL "Continuous")
  ctest_start(${TEST_TYPE} TRACK ${TEST_TRACK})
  ctest_update (RETURN_VALUE count)

  if (count GREATER 0 OR initial_checkout)
    ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})
    ctest_start(${TEST_TYPE} TRACK ${TEST_TRACK}) # we have to do this twice to use the proper CTestConfig.cmake file from the update
    ctest_update ()
    update_test_models()
    ctest_configure()
    ctest_build()
    if (NOT CTEST_SKIP_TESTING)
      ctest_test()
    endif()
    ctest_submit()
  endif()
else()
  ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})
  ctest_start(${TEST_TYPE} TRACK ${TEST_TRACK})
  ctest_update()
  ctest_start(${TEST_TYPE} TRACK ${TEST_TRACK}) # we have to do this twice to use the proper CTestConfig.cmake file from the update
  update_test_models()
  ctest_configure()
  ctest_build()
  if (NOT CTEST_SKIP_TESTING)
    ctest_test()
  endif()
  ctest_submit()
endif()
