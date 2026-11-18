
message("WHAT             = ${WHAT}")

if(${WHAT} STREQUAL "UNPACK")
    message("PACKAGE_BASENAME = ${PACKAGE_BASENAME}")
    message("PACKAGE_EXT      = ${PACKAGE_EXT}")
    file(ARCHIVE_EXTRACT INPUT "${PACKAGE_BASENAME}.${PACKAGE_EXT}" DESTINATION "embree_install")
    # the testing package has no top-level directory, extract it into the Embree package directory
    file(ARCHIVE_EXTRACT INPUT "${PACKAGE_TESTING_NAME}.${PACKAGE_EXT}" DESTINATION "embree_install/${PACKAGE_BASENAME}")
elseif(${WHAT} STREQUAL "CHECK")
    file(READ ctest.output output_content)
    string(FIND "${output_content}" "100% tests passed, 0 tests failed" pos)
    if(pos EQUAL -1)
        message(FATAL_ERROR "Some tests failed, or no tests have been executed")
    endif()
endif()
