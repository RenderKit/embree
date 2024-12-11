#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "embree" for configuration "Release"
set_property(TARGET embree APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(embree PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libembree4.so.4"
  IMPORTED_SONAME_RELEASE "libembree4.so.4"
  )

list(APPEND _cmake_import_check_targets embree )
list(APPEND _cmake_import_check_files_for_embree "${_IMPORT_PREFIX}/lib/libembree4.so.4" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
