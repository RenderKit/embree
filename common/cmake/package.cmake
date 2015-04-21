## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

OPTION(ENABLE_INSTALLER "Switches between installer or ZIP file creation for 'make package'" ON)

IF (NOT ENABLE_INSTALLER)
  SET(DOC_INSTALL_DIR doc)
  SET(TUTORIALS_INSTALL_DIR bin)
  SET(UTILITIES_INSTALL_DIR bin)
ELSEIF (WIN32)
  SET(DOC_INSTALL_DIR doc)
  SET(TUTORIALS_INSTALL_DIR bin)
  SET(UTILITIES_INSTALL_DIR bin)
ELSEIF (APPLE)
  SET(DOC_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/documentation)
  SET(TUTORIALS_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/tutorials)
  SET(UTILITIES_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/tutorials)
ELSE()
  SET(DOC_INSTALL_DIR share/doc/embree-${EMBREE_VERSION})
  SET(TUTORIALS_INSTALL_DIR bin)
  SET(UTILITIES_INSTALL_DIR bin)
ENDIF()

##############################################################
# Install Headers
##############################################################
INSTALL(DIRECTORY include/embree2 DESTINATION include COMPONENT headers)

##############################################################
# Install Embree CMake Configuration
##############################################################
IF (WIN32)
  CONFIGURE_FILE(common/cmake/embree-config-windows.cmake embree-config.cmake @ONLY)
ELSEIF (APPLE)
  CONFIGURE_FILE(common/cmake/embree-config-macosx.cmake embree-config.cmake @ONLY)
ELSE()
  CONFIGURE_FILE(common/cmake/embree-config-linux.cmake embree-config.cmake @ONLY)
ENDIF()

CONFIGURE_FILE(common/cmake/embree-config-version.cmake embree-config-version.cmake @ONLY)

IF (ENABLE_INSTALLER)
  INSTALL(FILES "${PROJECT_BINARY_DIR}/embree-config.cmake" DESTINATION "lib/cmake/embree-${EMBREE_VERSION}" COMPONENT libraries)
  INSTALL(FILES "${PROJECT_BINARY_DIR}/embree-config-version.cmake" DESTINATION "lib/cmake/embree-${EMBREE_VERSION}" COMPONENT libraries)
ENDIF()

##############################################################
# Install TBB
##############################################################

IF (WIN32)
  INSTALL(PROGRAMS ${TBB_LIBRARY} ${TBB_LIBRARY_MALLOC} DESTINATION bin COMPONENT tutorials)
  INSTALL(PROGRAMS ${TBB_LIBRARY} ${TBB_LIBRARY_MALLOC} DESTINATION lib COMPONENT library)
ELSEIF (APPLE)
  # install TBB with libc++ linkage for MacOSX
  INSTALL(PROGRAMS ${TBB_ROOT}/lib/libc++/libtbb.dylib ${TBB_ROOT}/lib/libc++/libtbbmalloc.dylib DESTINATION bin COMPONENT tutorials)
  INSTALL(PROGRAMS ${TBB_ROOT}/lib/libc++/libtbb.dylib ${TBB_ROOT}/lib/libc++/libtbbmalloc.dylib DESTINATION lib COMPONENT library)
ENDIF()

##############################################################
# CPack specific stuff
##############################################################

SET(CPACK_PACKAGE_NAME Embree)
SET(CPACK_PACKAGE_FILE_NAME "embree-${EMBREE_VERSION}")
#SET(CPACK_PACKAGE_ICON ${PROJECT_SOURCE_DIR}/embree-doc/images/icon.png)

SET(CPACK_PACKAGE_VERSION_MAJOR ${EMBREE_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${EMBREE_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${EMBREE_VERSION_PATCH})
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Embree: High Performance Ray Tracing Kernels")
SET(CPACK_PACKAGE_VENDOR "Intel Corporation")
SET(CPACK_PACKAGE_CONTACT embree_support@intel.com)

SET(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
SET(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "The Embree library.")

SET(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "Headers")
SET(CPACK_COMPONENT_HEADERS_DESCRIPTION "Header Files for C and ISPC required to develop applications with Embree.")

SET(CPACK_COMPONENT_DOCUMENTATION_DISPLAY_NAME "Documentation")
SET(CPACK_COMPONENT_DOCUMENTATION_DESCRIPTION "Documents describing Embree's API, usage and tutorials.")

SET(CPACK_COMPONENT_TUTORIALS_DISPLAY_NAME "Tutorials")
SET(CPACK_COMPONENT_TUTORIALS_DESCRIPTION "Tutorials demonstrating how to use Embree.")

SET(CPACK_COMPONENT_UTILITIES_DISPLAY_NAME "Utilities")
SET(CPACK_COMPONENT_UTILITIES_DESCRIPTION "Tools to benchmark, test and debug Embree.")

# devel install group
SET(CPACK_COMPONENT_LIBRARIES_GROUP devel)
SET(CPACK_COMPONENT_HEADERS_GROUP devel)
SET(CPACK_COMPONENT_DOCUMENTATION_GROUP devel)
SET(CPACK_COMPONENT_GROUP_DEVEL_DISPLAY_NAME "Development")
SET(CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION "Files needed to develop applications based on Embree.")
SET(CPACK_COMPONENT_DEVEL_DESCRIPTION ${CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION})

# example install group
SET(CPACK_COMPONENT_TUTORIALS_GROUP examples)
SET(CPACK_COMPONENT_UTILITIES_GROUP examples)
SET(CPACK_COMPONENT_GROUP_EXAMPLES_DISPLAY_NAME "Examples")
SET(CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION "Example tutorials and tools for Embree.")
SET(CPACK_COMPONENT_EXAMPLES_DESCRIPTION ${CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION})

# xeon phi install groups
SET(CPACK_COMPONENT_LIBRARIES_XEONPHI_GROUP devel_xeonphi)
SET(CPACK_COMPONENT_TUTORIALS_XEONPHI_GROUP examples_xeonphi)
SET(CPACK_COMPONENT_UTILITIES_XEONPHI_GROUP examples_xeonphi)
SET(XEONPHI_DESCRIPTION "For The Intel Xeon Phi coprocessor." )
SET(CPACK_COMPONENT_DEVEL_XEONPHI_DESCRIPTION "${CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION}\n${XEONPHI_DESCRIPTION}")
SET(CPACK_COMPONENT_EXAMPLES_XEONPHI_DESCRIPTION "${CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION}\n${XEONPHI_DESCRIPTION}")

# dependencies between components
SET(CPACK_COMPONENT_TUTORIALS_DEPENDS libraries)
SET(CPACK_COMPONENT_UTILITIES_DEPENDS libraries)
SET(CPACK_COMPONENT_LIBRARIES_XEONPHI_DEPENDS libraries)
SET(CPACK_COMPONENT_TUTORIALS_XEONPHI_DEPENDS libraries_xeonphi)
SET(CPACK_COMPONENT_UTILITIES_XEONPHI_DEPENDS libraries_xeonphi)

# point to readme and license files
SET(CPACK_RESOURCE_FILE_README ${PROJECT_SOURCE_DIR}/README.md)
SET(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.txt)

# Windows specific settings
IF(WIN32)

  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ARCH x64)
    SET(PROGRAMFILES "\$PROGRAMFILES64")
  ELSE()
    SET(ARCH win32)
    SET(PROGRAMFILES "\$PROGRAMFILES")
  ENDIF()
  SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}.${ARCH}.windows")

  # NSIS specific settings
  IF (ENABLE_INSTALLER)
    SET(CPACK_GENERATOR NSIS)
    SET(CPACK_COMPONENTS_ALL libraries headers documentation tutorials utilities)
    SET(CPACK_NSIS_INSTALL_ROOT "${PROGRAMFILES}\\\\Intel")
    SET(CPACK_NSIS_DISPLAY_NAME "Embree: High Performance Ray Tracing Kernels")
    SET(CPACK_NSIS_PACKAGE_NAME "Embree ${EMBREE_VERSION}")
    SET(CPACK_NSIS_URL_INFO_ABOUT http://embree.github.io/)
    #SET(CPACK_NSIS_HELP_LINK http://embree.github.io/downloads.html#windows)
    SET(CPACK_NSIS_MUI_ICON ${PROJECT_SOURCE_DIR}/embree-doc/images/icon32.ico)
    SET(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
  ELSE()
    SET(CPACK_GENERATOR ZIP)
    SET(CPACK_MONOLITHIC_INSTALL 1)
  ENDIF()

# MacOSX specific settings
ELSEIF(APPLE)

  CONFIGURE_FILE(README.md README.txt)
  SET(CPACK_RESOURCE_FILE_README ${PROJECT_BINARY_DIR}/README.txt)
  SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}.x86_64.macosx")

  IF (ENABLE_INSTALLER)
    SET(CPACK_GENERATOR PackageMaker)
    SET(CPACK_MONOLITHIC_INSTALL 1)
    SET(CPACK_PACKAGE_VENDOR "intel") # creates short name com.intel.embree2.xxx in pkgutil
    SET(CPACK_OSX_PACKAGE_VERSION 10.7)
  ELSE()
    SET(CPACK_GENERATOR TGZ)
    SET(CPACK_MONOLITHIC_INSTALL 1)
  ENDIF()

# Linux specific settings
ELSE()

  SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}.x86_64.linux")

  IF (ENABLE_INSTALLER)

    SET(CPACK_GENERATOR RPM)
    SET(CPACK_COMPONENTS_ALL libraries headers documentation tutorials utilities libraries_xeonphi tutorials_xeonphi utilities_xeonphi)
    SET(CPACK_RPM_COMPONENT_INSTALL ON)
    SET(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0") # Apache Software License, Version 2.0
    SET(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
    #SET(CPACK_RPM_CHANGELOG_FILE "") # ChangeLog of the RPM; also CHANGELOG.md is not in the required format
    SET(CPACK_RPM_PACKAGE_ARCHITECTURE x86_64)
    SET(CPACK_RPM_PACKAGE_URL http://embree.github.io/)

    # post install and uninstall scripts
    SET(CPACK_RPM_devel_POST_INSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
    SET(CPACK_RPM_devel_POST_UNINSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
    SET(CPACK_RPM_devel_xeonphi_POST_INSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
    SET(CPACK_RPM_devel_xeonphi_POST_UNINSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
  ELSE()
  
    SET(CPACK_GENERATOR TGZ)
    SET(CPACK_MONOLITHIC_INSTALL 1)
  ENDIF()
  
ENDIF()

