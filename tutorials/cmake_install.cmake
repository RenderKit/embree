# Install script for directory: D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/embree2")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/common/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/triangle_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/dynamic_scene/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/user_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/viewer/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/instanced_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/intersection_filter/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/pathtracer/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/hair_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/subdivision_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/displacement_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/bvh_builder/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/lazy_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/bvh_access/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/motion_blur_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/interpolation/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/convert/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/curve_geometry/cmake_install.cmake")
  include("D:/info/embree/0embree-2.10.0/embree-2.10.0/tutorials/viewer_stream/cmake_install.cmake")

endif()

