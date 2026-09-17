## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)

INCLUDE(tutorial)
ADD_TUTORIAL(gaussian_splats)

IF (EMBREE_TUTORIALS_SPZ)
  IF (MSVC)
    SET(SPZ_CXX17_FLAG /std:c++17)
  ELSE()
    SET(SPZ_CXX17_FLAG -std=c++17)
  ENDIF()

  FIND_PACKAGE(spz 1.1 CONFIG QUIET)

  IF (NOT TARGET spz::spz)
    INCLUDE(FetchContent)
    SET(SPZ_BUILD_EXTENSIONS ON CACHE BOOL "" FORCE)
    SET(SPZ_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    SET(SPZ_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
      spz
      GIT_REPOSITORY https://github.com/nianticlabs/spz.git
      GIT_TAG affd0ecea7fbb4c265ee119475af7ee5b2997482
    )
    FetchContent_MakeAvailable(spz)
    TARGET_COMPILE_OPTIONS(spz PRIVATE ${SPZ_CXX17_FLAG})
  ENDIF()

  TARGET_COMPILE_OPTIONS(embree_gaussian_splats PRIVATE ${SPZ_CXX17_FLAG})
  TARGET_COMPILE_DEFINITIONS(embree_gaussian_splats PRIVATE EMBREE_TUTORIALS_SPZ)
  TARGET_LINK_LIBRARIES(embree_gaussian_splats spz::spz)
ENDIF()

ENDIF()
