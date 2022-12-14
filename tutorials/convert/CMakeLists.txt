## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(embree_convert ../../kernels/embree.rc convert.cpp distribution1d.cpp distribution2d.cpp)
TARGET_LINK_LIBRARIES(embree_convert scenegraph image tasking)
SET_PROPERTY(TARGET embree_convert PROPERTY FOLDER tutorials/single)
SET_PROPERTY(TARGET embree_convert APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_convert DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(embree_convert)
