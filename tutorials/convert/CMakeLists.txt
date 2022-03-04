## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_EXECUTABLE(convert ../../kernels/embree.rc convert.cpp distribution1d.cpp distribution2d.cpp)
TARGET_LINK_LIBRARIES(convert scenegraph image tasking)
SET_PROPERTY(TARGET convert PROPERTY FOLDER tutorials/single)
SET_PROPERTY(TARGET convert APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS convert DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(convert)
