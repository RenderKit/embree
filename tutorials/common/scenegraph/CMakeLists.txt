## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(scenegraph STATIC
    xml_parser.cpp
    xml_loader.cpp
    xml_writer.cpp
    obj_loader.cpp
    ply_loader.cpp
    corona_loader.cpp
    texture.cpp
    scenegraph.cpp
    geometry_creation.cpp)

TARGET_LINK_LIBRARIES(scenegraph sys math lexers image embree)
SET_PROPERTY(TARGET scenegraph PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET scenegraph APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
