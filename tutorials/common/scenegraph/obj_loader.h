// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "scenegraph.h"

namespace embree
{
  Ref<SceneGraph::Node> loadOBJ(const FileName& fileName, 
                                const bool subdivMode = false,
                                const bool combineIntoSingleObject = false);
}
