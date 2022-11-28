// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "scenegraph.h"

namespace embree
{
  Ref<SceneGraph::Node> loadRTAS(const FileName& fileName, 
                                 const bool combineIntoSingleObject = false);
}
