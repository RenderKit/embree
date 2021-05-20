// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "scenegraph.h"

namespace embree
{
  namespace SceneGraph
  {
    void storeXML(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures, bool referenceMaterials, bool binaryFormat);
  }
}

