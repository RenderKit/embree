// ======================================================================== //
//                                                                          //
// Copyright 2009-2019 Intel Corporation                                    //
//                                                                          //
// This program is the sole property of the Intel Corporation and           //
// contains proprietary and confidential information. Unauthorized          //
// use or distribution will be subject to action as prescribed by           //
// the license agreement.                                                   //
//                                                                          //
// ======================================================================== //

// Glue code between RTASFile and Embree's scene graph

#pragma once

#include "RTASFile.h"
#include "scenegraph.h"
using namespace embree;

namespace RTASFile
{

    Ref<SceneGraph::TriangleMeshNode> GeoToEmbree(  RTASFile::Geo& geo );

}
