// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "geometry/primitive.h"

namespace embree
{
  class BVH2HairBuilder : public Builder
  {
    ALIGNED_CLASS;
  public:

    /*! Type shortcuts */
    typedef typename BVH2Hair::Node    Node;
    typedef typename BVH2Hair::NodeRef NodeRef;
    typedef typename BVH2Hair::Bezier1 Bezier1;
    
  public:

    /*! builder entry point */
    void build(size_t threadIndex, size_t threadCount);

    /*! Constructor. */
    BVH2HairBuilder (BVH2Hair* bvh, Scene* scene);

    /*! Destructor. */
    ~BVH2HairBuilder ();

    /*! creates a leaf node */
    NodeRef createLeaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3f& bounds);

    /*! recursive build function */
    NodeRef recurse   (size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3f& bounds);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf
    BVH2Hair* bvh;         //!< output
    Bezier1* curves;       //!< array with all curves
  };
}
