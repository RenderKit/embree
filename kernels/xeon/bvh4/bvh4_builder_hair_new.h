// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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
#include "geometry/bezier1v.h"
#include "builders_new/heuristic_binning.h"

namespace embree
{
  namespace isa
  {
    class BVH4BuilderHairNew : public Builder
    {
      ALIGNED_CLASS;
    public:
      
      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);
      
      /*! Constructor. */
      BVH4BuilderHairNew (BVH4* bvh, Scene* scene, size_t mode);
      
    private:
      typedef LinearAllocatorPerThread::ThreadAllocator Allocator;
      
      /*! creates a leaf node */
      BVH4::NodeRef createLeaf(Allocator& alloc, size_t depth, const PrimInfo& pinfo);
      
      /*! creates a large leaf that could be larger than supported by the BVH */
      BVH4::NodeRef createLargeLeaf(Allocator& alloc, size_t depth, const PrimInfo& pinfo);
    
      /*! performs split */
      bool split(const PrimInfo& pinfo, PrimInfo& linfo, PrimInfo& rinfo);
      
      /*! recursive build */
      BVH4::NodeRef recurse(Allocator& alloc, size_t depth, const PrimInfo& pinfo);
      
    public:
      Scene* scene;          //!< source
      BVH4* bvh;         //!< output
      size_t maxDepth;
      size_t minLeafSize;    //!< minimal size of a leaf
      size_t maxLeafSize;    //!< maximal size of a leaf

      HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic;
      vector_t<BezierPrim> prims;
    };
  }
}
