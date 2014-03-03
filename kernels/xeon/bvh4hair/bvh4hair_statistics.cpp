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

#include "bvh4hair_statistics.h"

namespace embree
{
  BVH4HairStatistics::BVH4HairStatistics (BVH4Hair* bvh) : bvh(bvh)
  {
    numAlignedNodes = numUnalignedNodes = numLeaves = numPrims = depth = 0;
    childrenAlignedNodes = childrenUnalignedNodes = 0;
    bvhSAH = 0.0f;
    float A = max(0.0f,halfArea(bvh->bounds));
    statistics(bvh->root,A,depth);
    bvhSAH /= area(bvh->bounds);
    assert(depth <= BVH4Hair::maxDepth);
  }

  std::string BVH4HairStatistics::str()  
  {
    std::ostringstream stream;
    size_t bytesAlignedNodes = numAlignedNodes*sizeof(AlignedNode);
    size_t bytesUnalignedNodes = numUnalignedNodes*sizeof(UnalignedNode);
    size_t bytesPrims  = numPrims*bvh->primTy.bytes;
    size_t numVertices = bvh->numVertices;
    size_t bytesVertices = numVertices*sizeof(Vec3fa); 
    size_t bytesTotal = bytesAlignedNodes+bytesUnalignedNodes+bytesPrims+bytesVertices;
    size_t bytesTotalAllocated = bvh->alloc.bytes();
    if (g_benchmark) std::cout << "BENCHMARK_HAIR_ACCEL " << bvhSAH << " " << bytesTotal << std::endl;
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream << "  primitives = " << bvh->numPrimitives << ", vertices = " << bvh->numVertices << std::endl;
    stream.precision(4);
    stream << "  sah = " << bvhSAH;
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(1);
    stream << ", depth = " << depth << std::endl;
    stream << "  used = " << bytesTotal/1E6 << " MB, allocated = " << bytesTotalAllocated/1E6 << " MB, perPrimitive = " << double(bytesTotal)/double(bvh->numPrimitives) << " B" << std::endl;
    stream.precision(1);
    stream << "  alignedNodes = "  << numAlignedNodes << " "
           << "(" << 100.0*double(childrenAlignedNodes)/double(BVH4Hair::N*numAlignedNodes) << "% filled) " 
           << "(" << bytesAlignedNodes/1E6  << " MB) " 
           << "(" << 100.0*double(bytesAlignedNodes)/double(bytesTotal) << "% of total)"
           << std::endl;
    stream << "  unalignedNodes = "  << numUnalignedNodes << " "
           << "(" << 100.0*double(childrenUnalignedNodes)/double(BVH4Hair::N*numUnalignedNodes) << "% filled) " 
           << "(" << bytesUnalignedNodes/1E6  << " MB) " 
           << "(" << 100.0*double(bytesUnalignedNodes)/double(bytesTotal) << "% of total)"
           << std::endl;
    stream << "  leaves = " << numLeaves << " "
           << "(" << bytesPrims/1E6  << " MB) "
           << "(" << 100.0*double(bytesPrims)/double(bytesTotal) << "% of total)"
           << std::endl;
    stream << "  vertices = " << numVertices << " "
           << "(" << bytesVertices/1E6 << " MB) " 
           << "(" << 100.0*double(bytesVertices)/double(bytesTotal) << "% of total) "
           << "(" << 100.0*12.0f/float(sizeof(Vec3fa)) << "% used)" 
           << std::endl;
    return stream.str();
  }

  void BVH4HairStatistics::statistics(NodeRef node, const float A, size_t& depth)
  {
    if (node.isAlignedNode())
    {
      numAlignedNodes++;
      AlignedNode* n = node.alignedNode();
      bvhSAH += A*BVH4Hair::travCostAligned;

      depth = 0;
      for (size_t i=0; i<BVH4Hair::N; i++) {
        if (n->child(i) != BVH4Hair::emptyNode) childrenAlignedNodes++;
        const float Ai = max(0.0f,halfArea(n->extend(i)));
        size_t cdepth; statistics(n->child(i),Ai,cdepth); 
        depth=max(depth,cdepth);
      }
      depth++;
    }
    else if (node.isUnalignedNode())
    {
      numUnalignedNodes++;
      UnalignedNode* n = node.unalignedNode();
      bvhSAH += A*BVH4Hair::travCostUnaligned;

      depth = 0;
      for (size_t i=0; i<BVH4Hair::N; i++) {
        if (n->child(i) != BVH4Hair::emptyNode) childrenUnalignedNodes++;
        const float Ai = max(0.0f,halfArea(n->extend(i)));
        size_t cdepth; statistics(n->child(i),Ai,cdepth); 
        depth=max(depth,cdepth);
      }
      depth++;
    }
    else
    {
      depth = 0;
      size_t num; const char* tri = node.leaf(num);
      if (!num) return;
      
      numLeaves++;
      numPrims += num;
      float sah = A * BVH4Hair::intCost * num;
      bvhSAH += sah;
    }
  }
}
