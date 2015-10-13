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

#include "bvh_statistics.h"
#include <sstream>

namespace embree
{
  static const int travCostAligned = 1;
  static const int travCostUnaligned = 1;
  static const int travCostTransform = 1;
  static const int intCost = 1;

  template<int N>
  BVHNStatistics<N>::BVHNStatistics (BVH* bvh) : bvh(bvh)
  {
    numAlignedNodes = numUnalignedNodes = 0;
    numAlignedNodesMB = numUnalignedNodesMB = 0;
    numTransformNodes = 0;
    numLeaves = numPrims = numPrimBlocks = depth = 0;
    childrenAlignedNodes = childrenUnalignedNodes = 0;
    childrenAlignedNodesMB = childrenUnalignedNodesMB = 0;
    bvhSAH = 0.0f; leafSAH = 0.0f;
    float A = max(0.0f,halfArea(bvh->bounds));
    statistics(bvh->root,A,depth);
    bvhSAH /= halfArea(bvh->bounds);
    leafSAH /= halfArea(bvh->bounds);
    assert(depth <= BVH::maxDepth);
  }
  
  template<int N>
  size_t BVHNStatistics<N>::bytesUsed() const
  {
    size_t bytesAlignedNodes = numAlignedNodes*sizeof(AlignedNode);
    size_t bytesUnalignedNodes = numUnalignedNodes*sizeof(UnalignedNode);
    size_t bytesAlignedNodesMB = numAlignedNodesMB*sizeof(AlignedNodeMB);
    size_t bytesUnalignedNodesMB = numUnalignedNodesMB*sizeof(UnalignedNodeMB);
    size_t bytesTransformNodes = numTransformNodes*sizeof(TransformNode);
    size_t bytesPrims  = numPrimBlocks*bvh->primTy.bytes;
    size_t numVertices = bvh->numVertices;
    size_t bytesVertices = numVertices*sizeof(Vec3fa); 
    return bytesAlignedNodes+bytesUnalignedNodes+bytesAlignedNodesMB+bytesUnalignedNodesMB+bytesTransformNodes+bytesPrims+bytesVertices;
  }
  
  template<int N>
  std::string BVHNStatistics<N>::str()
  {
    std::ostringstream stream;
    size_t bytesAlignedNodes = numAlignedNodes*sizeof(AlignedNode);
    size_t bytesUnalignedNodes = numUnalignedNodes*sizeof(UnalignedNode);
    size_t bytesAlignedNodesMB = numAlignedNodesMB*sizeof(AlignedNodeMB);
    size_t bytesUnalignedNodesMB = numUnalignedNodesMB*sizeof(UnalignedNodeMB);
    size_t bytesTransformNodes = numTransformNodes*sizeof(TransformNode);
    size_t bytesPrims  = numPrimBlocks*bvh->primTy.bytes;
    size_t numVertices = bvh->numVertices;
    size_t bytesVertices = numVertices*sizeof(Vec3fa); 
    size_t bytesTotal = bytesAlignedNodes+bytesUnalignedNodes+bytesAlignedNodesMB+bytesUnalignedNodesMB+bytesTransformNodes+bytesPrims+bytesVertices;
    //size_t bytesTotalAllocated = bvh->alloc.bytes();
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream << "  primitives = " << bvh->numPrimitives << ", vertices = " << bvh->numVertices << std::endl;
    stream.precision(4);
    stream << "  sah = " << bvhSAH+leafSAH << " (" << bvhSAH << " + " << leafSAH << ")";
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(1);
    stream << ", depth = " << depth << std::endl;
    stream << "  used = " << bytesTotal/1E6 << " MB, perPrimitive = " << double(bytesTotal)/double(bvh->numPrimitives) << " B" << std::endl;
    stream.precision(1);
    if (numAlignedNodes) {
      stream << "  alignedNodes = "  << numAlignedNodes << " "
             << "(" << 100.0*double(childrenAlignedNodes)/double(N*numAlignedNodes) << "% filled) "
	     << "(" << bytesAlignedNodes/1E6  << " MB) " 
	     << "(" << 100.0*double(bytesAlignedNodes)/double(bytesTotal) << "% of total)"
	     << std::endl;
    }
    if (numUnalignedNodes) {
      stream << "  unalignedNodes = "  << numUnalignedNodes << " "
             << "(" << 100.0*double(childrenUnalignedNodes)/double(N*numUnalignedNodes) << "% filled) "
	     << "(" << bytesUnalignedNodes/1E6  << " MB) " 
	     << "(" << 100.0*double(bytesUnalignedNodes)/double(bytesTotal) << "% of total)"
	     << std::endl;
    }
    if (numAlignedNodesMB) {
      stream << "  alignedNodesMB = "  << numAlignedNodesMB << " "
             << "(" << 100.0*double(childrenAlignedNodesMB)/double(N*numAlignedNodesMB) << "% filled) "
	     << "(" << bytesAlignedNodesMB/1E6  << " MB) " 
	     << "(" << 100.0*double(bytesAlignedNodesMB)/double(bytesTotal) << "% of total)"
	     << std::endl;
    }
    if (numUnalignedNodesMB) {
      stream << "  unalignedNodesMB = "  << numUnalignedNodesMB << " "
             << "(" << 100.0*double(childrenUnalignedNodesMB)/double(N*numUnalignedNodesMB) << "% filled) "
	     << "(" << bytesUnalignedNodesMB/1E6  << " MB) " 
	     << "(" << 100.0*double(bytesUnalignedNodesMB)/double(bytesTotal) << "% of total)"
	     << std::endl;
    }
    if (numTransformNodes) {
      stream << "  transformNodes = "  << numTransformNodes << " "
	     << "(" << bytesTransformNodes/1E6  << " MB) " 
	     << "(" << 100.0*double(bytesTransformNodes)/double(bytesTotal) << "% of total)"
	     << std::endl;
    }
    stream << "  leaves = " << numLeaves << " "
           << "(" << bytesPrims/1E6  << " MB) "
           << "(" << 100.0*double(bytesPrims)/double(bytesTotal) << "% of total)"
           << "(" << 100.0*double(numPrims)/double(bvh->primTy.blockSize*numPrimBlocks) << "% used)" 
           << std::endl;
    stream << "  vertices = " << numVertices << " "
           << "(" << bytesVertices/1E6 << " MB) " 
           << "(" << 100.0*double(bytesVertices)/double(bytesTotal) << "% of total) "
           << "(" << 100.0*12.0f/float(sizeof(Vec3fa)) << "% used)" 
           << std::endl;
    return stream.str();
  }
  
  template<int N>
  void BVHNStatistics<N>::statistics(NodeRef node, const float A, size_t& depth)
  {
    if (node.isNode())
    {
      numAlignedNodes++;
      AlignedNode* n = node.node();
      bvhSAH += A*travCostAligned;
      depth = 0;
      for (size_t i=0; i<N; i++) {
        if (n->child(i) == BVH::emptyNode) continue;
        childrenAlignedNodes++;
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
      bvhSAH += A*travCostUnaligned;
      
      depth = 0;
      for (size_t i=0; i<N; i++) {
        if (n->child(i) == BVH::emptyNode) continue;
        childrenUnalignedNodes++;
        const float Ai = max(0.0f,halfArea(n->extend(i)));
        size_t cdepth; statistics(n->child(i),Ai,cdepth); 
        depth=max(depth,cdepth);
      }
      depth++;
    }
    else if (node.isNodeMB())
    {
      numAlignedNodesMB++;
      AlignedNodeMB* n = node.nodeMB();
      bvhSAH += A*travCostAligned;
      
      depth = 0;
      for (size_t i=0; i<N; i++) {
        if (n->child(i) == BVH::emptyNode) continue;
        childrenAlignedNodesMB++;
        const float Ai = max(0.0f,halfArea(n->extend0(i)));
        size_t cdepth; statistics(n->child(i),Ai,cdepth); 
        depth=max(depth,cdepth);
      }
      depth++;
    }
    else if (node.isUnalignedNodeMB())
    {
      numUnalignedNodesMB++;
      UnalignedNodeMB* n = node.unalignedNodeMB();
      bvhSAH += A*travCostUnaligned;
      
      depth = 0;
      for (size_t i=0; i<N; i++) {
        if (n->child(i) == BVH::emptyNode) continue;
        childrenUnalignedNodesMB++;
        const float Ai = max(0.0f,halfArea(n->extend0(i)));
        size_t cdepth; statistics(n->child(i),Ai,cdepth); 
        depth=max(depth,cdepth);
      }
      depth++;
    }
    else if (node.isTransformNode())
    {
      numTransformNodes++;
      TransformNode* n = node.transformNode();
      bvhSAH += A*travCostTransform;

      depth = 0;
      const BBox3fa worldBounds = xfmBounds(n->local2world,n->localBounds);
      const float Ai = max(0.0f,halfArea(worldBounds));
      //size_t cdepth; statistics(n->child,Ai,cdepth); 
      //depth=max(depth,cdepth)+1;
    }
    else
    {
      depth = 0;
      size_t num; const char* tri = node.leaf(num);
      if (!num) return;
      
      numLeaves++;
      numPrimBlocks += num;
      for (size_t i=0; i<num; i++)
        numPrims += bvh->primTy.size(tri+i*bvh->primTy.bytes);
      
      float sah = A * intCost * num;
      leafSAH += sah;
    }
  } 

#if defined(__AVX__)
  template class BVHNStatistics<8>;
#else
  template class BVHNStatistics<4>;
#endif
}
