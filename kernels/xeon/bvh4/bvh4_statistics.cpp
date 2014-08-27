// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "bvh4_statistics.h"

namespace embree
{
  BVH4Statistics::BVH4Statistics (BVH4* bvh) : bvh(bvh)
  {
    numNodes = numNodesMB = numLeaves = numPrimBlocks = numPrims = depth = 0;
    numNodesChildren = numNodesMBChildren = 0;
    bvhSAH = leafSAH = 0.0f;
    statistics(bvh->root,bvh->bounds,depth);
    bvhSAH /= area(bvh->bounds);
    leafSAH /= area(bvh->bounds);
    assert(depth <= BVH4::maxDepth);
  }

  size_t BVH4Statistics::bytesUsed()
  {
    size_t bytesNodes = numNodes*sizeof(BVH4::Node);
    size_t bytesNodesMB = numNodesMB*sizeof(BVH4::NodeMB);
    size_t bytesPrimitives = numPrimBlocks*bvh->primTy.bytes;
    size_t numVertices = bvh->numVertices;
    size_t bytesVertices = numVertices*sizeof(Vec3fa); 
    return bytesNodes+bytesNodesMB+bytesPrimitives+bytesVertices;
  }

  std::string BVH4Statistics::str()  
  {
    std::ostringstream stream;
    size_t bytesNodes = numNodes*sizeof(BVH4::Node);
    size_t bytesNodesMB = numNodesMB*sizeof(BVH4::NodeMB);
    size_t bytesPrimitives = numPrimBlocks*bvh->primTy.bytes;
    size_t numVertices = bvh->numVertices;
    size_t bytesVertices = numVertices*sizeof(Vec3fa); 
    size_t bytesTotal = bytesNodes+bytesNodesMB+bytesPrimitives+bytesVertices;
    size_t bytesTotalAllocated = bvh->bytesAllocated();
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream << "  primitives = " << bvh->numPrimitives << ", vertices = " << bvh->numVertices << std::endl;
    stream.setf(std::ios::scientific, std::ios::floatfield);
    stream.precision(4);
    stream << "  sah = " << bvhSAH << ", leafSAH = " << leafSAH;
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(1);
    stream << ", depth = " << depth << std::endl;
    stream << "  used = " << bytesTotal/1E6 << " MB, allocated = " << bytesTotalAllocated/1E6 << " MB, perPrimitive = " << double(bytesTotal)/double(bvh->numPrimitives) << " B" << std::endl;
    stream.precision(1);
    
    if (numNodes) {
      stream << "  nodes = "  << numNodes << " "
	     << "(" << bytesNodes/1E6  << " MB) "
	     << "(" << 100.0*double(bytesNodes)/double(bytesTotal) << "% of total) "
	     << "(" << 100.0*numNodes/numNodesChildren << "% used)" 
	     << std::endl;
    }
    
    if (numNodesMB) {
      stream << "  nodesMB = "  << numNodesMB << " "
	     << "(" << bytesNodesMB/1E6  << " MB) "
	     << "(" << 100.0*double(bytesNodesMB)/double(bytesTotal) << "% of total) "
	     << "(" << 100.0*numNodesMB/numNodesMBChildren << "% used)" 
	     << std::endl;
    }

    stream << "  leaves = " << numLeaves << " "
           << "(" << bytesPrimitives/1E6  << " MB) "
           << "(" << 100.0*double(bytesPrimitives)/double(bytesTotal) << "% of total) "
           << "(" << 100.0*double(numPrims)/double(bvh->primTy.blockSize*numPrimBlocks) << "% used)" 
           << std::endl;

    stream << "  vertices = " << numVertices << " "
           << "(" << bytesVertices/1E6 << " MB) " 
           << "(" << 100.0*double(bytesVertices)/double(bytesTotal) << "% of total) "
           << "(" << 100.0*12.0f/float(sizeof(Vec3fa)) << "% used)" 
           << std::endl;
    return stream.str();
  }

  void BVH4Statistics::statistics(BVH4::NodeRef node, const BBox3fa& bounds, size_t& depth)
  {
    float A = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode())
    {
      numNodes++;
      depth = 0;
      size_t cdepth = 0;
      BVH4::Node* n = node.node();
      bvhSAH += A*BVH4::travCost;
      for (size_t i=0; i<BVH4::N; i++) {
	if (n->child(i) != BVH4::emptyNode) numNodesChildren++;
        statistics(n->child(i),n->bounds(i),cdepth); 
        depth=max(depth,cdepth);
      }
      for (size_t i=0; i<BVH4::N; i++) {
        if (n->child(i) == BVH4::emptyNode) {
          for (; i<BVH4::N; i++) {
            if (n->child(i) != BVH4::emptyNode)
	      throw std::runtime_error("invalid node");
          }
          break;
        }
      }    
      depth++;
      return;
    }
    else if (node.isNodeMB())
    {
      numNodesMB++;
      depth = 0;
      size_t cdepth = 0;
      BVH4::NodeMB* n = node.nodeMB();
      bvhSAH += A*BVH4::travCost;
      for (size_t i=0; i<BVH4::N; i++) {
	if (n->child(i) != BVH4::emptyNode) numNodesMBChildren++;
        statistics(n->child(i),n->bounds0(i),cdepth); 
        depth=max(depth,cdepth);
      }
      for (size_t i=0; i<BVH4::N; i++) {
        if (n->child(i) == BVH4::emptyNode) {
          for (; i<BVH4::N; i++) {
            if (n->child(i) != BVH4::emptyNode)
	      throw std::runtime_error("invalid node");
          }
          break;
        }
      }    
      depth++;
      return;
    }
    else
    {
      depth = 0;
      size_t num; const char* tri = node.leaf(num);
      if (!num) return;
      
      numLeaves++;
      numPrimBlocks += num;
      for (size_t i=0; i<num; i++) {
        numPrims += bvh->primTy.size(tri+i*bvh->primTy.bytes);
      }
      float sah = A * bvh->primTy.intCost * num;
      bvhSAH += sah;
      leafSAH += sah;
    }
  }
}
