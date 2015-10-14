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

#include "bvh4i.h"

namespace embree
{
  __aligned(64) BVH4i::Node::Helper BVH4i::Node::initQBVHNode[4] = { 
    { pos_inf, pos_inf, pos_inf,BVH4i::invalidNode},
    { neg_inf, neg_inf, neg_inf,BVH4i::invalidNode},
    { pos_inf, pos_inf, pos_inf,BVH4i::invalidNode},
    { neg_inf, neg_inf, neg_inf,BVH4i::invalidNode}
   };

  BVH4i::~BVH4i()
  {
    if (qbvh)  os_free(qbvh,size_node);
    if (accel) os_free(accel,size_accel);
  }

  float BVH4i::sah () {
    return sah(root,bounds)/area(bounds);
  }

  float BVH4i::sah (NodeRef& node, BBox3fa bounds)
  {
    float f = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode()) 
    {
      Node* n = node.node(nodePtr());
      for (size_t c=0; c<BVH4i::N; c++) 
	if (n->child(c) != BVH4i::invalidNode)
	  f += sah(n->child(c),n->bounds(c));
      return f;
    }
    else 
    {
      unsigned int num; node.leaf(triPtr(),num);
      return f*num;
    }
  }
}
