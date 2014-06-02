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

#include "bvh4hair.h"
#include "bvh4hair_builder.h"
#include "common/accelinstance.h"
#include "geometry/triangle1.h"

namespace embree
{
#define DBG(x) x

  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4HairIntersector1Bezier1i);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4HairIntersector16Bezier1i);

  void BVH4HairRegister () 
  {
    int features = getCPUFeatures();

    SELECT_SYMBOL_KNC(features,BVH4HairIntersector1Bezier1i);
    SELECT_SYMBOL_KNC(features,BVH4HairIntersector16Bezier1i);

  }

  Accel::Intersectors BVH4HairIntersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4HairIntersector1Bezier1i;
    intersectors.intersector16 = BVH4HairIntersector16Bezier1i;
    return intersectors;
  }

  Accel* BVH4Hair::BVH4HairBinnedSAH(Scene* scene)
  {
    BVH4Hair* accel  = new BVH4Hair(SceneTriangle1::type,scene);    
    Builder* builder = new BVH4HairBuilder(accel,NULL,scene);   
    Accel::Intersectors intersectors = BVH4HairIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }

  // =================================================================================
  // =================================================================================
  // =================================================================================


  __aligned(64) float BVH4Hair::UnalignedNode::identityMatrix[16] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
  };

  __aligned(64) float BVH4Hair::UnalignedNode::invalidMatrix[16] = {
    NAN,0,0,0,
    0,NAN,0,0,
    0,0,NAN,0,
    NAN,NAN,NAN,NAN
  };

  void BVH4Hair::UnalignedNode::convertFromBVH4iNode(const BVH4i::Node &bvh4i_node, UnalignedNode *ptr)
  {    
    setIdentityMatrix();
    for (size_t m=0;m<4;m++)
      {
	const float dx = bvh4i_node.upper[m].x - bvh4i_node.lower[m].x;
	const float dy = bvh4i_node.upper[m].y - bvh4i_node.lower[m].y;
	const float dz = bvh4i_node.upper[m].z - bvh4i_node.lower[m].z;
	const float inv_dx = 1.0f / dx;
	const float inv_dy = 1.0f / dy;
	const float inv_dz = 1.0f / dz;
	const float min_x = bvh4i_node.lower[m].x;
	const float min_y = bvh4i_node.lower[m].y;
	const float min_z = bvh4i_node.lower[m].z;
	set_scale(inv_dx,inv_dy,inv_dz,m);
	set_translation(-min_x*inv_dx,-min_y*inv_dy,-min_z*inv_dz,m);

	BVH4i::NodeRef n4i = bvh4i_node.child(m);
	if (n4i.isLeaf())
	  {
	    const size_t index = n4i.offsetIndex();
	    const size_t items = n4i.items();
	    if (n4i != BVH4i::emptyNode)
	      child(m) = (size_t)n4i;
	    else
	      child(m) = (size_t)-1;
	  }
	else
	  {
	    const size_t nodeID = n4i.nodeID();
	    child(m) = (size_t)&ptr[nodeID]; 	    
	  }
	geomID[m] = 0;
	primID[m] = 0;
      }
  }




};
