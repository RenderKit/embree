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

#if defined(USE_TBB)

#define PROFILE_MORTON_GENERAL

#include "bvh4.h"
#include "bvh4_builder_morton_general.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"


#define DBG(x) 


namespace embree 
{
  namespace isa
  {
    static double dt = 0.0f;
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderMortonGeneral::createSmallLeaf(BuildRecord& current, Allocator* alloc, BBox3fa& box_o)
    {
      ssef lower(pos_inf);
      ssef upper(neg_inf);
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
      *current.parent = bvh->encodeLeaf((char*)accel,listMode ? listMode : 1);
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t index = morton[start+i].index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
        upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,listMode));
      box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
    }

    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    

      Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
        return new class BVH4BuilderMortonGeneral ((BVH4*)bvh,scene,NULL,mode,2,false,sizeof(Triangle4),4,inf); 
      }
  }
}

#else

#include "common/builder.h"

namespace embree 
{
  namespace isa
  {
    Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
      return NULL;
    }
  }
}
#endif
  
  
