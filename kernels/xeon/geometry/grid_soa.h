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

#include "../../common/ray.h"
#include "../../common/scene_subdiv_mesh.h"
#include "filter.h"
#include "../bvh4/bvh4.h"
#include "../../common/subdiv/tessellation.h"
#include "../../common/subdiv/tessellation_cache.h"
#include "subdivpatch1cached.h"
#include "grid_aos.h"

namespace embree
{
  namespace isa
  {
    class GridSOA
    {
    public:

      /*! GridSOA constructor */
      GridSOA(const SubdivPatch1Base& patch, 
              const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight,
              const SubdivMesh* const geom, const size_t bvhBytes);

      /*! performs cache lookup of grid BVH and builds grid if not in cache */
      template<typename Allocator>
        static void* create(SubdivPatch1Base* const patch, const Scene* scene, const Allocator& alloc)
      {
        const size_t width = patch->grid_u_res;
        const size_t height = patch->grid_v_res;
        const GridRange range(0,width-1,0,height-1);
        const size_t bvhBytes  = getBVHBytes(range,0);
        const size_t gridBytes = patch->getGridBytes();
        return new (alloc(offsetof(GridSOA,data)+bvhBytes+gridBytes)) GridSOA(*patch,0,width-1,0,height-1,width,height,scene->getSubdivMesh(patch->geom),bvhBytes);  
      }

      /*static size_t getNumEagerLeaves(size_t width, size_t height) {
        const size_t w = (((width +1)/2)+3)/4;
        const size_t h = (((height+1)/2)+3)/4;
        return w*h;
      }

      template<typename Allocator>
        __forceinline static size_t createEager(const SubdivPatch1Base& patch, Scene* scene, SubdivMesh* mesh, size_t primID, Allocator& alloc, PrimRef* prims)
      {
        size_t N = 0;
        const size_t x0 = 0, x1 = patch.grid_u_res-1;
        const size_t y0 = 0, y1 = patch.grid_v_res-1;
        
        for (size_t y=y0; y<y1; y+=8)
        {
          for (size_t x=x0; x<x1; x+=8) 
          {
            const size_t lx0 = x, lx1 = min(lx0+8,x1);
            const size_t ly0 = y, ly1 = min(ly0+8,y1);
            GridAOS* leaf = GridAOS::create(alloc,lx1-lx0+1,ly1-ly0+1,mesh->id,primID);
            leaf->build(scene,mesh,primID,patch,lx0,lx1,ly0,ly1);
            size_t n = leaf->createEagerPrims(alloc,prims,lx0,lx1,ly0,ly1);
            prims += n;
            N += n;
          }
        }
        return N;
        }*/

      /*! returns pointer to BVH array */
      __forceinline char* bvhData() {
        return &data[0];
      }

      /*! returns pointer to Grid array */
      __forceinline float* gridData() {
        return (float*) &data[bvhBytes];
      }
      
      /*! returns the size of the BVH over the grid in bytes */
      static size_t getBVHBytes(const GridRange& range, const unsigned int leafBytes);

      /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
      BVH4::NodeRef buildBVH(char* node_array, float* grid_array, const size_t bvhBytes);
      
      /*! Create BVH4 tree over grid. */
      BBox3fa buildBVH(BVH4::NodeRef& curNode, char* node_array, float* grid_array, const GridRange& range, size_t& localCounter);

    public:
      BVH4::NodeRef root;
      unsigned width;
      unsigned height;
      unsigned dim_offset;
      unsigned geomID;
      unsigned primID;
      unsigned bvhBytes;
      char data[1];        //!< after the struct we first store the BVH and then the grid
    };
  }
}
