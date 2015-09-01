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
              const SubdivMesh* const geom, const size_t bvhBytes, BBox3fa* bounds_o = nullptr);

      static __forceinline size_t calculate_grid_size(size_t width, size_t height) {
        return ((width*height+vfloat::size-1)&(-vfloat::size)) / vfloat::size * vfloat::size;
      }

      /*! Subgrid creation */
      template<typename Allocator>
        static GridSOA* create(SubdivPatch1Base* const patch, unsigned x0, unsigned x1, unsigned y0, unsigned y1, const Scene* scene, Allocator& alloc, BBox3fa* bounds_o = nullptr)
      {
        const size_t width = x1-x0+1;
        const size_t height = y1-y0+1;
        const GridRange range(0,width-1,0,height-1);
        const size_t bvhBytes  = getBVHBytes(range,0);
        const size_t gridBytes = 4*4*calculate_grid_size(width,height);
        return new (alloc(offsetof(GridSOA,data)+bvhBytes+gridBytes)) GridSOA(*patch,x0,x1,y0,y1,patch->grid_u_res,patch->grid_v_res,scene->getSubdivMesh(patch->geom),bvhBytes,bounds_o);  
      }

      /*! Grid creation */
      template<typename Allocator>
        static GridSOA* create(SubdivPatch1Base* const patch, const Scene* scene, const Allocator& alloc, BBox3fa* bounds_o = nullptr) 
      {
        return create(patch,0,patch->grid_u_res-1,0,patch->grid_v_res-1,scene,alloc,bounds_o);
      }

      static size_t getNumEagerLeaves(size_t width, size_t height) {
        const size_t w = (((width +1)/2)+3)/4;
        const size_t h = (((height+1)/2)+3)/4;
        return w*h;
      }

      template<typename Allocator>
        __forceinline static size_t createEager(SubdivPatch1Base& patch, Scene* scene, SubdivMesh* mesh, size_t primID, Allocator& alloc, PrimRef* prims)
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
            BBox3fa bounds;
            GridSOA* leaf = create(&patch,lx0,lx1,ly0,ly1,scene,alloc,&bounds);
            *prims = PrimRef(bounds,(size_t)BVH4::encodeTypedLeaf(leaf,1)); prims++;
            N++;
          }
        }
        return N;
      }

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
      BVH4::NodeRef buildBVH(char* node_array, float* grid_array, const size_t bvhBytes, BBox3fa* bounds_o);
      
      /*! Create BVH4 tree over grid. */
      BBox3fa buildBVH(BVH4::NodeRef& curNode, char* node_array, float* grid_array, const GridRange& range, size_t& localCounter);

      struct Gather2x3
      {
        typedef bool4 vbool;
        typedef int4 vint;
        typedef float4 vfloat;
        
        static __forceinline const Vec3<float4> gather(const float* const grid, const size_t line_offset)
        {
          const float4 r0 = loadu4f(grid + 0*line_offset); 
          const float4 r1 = loadu4f(grid + 1*line_offset); // FIXME: this accesses 1 element too much
          return Vec3<float4>(unpacklo(r0,r1),       // r00, r10, r01, r11  
                              shuffle<1,1,2,2>(r0),  // r01, r01, r02, r02
                              shuffle<0,1,1,2>(r1)); // r10, r11, r11, r12
        }
      };
      
#if defined (__AVX__)
      struct Gather3x3
      {
        typedef bool8 vbool;
        typedef int8 vint;
        typedef float8 vfloat;
        
        static __forceinline const Vec3<float8> gather(const float* const grid, const size_t line_offset)
        {
          const float4 ra = loadu4f(grid + 0*line_offset);
          const float4 rb = loadu4f(grid + 1*line_offset);
          const float4 rc = loadu4f(grid + 2*line_offset); // FIXME: this accesses 1 element too much
          const float8 r0 = float8(ra,rb);
          const float8 r1 = float8(rb,rc);
          return Vec3<float8>(unpacklo(r0,r1),         // r00, r10, r01, r11, r10, r20, r11, r21   
                              shuffle<1,1,2,2>(r0),    // r01, r01, r02, r02, r11, r11, r12, r12
                              shuffle<0,1,1,2>(r1));   // r10, r11, r11, r12, r20, r21, r21, r22
        }
      };
#endif

      template<typename vfloat>
      static __forceinline Vec2<vfloat> decodeUV(const vfloat& uv)
      {
        typedef typename vfloat::Int vint;
	const vint iu  = cast(uv) & 0xffff;
	const vint iv  = srl(cast(uv),16);
	const vfloat u = (vfloat)iu * vfloat(1.0f/0xFFFF);
	const vfloat v = (vfloat)iv * vfloat(1.0f/0xFFFF);
	return Vec2<vfloat>(u,v);
      }

    public:
      BVH4::NodeRef root;
#if !defined (__X86_64__)
      unsigned align0;
#endif
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
