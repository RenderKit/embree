// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../common/ray.h"
#include "../common/scene_subdiv_mesh.h"
#include "filter.h"
#include "../bvh/bvh.h"
#include "../subdiv/tessellation.h"
#include "../subdiv/tessellation_cache.h"
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
      GridSOA(const SubdivPatch1Base* patches, const unsigned time_steps,
              const unsigned x0, const unsigned x1, const unsigned y0, const unsigned y1, const unsigned swidth, const unsigned sheight,
              const SubdivMesh* const geom, const size_t bvhBytes, const size_t gridBytes, BBox3fa* bounds_o = nullptr);

      /*! Subgrid creation */
      template<typename Allocator>
        static GridSOA* create(const SubdivPatch1Base* patches, const unsigned time_steps,
                               unsigned x0, unsigned x1, unsigned y0, unsigned y1, 
                               const Scene* scene, Allocator& alloc, BBox3fa* bounds_o = nullptr)
      {
        const unsigned width = x1-x0+1;
        const unsigned height = y1-y0+1;
        const GridRange range(0,width-1,0,height-1);
        const size_t bvhBytes  = getBVHBytes(range,0);
        const size_t gridBytes = 4*size_t(width)*size_t(height)*sizeof(float)+4; // 4 bytes of padding required because of off by 1 read below
        void* data = alloc(offsetof(GridSOA,data)+time_steps*bvhBytes+time_steps*gridBytes);
        return new (data) GridSOA(patches,time_steps,x0,x1,y0,y1,patches->grid_u_res,patches->grid_v_res,scene->getSubdivMesh(patches->geom),bvhBytes,gridBytes,bounds_o);  
      }

      /*! Grid creation */
      template<typename Allocator>
        static GridSOA* create(const SubdivPatch1Base* const patches, const unsigned time_steps,
                               const Scene* scene, const Allocator& alloc, BBox3fa* bounds_o = nullptr) 
      {
        return create(patches,time_steps,0,patches->grid_u_res-1,0,patches->grid_v_res-1,scene,alloc,bounds_o);
      }

      static unsigned getNumEagerLeaves(unsigned width, unsigned height) {
        const unsigned w = (((width +1)/2)+3)/4;
        const unsigned h = (((height+1)/2)+3)/4;
        return w*h;
      }

      template<typename Allocator>
        __forceinline static unsigned createEager(SubdivPatch1Base& patch, Scene* scene, SubdivMesh* mesh, unsigned primID, Allocator& alloc, PrimRef* prims)
      {
        unsigned N = 0;
        const unsigned x0 = 0, x1 = patch.grid_u_res-1;
        const unsigned y0 = 0, y1 = patch.grid_v_res-1;
        
        for (unsigned y=y0; y<y1; y+=8)
        {
          for (unsigned x=x0; x<x1; x+=8) 
          {
            const unsigned lx0 = x, lx1 = min(lx0+8,x1);
            const unsigned ly0 = y, ly1 = min(ly0+8,y1);
            BBox3fa bounds;
            GridSOA* leaf = create(&patch,1,lx0,lx1,ly0,ly1,scene,alloc,&bounds);
            *prims = PrimRef(bounds,(unsigned)BVH4::encodeTypedLeaf(leaf,1)); prims++;
            N++;
          }
        }
        return N;
      }

      /*! returns pointer to BVH array */
      __forceinline       char* bvhData(size_t t = 0)       { return &data[t*bvhBytes]; }
      __forceinline const char* bvhData(size_t t = 0) const { return &data[t*bvhBytes]; }

      /*! returns pointer to Grid array */
      __forceinline       float* gridData(size_t t = 0)       { return (float*) &data[time_steps*bvhBytes + t*gridBytes]; }
      __forceinline const float* gridData(size_t t = 0) const { return (float*) &data[time_steps*bvhBytes + t*gridBytes]; }
      
      __forceinline void* encodeLeaf(size_t u, size_t v) {
        return (void*) (16*(v * width + u + 1)); // +1 to not create empty leaf
      }
      __forceinline float* decodeLeaf(size_t t, const void* ptr) {
        return gridData(t) + (((size_t) (ptr) >> 4) - 1);
      }

      /*! returns the size of the BVH over the grid in bytes */
      static size_t getBVHBytes(const GridRange& range, const unsigned int leafBytes);

      /*! calculates bounding box of grid range */
      __forceinline BBox3fa calculateBounds(size_t time, const GridRange& range) const
      {
        const float* const grid_array = gridData(time);
        const float* const grid_x_array = grid_array + 0 * dim_offset;
        const float* const grid_y_array = grid_array + 1 * dim_offset;
        const float* const grid_z_array = grid_array + 2 * dim_offset;
        
        /* compute the bounds just for the range! */
        BBox3fa bounds( empty );
        for (unsigned v = range.v_start; v<=range.v_end; v++) 
        {
          for (unsigned u = range.u_start; u<=range.u_end; u++)
          {
            const float x = grid_x_array[ v * width + u];
            const float y = grid_y_array[ v * width + u];
            const float z = grid_z_array[ v * width + u];
            bounds.extend( Vec3fa(x,y,z) );
          }
        }
        assert(is_finite(bounds));
        return bounds;
      }

      /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
      BVH4::NodeRef buildBVH(size_t time, BBox3fa* bounds_o);
      
      /*! Create BVH4 tree over grid. */
      BBox3fa buildBVH(BVH4::NodeRef& curNode, size_t time, const GridRange& range, size_t& allocator);

      /*! Evaluates grid over patch and builds MBlur BVH4 tree over the grid. */
      BVH4::NodeRef buildMBlurBVH(size_t time, std::pair<BBox3fa,BBox3fa>* bounds_o);
      
      /*! Create MBlur BVH4 tree over grid. */
      std::pair<BBox3fa,BBox3fa> buildMBlurBVH(BVH4::NodeRef& curNode, size_t time, const GridRange& range, size_t& allocator);

      struct Gather2x3
      {
        enum { M = 4 };
        typedef vbool4 vbool;
        typedef vint4 vint;
        typedef vfloat4 vfloat;
        
        static __forceinline const Vec3<vfloat4> gather(const float* const grid, const size_t line_offset)
        {
          const vfloat4 r0 = vfloat4::loadu(grid + 0*line_offset);
          const vfloat4 r1 = vfloat4::loadu(grid + 1*line_offset); // this accesses 1 element too much, but this is ok as we ensure enough padding after the grid
          return Vec3<vfloat4>(unpacklo(r0,r1),       // r00, r10, r01, r11
                               shuffle<1,1,2,2>(r0),  // r01, r01, r02, r02
                               shuffle<0,1,1,2>(r1)); // r10, r11, r11, r12
        }
      };
      
#if defined (__AVX__)
      struct Gather3x3
      {
        enum { M = 8 };
        typedef vbool8 vbool;
        typedef vint8 vint;
        typedef vfloat8 vfloat;
        
        static __forceinline const Vec3<vfloat8> gather(const float* const grid, const size_t line_offset)
        {
          const vfloat4 ra = vfloat4::loadu(grid + 0*line_offset);
          const vfloat4 rb = vfloat4::loadu(grid + 1*line_offset);
          const vfloat4 rc = vfloat4::loadu(grid + 2*line_offset); // this accesses 1 element too much, but this is ok as we ensure enough padding after the grid
          const vfloat8 r0 = vfloat8(ra,rb);
          const vfloat8 r1 = vfloat8(rb,rc);
          return Vec3<vfloat8>(unpacklo(r0,r1),         // r00, r10, r01, r11, r10, r20, r11, r21
                               shuffle<1,1,2,2>(r0),    // r01, r01, r02, r02, r11, r11, r12, r12
                               shuffle<0,1,1,2>(r1));   // r10, r11, r11, r12, r20, r21, r21, r22
        }
      };
#endif

      template<typename vfloat>
      static __forceinline Vec2<vfloat> decodeUV(const vfloat& uv)
      {
        typedef typename vfloat::Int vint;
        const vint iu  = asInt(uv) & 0xffff;
        const vint iv  = srl(asInt(uv),16);
	const vfloat u = (vfloat)iu * vfloat(1.0f/0xFFFF);
	const vfloat v = (vfloat)iv * vfloat(1.0f/0xFFFF);
	return Vec2<vfloat>(u,v);
      }

    public:
      BVH4::NodeRef root;
#if !defined (__X86_64__)
      unsigned align0;
#endif
      unsigned time_steps;
      unsigned width;

      unsigned height;
      unsigned dim_offset;
      unsigned geomID;
      unsigned primID;

      unsigned bvhBytes;
      unsigned gridBytes;
      unsigned align0;
      unsigned align1;

      char data[1];        //!< after the struct we first store the BVH and then the grid
    };
  }
}
