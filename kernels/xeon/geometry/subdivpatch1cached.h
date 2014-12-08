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

#pragma once

#include "primitive.h"
#include "common/subdiv/bspline_patch.h"
#include "common/subdiv/gregory_patch.h"
#include "common/subdiv/tessellation.h"

#define FORCE_TESSELLATION_BOUNDS 1
#define USE_DISPLACEMENT_FOR_TESSELLATION_BOUNDS 1

using namespace std;

namespace embree
{

#if defined(__AVX__)
  __forceinline BBox3fa getBBox3fa(const avx3f &v)
  {
    const Vec3fa b_min( reduce_min(v.x), reduce_min(v.y), reduce_min(v.z) );
    const Vec3fa b_max( reduce_max(v.x), reduce_max(v.y), reduce_max(v.z) );
    return BBox3fa( b_min, b_max );
  }
#else
  __forceinline BBox3fa getBBox3fa(const sse3f &v)
  {
    const Vec3fa b_min( reduce_min(v.x), reduce_min(v.y), reduce_min(v.z) );
    const Vec3fa b_max( reduce_max(v.x), reduce_max(v.y), reduce_max(v.z) );
    return BBox3fa( b_min, b_max );
  }
#endif

  struct GridRange
  {
    unsigned int u_start;
    unsigned int u_end;
    unsigned int v_start;
    unsigned int v_end;

    GridRange() {}

    GridRange(unsigned int u_start, unsigned int u_end, unsigned int v_start, unsigned int v_end) : u_start(u_start), u_end(u_end), v_start(v_start), v_end(v_end) {}

    __forceinline bool hasLeafSize() const
    {
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      return u_size <= 3 && v_size <= 3;
    }

    __forceinline unsigned int largestExtend() const
    {
      const int u_size = u_end-u_start+1;
      const int v_size = v_end-v_start+1;
      return max(u_size,v_size);
    }

    static __forceinline unsigned int split(unsigned int start,unsigned int end)
    {
      return (start+end)/2;
    }

    __forceinline bool split(GridRange &r0, GridRange &r1) const
    {
      if (hasLeafSize()) return false;
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      r0 = *this;
      r1 = *this;

      if (u_size >= v_size)
        {
          assert(u_size >= 3);
          const unsigned int u_mid = split(u_start,u_end);
          r0.u_end   = u_mid;
          r1.u_start = u_mid;
        }
      else
        {
          assert(v_size >= 3);
          const unsigned int v_mid = split(v_start,v_end);
          r0.v_end   = v_mid;
          r1.v_start = v_mid;
        }
      return true;
    }

    unsigned int splitIntoSubRanges(GridRange r[4]) const
    {
      unsigned int children = 1;
      r[0] = *this;
      while(children < 4)
        {
          ssize_t index = -1;
          ssize_t extend = 0;
          for (size_t i=0;i<children;i++)
            if (!r[i].hasLeafSize())
              if (r[i].largestExtend() > extend)
                {
                  extend = r[i].largestExtend();
                  index = i;
                }
          if (index == -1) break;

          GridRange tmp = r[index];
          tmp.split(r[index],r[children]);
          children++;          
        }
      return children;
    }

  };

  inline std::ostream& operator<<(std::ostream& cout, const GridRange& r) {
    cout << "range: u_start " << r.u_start << " u_end " << r.u_end << " v_start " << r.v_start << " v_end " << r.v_end << std::endl;
    return cout;
  }

  struct __aligned(64) SubdivPatch1Cached
  {
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t blocks(size_t x) const; 
      size_t size(const char* This) const;
    };

    static Type type;

  public:

    enum {
      REGULAR_PATCH     = 1,  // 0 => Gregory Patch 
      TRANSITION_PATCH  = 2,  // needs stiching?
      HAS_DISPLACEMENT  = 4   // 0 => no displacments
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1Cached () {}

    /*! Construction from vertices and IDs. */
    SubdivPatch1Cached (const CatmullClarkPatch& ipatch,
                        const unsigned int gID,
                        const unsigned int pID,
                        const SubdivMesh *const mesh);

    __forceinline bool needsStiching() const
    {
      return (flags & TRANSITION_PATCH) == TRANSITION_PATCH;      
    }

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      if (likely(isRegular()))
	return patch.eval(uu,vv);
      else 
	return GregoryPatch::eval( patch.v, uu, vv );
    }

    __forceinline sse3f eval4(const ssef &uu,
			      const ssef &vv) const
    {
      if (likely(isRegular()))
	return patch.eval4(uu,vv);
      else 
	return GregoryPatch::eval4( patch.v, uu, vv );
    }

#if defined(__AVX__)
    __forceinline avx3f eval8(const avxf &uu,
			      const avxf &vv) const
    {
      if (likely(isRegular()))
	return patch.eval8(uu,vv);
      else 
	return GregoryPatch::eval8( patch.v, uu, vv );
    }
#endif

    __forceinline Vec3fa normal(const float &uu,
				const float &vv) const
    {
      if (likely(isRegular()))
	return patch.normal(uu,vv);
      else 
	return GregoryPatch::normal( patch.v, uu, vv );
    }

    __forceinline sse3f normal4(const ssef &uu,
                                const ssef &vv) const
    {
      if (likely(isRegular()))
	return patch.normal4(uu,vv);
      else
        return GregoryPatch::normal4( patch.v, uu, vv );
    }

#if defined(__AVX__)
    __forceinline avx3f normal8(const avxf &uu,
                                const avxf &vv) const
    {
      if (likely(isRegular()))
	return patch.normal8(uu,vv);
      else
        return GregoryPatch::normal8( patch.v, uu, vv );
    }
#endif


    __forceinline bool isRegular() const
    {
      return (flags & REGULAR_PATCH) == REGULAR_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return !isRegular();
    }

    __forceinline bool hasDisplacement() const
    {
      return (flags & HAS_DISPLACEMENT) == HAS_DISPLACEMENT;
    }


    __forceinline void prefetchData() const
    {
      const char *const t = (char*)this;
      prefetchL1(t + 0*64);
      prefetchL1(t + 1*64);
      prefetchL1(t + 2*64);
      prefetchL1(t + 3*64);
      prefetchL1(t + 4*64);
    }

    __forceinline BBox3fa bounds(const SubdivMesh* const mesh) const
    {
#if FORCE_TESSELLATION_BOUNDS == 1

      __aligned(64) float u_array[(grid_size_simd_blocks+1)*8]; // +8 for unaligned access
      __aligned(64) float v_array[(grid_size_simd_blocks+1)*8]; // +8 for unaligned access

      const unsigned int real_grid_size = grid_u_res*grid_v_res;
      gridUVTessellator(level,grid_u_res,grid_v_res,u_array,v_array);
      
      if (unlikely(needsStiching()))
        stichUVGrid(level,grid_u_res,grid_v_res,u_array,v_array);

      // FIXME: remove
      for (size_t i=real_grid_size;i<grid_size_simd_blocks*8;i++)
        {
          u_array[i] = 1.0f;
          v_array[i] = 1.0f;
        }

      BBox3fa b ( empty );
      assert( grid_size_simd_blocks >= 1 );

#if !defined(__AVX__)

      for (size_t i=0;i<grid_size_simd_blocks*2;i++)
	{
	  ssef u = load4f(&u_array[i*4]);
	  ssef v = load4f(&v_array[i*4]);

	  sse3f vtx = eval4( u, v );

	  /* eval displacement function */
	  if (unlikely(mesh->displFunc != NULL))
	    {

	      sse3f nor = normal4(u,v);

	      nor = normalize_safe(nor);

	      mesh->displFunc(mesh->userPtr,
			      geom,
			      prim,
			      (const float*)&u_array[i*4],
			      (const float*)&v_array[i*4],
			      (const float*)&nor.x,
			      (const float*)&nor.y,
			      (const float*)&nor.z,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      4);
	    }
	  b.extend( getBBox3fa(vtx) );
	}


#else

      for (size_t i=0;i<grid_size_simd_blocks;i++)
	{
	  avxf u = load8f(&u_array[i*8]);
	  avxf v = load8f(&v_array[i*8]);

	  avx3f vtx = eval8( u, v );

	  /* eval displacement function */
	  if (unlikely(mesh->displFunc != NULL))
	    {

	      avx3f nor = normal8(u,v);

	      nor = normalize_safe(nor);

	      mesh->displFunc(mesh->userPtr,
			      geom,
			      prim,
			      (const float*)&u_array[i*8],
			      (const float*)&v_array[i*8],
			      (const float*)&nor.x,
			      (const float*)&nor.y,
			      (const float*)&nor.z,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      8);
	    }
	  b.extend( getBBox3fa(vtx) );
	}
#endif

      b.lower.a = 0.0f;
      b.upper.a = 0.0f;
     
#if DEBUG
      isfinite(b.lower.x);
      isfinite(b.lower.y);
      isfinite(b.lower.z);

      isfinite(b.upper.x);
      isfinite(b.upper.y);
      isfinite(b.upper.z);
#endif
      
#else
      BBox3fa b = patch.bounds();
      if (unlikely(isGregoryPatch()))
	{
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,0) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,1) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,2) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,3) );
	}
#endif

      return b;
    }


  private:

    size_t get64BytesBlocksForGridSubTree(const GridRange &range,
                                          const unsigned int leafBlocks)
    {
      if (range.hasLeafSize()) return leafBlocks;

      GridRange r[4];

      const unsigned int children = range.splitIntoSubRanges(r);

      size_t blocks = 2; /* 128 bytes bvh4 node layout */

      for (unsigned int i=0;i<children;i++)
	blocks += get64BytesBlocksForGridSubTree(r[i],
						 leafBlocks);
      return blocks;    
    }

    __forceinline unsigned int getSubTreeSize64bBlocks(const unsigned int leafBlocks = 2)
    {
      return get64BytesBlocksForGridSubTree(GridRange(0,grid_u_res-1,0,grid_v_res-1),leafBlocks);
    }

    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

  public:
    Vec2f u_range;
    Vec2f v_range;
    float level[4];

    unsigned int flags;
    unsigned int geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int prim;                          //!< primitive ID of this subdivision patch
    unsigned int reserved;

    unsigned int grid_u_res;
    unsigned int grid_v_res;
    unsigned int grid_size_simd_blocks;
    unsigned int grid_subtree_size_64b_blocks;

    __aligned(64) BSplinePatch patch;
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1Cached &p)
    {
      o << " flags " << p.flags << " geomID " << p.geom << " primID " << p.prim << " u_range << " << p.u_range << " v_range " << p.v_range << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3] << std::endl;
      o << " patch " << p.patch;

      return o;
    } 

}
