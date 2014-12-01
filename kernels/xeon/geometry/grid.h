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
#include "discrete_tessellation.h"
//#include "common/subdiv/bspline_patch.h"
//#include "common/subdiv/gregory_patch.h"

namespace embree
{
  struct Grid
  {
    struct UncompressedBounds16
    {
      static const size_t N = 4;

      /*! Sets bounding box of child. */
      __forceinline void clear() 
      {
	for (size_t i=0; i<4; i++) {
	  lower_x[i] = lower_y[i] = lower_z[i] = pos_inf;
	  upper_x[i] = upper_y[i] = upper_z[i] = neg_inf;
	}
      }
      
      /*! Sets bounding box of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
        assert(i < 4*N);
        ((float*)&lower_x)[i] = bounds.lower.x; 
	((float*)&lower_y)[i] = bounds.lower.y; 
	((float*)&lower_z)[i] = bounds.lower.z;
        ((float*)&upper_x)[i] = bounds.upper.x; 
	((float*)&upper_y)[i] = bounds.upper.y; 
	((float*)&upper_z)[i] = bounds.upper.z;
      }
      
      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t i, size_t _nearX, size_t _nearY, size_t _nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t nearX = 4*_nearX, nearY = 4*_nearY, nearZ = 4*_nearZ; 
        const size_t farX  = nearX ^ (4*sizeof(ssef)), farY  = nearY ^ (4*sizeof(ssef)), farZ  = nearZ ^ (4*sizeof(ssef));
#if defined (__AVX2__)
        const ssef tNearX = msub(load4f((const char*)&lower_x[i]+nearX), rdir.x, org_rdir.x);
        const ssef tNearY = msub(load4f((const char*)&lower_x[i]+nearY), rdir.y, org_rdir.y);
        const ssef tNearZ = msub(load4f((const char*)&lower_x[i]+nearZ), rdir.z, org_rdir.z);
        const ssef tFarX  = msub(load4f((const char*)&lower_x[i]+farX ), rdir.x, org_rdir.x);
        const ssef tFarY  = msub(load4f((const char*)&lower_x[i]+farY ), rdir.y, org_rdir.y);
        const ssef tFarZ  = msub(load4f((const char*)&lower_x[i]+farZ ), rdir.z, org_rdir.z);
#else
        const ssef tNearX = (load4f((const char*)&lower_x[i]+nearX) - org.x) * rdir.x;
        const ssef tNearY = (load4f((const char*)&lower_x[i]+nearY) - org.y) * rdir.y;
        const ssef tNearZ = (load4f((const char*)&lower_x[i]+nearZ) - org.z) * rdir.z;
        const ssef tFarX  = (load4f((const char*)&lower_x[i]+farX ) - org.x) * rdir.x;
        const ssef tFarY  = (load4f((const char*)&lower_x[i]+farY ) - org.y) * rdir.y;
        const ssef tFarZ  = (load4f((const char*)&lower_x[i]+farZ ) - org.z) * rdir.z;
#endif

        if (robust) {
          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);
          const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
          const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
          const sseb vmask = round_down*tNear <= round_up*tFar;
          const size_t mask = movemask(vmask);
          return mask;
        }
        
#if defined(__SSE4_1__)
        const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
        const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
        const sseb vmask = cast(tNear) > cast(tFar);
        const size_t mask = movemask(vmask)^0xf;
#else
        const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
        const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const sseb vmask = tNear <= tFar;
        const size_t mask = movemask(vmask);
#endif
        return mask;
      }

      template<bool robust>
      __forceinline size_t intersect(size_t nearX, size_t nearY, size_t nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t mask0 = intersect<robust>(0, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask1 = intersect<robust>(1, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask2 = intersect<robust>(2, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask3 = intersect<robust>(3, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        return mask0 | (mask1 << 4) | (mask2 << 8) | (mask3 << 12);
      }

#if defined (__AVX__)

      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t i, size_t _nearX, size_t _nearY, size_t _nearZ,
                                     const avx3f& org, const avx3f& rdir, const avx3f& org_rdir, const avxf& tnear, const avxf& tfar) const
      {
        const size_t nearX = 4*_nearX, nearY = 4*_nearY, nearZ = 4*_nearZ; 
        const size_t farX  = nearX ^ (4*sizeof(ssef)), farY  = nearY ^ (4*sizeof(ssef)), farZ  = nearZ ^ (4*sizeof(ssef));
#if defined (__AVX2__)
        const avxf tNearX = msub(load8f((const char*)&lower_x[i]+nearX), rdir.x, org_rdir.x);
        const avxf tNearY = msub(load8f((const char*)&lower_x[i]+nearY), rdir.y, org_rdir.y);
        const avxf tNearZ = msub(load8f((const char*)&lower_x[i]+nearZ), rdir.z, org_rdir.z);
        const avxf tFarX  = msub(load8f((const char*)&lower_x[i]+farX ), rdir.x, org_rdir.x);
        const avxf tFarY  = msub(load8f((const char*)&lower_x[i]+farY ), rdir.y, org_rdir.y);
        const avxf tFarZ  = msub(load8f((const char*)&lower_x[i]+farZ ), rdir.z, org_rdir.z);
#else
        const avxf tNearX = (load8f((const char*)&lower_x[i]+nearX) - org.x) * rdir.x;
        const avxf tNearY = (load8f((const char*)&lower_x[i]+nearY) - org.y) * rdir.y;
        const avxf tNearZ = (load8f((const char*)&lower_x[i]+nearZ) - org.z) * rdir.z;
        const avxf tFarX  = (load8f((const char*)&lower_x[i]+farX ) - org.x) * rdir.x;
        const avxf tFarY  = (load8f((const char*)&lower_x[i]+farY ) - org.y) * rdir.y;
        const avxf tFarZ  = (load8f((const char*)&lower_x[i]+farZ ) - org.z) * rdir.z;
#endif

        if (robust) {
          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);
          const avxf tNear = max(tNearX,tNearY,tNearZ,tnear);
          const avxf tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
          const avxb vmask = round_down*tNear <= round_up*tFar;
          const size_t mask = movemask(vmask);
          return mask;
        }
        
/*#if defined(__AVX2__) // FIXME: not working for cube
        const avxf tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
        const avxf tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
        const avxb vmask = cast(tNear) > cast(tFar);
        const size_t mask = movemask(vmask)^0xf;
	#else*/
        const avxf tNear = max(tNearX,tNearY,tNearZ,tnear);
        const avxf tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const avxb vmask = tNear <= tFar;
        const size_t mask = movemask(vmask);
//#endif
        return mask;
      }

      template<bool robust>
      __forceinline size_t intersect(size_t nearX, size_t nearY, size_t nearZ,
                                     const avx3f& org, const avx3f& rdir, const avx3f& org_rdir, const avxf& tnear, const avxf& tfar) const
      {
        const size_t mask01 = intersect<robust>(0, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask23 = intersect<robust>(2, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        return mask01 | (mask23 << 8);
      }
      
#endif

    public:
      ssef lower_x[4];           //!< X dimension of lower bounds of all 4 children.
      ssef upper_x[4];           //!< X dimension of upper bounds of all 4 children.
      ssef lower_y[4];           //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y[4];           //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z[4];           //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z[4];           //!< Z dimension of upper bounds of all 4 children.
    };

    struct QuadList
    {
      struct Quads
      {
	enum struct Type : char { QUAD, QUADQUAD, NONE };
	
	__forceinline Quads () : type(Type::NONE), ofs(0) {}
	__forceinline Quads (Type type, char ofs) : type(type), ofs(ofs) {}
	
	Type type;
	char ofs;
      };

      __forceinline const BBox3fa getQuadBounds(const size_t x0, const size_t y0) const 
      {
	BBox3fa bounds = empty;
	bounds.extend(grid.point(x0+0,y0+0));
	bounds.extend(grid.point(x0+1,y0+0));
	bounds.extend(grid.point(x0+0,y0+1));
	bounds.extend(grid.point(x0+1,y0+1));
	return bounds;
      }

      __forceinline const BBox3fa getQuadQuadBounds(const size_t x0, const size_t y0) const 
      {
	BBox3fa bounds = empty;
	bounds.extend(grid.point(x0+0,y0+0));
	bounds.extend(grid.point(x0+1,y0+0));
	bounds.extend(grid.point(x0+2,y0+0));
	bounds.extend(grid.point(x0+0,y0+1));
	bounds.extend(grid.point(x0+1,y0+1));
	bounds.extend(grid.point(x0+2,y0+1));
	bounds.extend(grid.point(x0+0,y0+2));
	bounds.extend(grid.point(x0+1,y0+2));
	bounds.extend(grid.point(x0+2,y0+2));
	return bounds;
      }

      __forceinline QuadList (const Grid& grid) : grid(grid) {}

      __forceinline const BBox3fa init (size_t x0, size_t x1, size_t y0, size_t y1) 
      {
	BBox3fa box = empty;
	bounds.clear();
	for (size_t y=y0, i=0; y<y1; y+=2)
	{
	  for (size_t x=x0; x<x1; x+=2)
	  {
	    if (x+1 == x1 || y+1 == y1) {
	      for (size_t ly=y; ly<y+2; ly++) {
		for (size_t lx=x; lx<x+2; lx++) { 
		  //new (&quads[i]) Quads(Quads::Type::QUAD,ly*grid.width+lx);
		  new (&quads[i]) Quads(Quads::Type::NONE,ly*grid.width+lx);
		  const BBox3fa b = getQuadBounds(lx,ly);
		  bounds.set(i,b);
		  box.extend(b);
		  i++;
		}
	      }
	    } 
	    else {
	      //new (&quads[i]) Quads(Quads::Type::QUADQUAD,y*grid.width+x);
	      new (&quads[i]) Quads(Quads::Type::QUADQUAD,y*grid.width+x);
	      const BBox3fa b = getQuadQuadBounds(x,y);
	      bounds.set(i,b);
	      box.extend(b);
	      i++;
	    }
	  }
	}
	return box;
      }

      UncompressedBounds16 bounds;
      Quads quads[16];
      const Grid& grid;
    };

  public:
    
    __forceinline Grid(unsigned geomID, unsigned primID)
      : width(0), height(0), geomID(geomID), primID(primID), p(NULL), uv(NULL) {}
    
    __forceinline       Vec3fa& point(const size_t x, const size_t y)       { return p[y*width+x]; }
    __forceinline const Vec3fa& point(const size_t x, const size_t y) const { return p[y*width+x]; }
    __forceinline       Vec2f&  uvs  (const size_t x, const size_t y)       { return uv[y*width+x]; }
    __forceinline const Vec2f&  uvs  (const size_t x, const size_t y) const { return uv[y*width+x]; }

    static size_t getNumQuadLists(size_t width, size_t height) {
      const size_t w = ((width /2)+3)/4;
      const size_t h = ((height/2)+3)/4;
      return w*h;
    }

    template<typename Patch>
    size_t build(Scene* scene, const Patch& patch,
	       FastAllocator& alloc, PrimRef* prims,
	       const int x0, const int x1,
	       const int y0, const int y1,
	       const Vec2f& uv0, const Vec2f& uv1, const Vec2f& uv2, const Vec2f& uv3,
	       const DiscreteTessellationPattern& pattern0, 
	       const DiscreteTessellationPattern& pattern1, 
	       const DiscreteTessellationPattern& pattern2, 
	       const DiscreteTessellationPattern& pattern3, 
	       const DiscreteTessellationPattern& pattern_x,
	       const DiscreteTessellationPattern& pattern_y)
    {
      width  = x1-x0;
      height = y1-y0;
      p = new Vec3fa[width*height]; // FIXME: use allocator
      uv = new Vec2f[width*height];
      Vec2f* luv = (Vec2f*) alloca(width*height*sizeof(Vec2f));

      for (int y=0; y<height; y++) {
        const float fy = pattern_y(y0+y);
        for (int x=0; x<width; x++) {
          const float fx = pattern_x(x0+x);
	  PRINT4(x,y,fx,fy);
          luv[y*width+x] = Vec2f(fx,fy);
        }
      }

      /* evaluate position and uvs */
      BBox3fa bounds = empty;
      for (int y=0; y<height; y++) 
      {
        for (int x=0; x<width; x++) 
	{
	  const Vec3fa p = patch.eval(luv[y*width+x].x,luv[y*width+x].y);
          point(x,y) = p;
	  bounds.extend(p);

	  const Vec2f& uv = luv[y*width+x];
	  const Vec2f uv01 = (1.0f-uv.x) * uv0  + uv.x * uv1;
	  const Vec2f uv32 = (1.0f-uv.x) * uv3  + uv.x * uv2;
	  const Vec2f uvxy = (1.0f-uv.y) * uv01 + uv.y * uv32;
          uvs(x,y) = uvxy;
        }
      }

      /* create lists of quads */
      size_t i=0;
      for (size_t y=0; y<height-1; y+=8) {
	for (size_t x=0; x<width-1; x+=8) {
	  //QuadList* leaf = new (alloc.malloc(sizeof(QuadList))) QuadList(*this);
	  QuadList* leaf = new QuadList(*this); // FIXME: use allocator
	  const BBox3fa bounds = leaf->init(x,min(x+8,width),y,min(y+8,height));
	  prims[i++] = PrimRef(bounds,BVH4::encodeTypedLeaf(leaf,0));
	}
      }
      return i;
    }
    
  public:
    unsigned width;
    unsigned height;
    unsigned primID;
    unsigned geomID;
    Vec3fa* p;
    Vec2f*  uv;
  };

}
