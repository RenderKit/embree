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

#define GRID_COMPRESS_BOUNDS 1

namespace embree
{
  struct Grid
  {
    struct CompressedBounds16;
    struct UncompressedBounds16;

#if GRID_COMPRESS_BOUNDS && defined(__SSE4_1__)
    typedef CompressedBounds16 Bounds16;
#else
    typedef UncompressedBounds16 Bounds16;
#endif

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

      /*! set bounds of all children (only required for compression) */
      __forceinline void set(const BBox3fa& bounds) {
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

#if defined (__SSE4_1__)

    struct CompressedBounds16
    {
      static const size_t N = 4;

      /*! Sets bounding box of child. */
      __forceinline void clear() 
      {
	for (size_t i=0; i<16; i++) {
	  lower_x[i] = lower_y[i] = lower_z[i] = 255;
	  upper_x[i] = upper_y[i] = upper_z[i] = 0;
	}
      }

      __forceinline void set(const BBox3fa& bounds) {
        offset = bounds.lower;
        scale = max(Vec3fa(1E-20f),bounds.size()/255.0f);
      }

      /*! Sets bounding box of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
	assert(i < 4*N);
        const Vec3fa lower = clamp(floor((bounds.lower-offset)/scale),Vec3fa(0.0f),Vec3fa(255.0f));
        const Vec3fa upper = clamp(ceil ((bounds.upper-offset)/scale),Vec3fa(0.0f),Vec3fa(255.0f));
        lower_x[i] = (unsigned char) lower.x;
        lower_y[i] = (unsigned char) lower.y;
        lower_z[i] = (unsigned char) lower.z;
        upper_x[i] = (unsigned char) upper.x;
        upper_y[i] = (unsigned char) upper.y;
        upper_z[i] = (unsigned char) upper.z;
      }
      
      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t i, size_t nearX, size_t nearY, size_t nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;

        const sse3f vscale(scale), voffset(offset);
        const ssef near_x = madd(ssef::load(&this->lower_x[i]+nearX),vscale.x,voffset.x);
        const ssef near_y = madd(ssef::load(&this->lower_x[i]+nearY),vscale.y,voffset.y);
        const ssef near_z = madd(ssef::load(&this->lower_x[i]+nearZ),vscale.z,voffset.z);
        const ssef far_x  = madd(ssef::load(&this->lower_x[i]+farX),vscale.x,voffset.x);
        const ssef far_y  = madd(ssef::load(&this->lower_x[i]+farY),vscale.y,voffset.y);
        const ssef far_z  = madd(ssef::load(&this->lower_x[i]+farZ),vscale.z,voffset.z);

#if defined (__AVX2__)
        const ssef tNearX = msub(near_x, rdir.x, org_rdir.x);
        const ssef tNearY = msub(near_y, rdir.y, org_rdir.y);
        const ssef tNearZ = msub(near_z, rdir.z, org_rdir.z);
        const ssef tFarX  = msub(far_x , rdir.x, org_rdir.x);
        const ssef tFarY  = msub(far_y , rdir.y, org_rdir.y);
        const ssef tFarZ  = msub(far_z , rdir.z, org_rdir.z);
#else
        const ssef tNearX = (near_x - org.x) * rdir.x;
        const ssef tNearY = (near_y - org.y) * rdir.y;
        const ssef tNearZ = (near_z - org.z) * rdir.z;
        const ssef tFarX  = (far_x  - org.x) * rdir.x;
        const ssef tFarY  = (far_y  - org.y) * rdir.y;
        const ssef tFarZ  = (far_z  - org.z) * rdir.z;
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
      __forceinline size_t intersect(size_t i, size_t nearX, size_t nearY, size_t nearZ,
                                     const avx3f& org, const avx3f& rdir, const avx3f& org_rdir, const avxf& tnear, const avxf& tfar) const
      {
        const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;

        const avx3f vscale(scale), voffset(offset);
        const avxf near_x = madd(avxf::load(&this->lower_x[i]+nearX),vscale.x,voffset.x);
        const avxf near_y = madd(avxf::load(&this->lower_x[i]+nearY),vscale.y,voffset.y);
        const avxf near_z = madd(avxf::load(&this->lower_x[i]+nearZ),vscale.z,voffset.z);
        const avxf far_x  = madd(avxf::load(&this->lower_x[i]+farX),vscale.x,voffset.x);
        const avxf far_y  = madd(avxf::load(&this->lower_x[i]+farY),vscale.y,voffset.y);
        const avxf far_z  = madd(avxf::load(&this->lower_x[i]+farZ),vscale.z,voffset.z);

#if defined (__AVX2__)

        const avxf tNearX = msub(near_x, rdir.x, org_rdir.x);
        const avxf tNearY = msub(near_y, rdir.y, org_rdir.y);
        const avxf tNearZ = msub(near_z, rdir.z, org_rdir.z);
        const avxf tFarX  = msub(far_x , rdir.x, org_rdir.x);
        const avxf tFarY  = msub(far_y , rdir.y, org_rdir.y);
        const avxf tFarZ  = msub(far_z , rdir.z, org_rdir.z);
#else

        const avxf tNearX = (near_x - org.x) * rdir.x;
        const avxf tNearY = (near_y - org.y) * rdir.y;
        const avxf tNearZ = (near_z - org.z) * rdir.z;
        const avxf tFarX  = (far_x  - org.x) * rdir.x;
        const avxf tFarY  = (far_y  - org.y) * rdir.y;
        const avxf tFarZ  = (far_z  - org.z) * rdir.z;
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
        const size_t mask23 = intersect<robust>(8, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        return mask01 | (mask23 << 8);
      }
      
#endif

    public:
      Vec3fa offset;               //!< offset to decompress bounds
      Vec3fa scale;                //!< scale  to decompress bounds
      unsigned char lower_x[16]; 
      unsigned char upper_x[16]; 
      unsigned char lower_y[16]; 
      unsigned char upper_y[16]; 
      unsigned char lower_z[16]; 
      unsigned char upper_z[16]; 
    };

#endif

    struct QuadList
    {
      struct Quads
      {
	enum { QUAD2X2 = 0, QUAD1X2 = 1, QUAD2X1 = 2, QUAD1X1 = 3, NONE = 4 };
	
	__forceinline Quads () : type(NONE), ofs(0) {}
	__forceinline Quads (unsigned char type, unsigned char ofs) : type(type), ofs(ofs) {}
	
	unsigned char type;
	unsigned char ofs;
      };

      __forceinline const BBox3fa getBounds(const size_t x0, const size_t x1, const size_t y0, const size_t y1) const 
      {
	BBox3fa bounds = empty;
	for (size_t y=y0; y<=y1; y++)
	  for (size_t x=x0; x<=x1; x++)
	    bounds.extend(grid.point(x,y));
	return bounds;
      }

      __forceinline QuadList (const Grid& grid) : grid(grid) {}

      __forceinline const BBox3fa init (size_t x0, size_t x1, size_t y0, size_t y1) 
      {
	BBox3fa box_list[16];

	BBox3fa box = empty;
	bounds.clear();
	size_t i=0;
	for (size_t y=y0; y<y1; y+=2)
	{
	  for (size_t x=x0; x<x1; x+=2)
	  {
	    const bool right = x+1 == x1;
	    const bool bottom = y+1 == y1;
	    new (&quads[i]) Quads(2*bottom+right,y*grid.width+x);
	    const BBox3fa b = getBounds(x,min(x+2,x1),y,min(y+2,y1));
	    box_list[i++] = b;
	    box.extend(b);
	  }
	}

	bounds.set(box);
	for (size_t j=0; j<i; j++)
	  bounds.set(j,box_list[j]);

	return box;
      }

      Bounds16 bounds;
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
    void displace(Scene* scene, const Patch& patch, const Vec2f* luv)
    {
      SubdivMesh* mesh = (SubdivMesh*) scene->get(geomID);
      if (mesh->displFunc == NULL) return;

      /* calculate uv coordinates */
      __aligned(64) float qu[17*17], qv[17*17];
      __aligned(64) float qx[17*17], qy[17*17], qz[17*17];
      __aligned(64) float nx[17*17], ny[17*17], nz[17*17];
      for (size_t y=0; y<height; y++) 
      {
        for (size_t x=0; x<width; x++) 
        {
          qu[y*width+x] = uv[y*width+x].x;
          qv[y*width+x] = uv[y*width+x].y;
          qx[y*width+x] = p[y*width+x].x;
          qy[y*width+x] = p[y*width+x].y;
          qz[y*width+x] = p[y*width+x].z;
          const Vec3fa N = normalize(patch.normal(luv[y*width+x].x, luv[y*width+x].y));
          nx[y*width+x] = N.x;
          ny[y*width+x] = N.y;
          nz[y*width+x] = N.z;
        }
      }
      
      /* call displacement shader */
      mesh->displFunc(mesh->userPtr,geomID,primID,
                      (float*)qu,(float*)qv,
                      (float*)nx,(float*)ny,(float*)nz,
                      (float*)qx,(float*)qy,(float*)qz,
                      width*height);

      /* add displacements */
      for (size_t y=0; y<height; y++) {
        for (size_t x=0; x<width; x++) {
          const Vec3fa P0 = p[y*width+x];
          const Vec3fa P1 = Vec3fa(qx[y*width+x],qy[y*width+x],qz[y*width+x]);
/*#if defined(DEBUG) // FIXME: enable
          if (!inside(mesh->displBounds,P1-P0))
            THROW_RUNTIME_ERROR("displacement out of bounds");
	    #endif*/
          p[y*width+x] = P1;
        }
      }
    }

    template<typename Patch>
    size_t build(Scene* scene, const Patch& patch,
	       FastAllocator::Thread& alloc, PrimRef* prims,
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
      p = (Vec3fa*) alloc.malloc(width*height*sizeof(Vec3fa));
      uv = (Vec2f*) alloc.malloc(width*height*sizeof(Vec2f));
      Vec2f luv[17*17]; //= (Vec2f*) alloca(width*height*sizeof(Vec2f));

      for (int y=0; y<height; y++) {
        const float fy = pattern_y(y0+y);
        for (int x=0; x<width; x++) {
          const float fx = pattern_x(x0+x);
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

      /* displace points */
      displace(scene,patch,luv);

      /* create lists of quads */
      size_t i=0;
      for (size_t y=0; y<height-1; y+=8) {
	for (size_t x=0; x<width-1; x+=8) {
	  QuadList* leaf = new (alloc.malloc(sizeof(QuadList))) QuadList(*this);
	  const BBox3fa bounds = leaf->init(x,min(x+8,width-1),y,min(y+8,height-1));
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
