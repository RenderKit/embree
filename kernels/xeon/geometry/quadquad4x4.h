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

namespace embree
{
  struct QuadQuad4x4
  {
    struct Node16
    {
      static const size_t N = 4;

      /*! Clears the node. */
      __forceinline void clear() {
        for (size_t i=0; i<4; i++) {
          lower_x[i] = lower_y[i] = lower_z[i] = pos_inf; 
          upper_x[i] = upper_y[i] = upper_z[i] = neg_inf;
        }
      }
      
      /*! Sets bounding box of child. */
      __forceinline void set(size_t j, size_t i, const BBox3fa& bounds) 
      {
        assert(j < 4);
        assert(i < N);
        lower_x[j][i] = bounds.lower.x; lower_y[j][i] = bounds.lower.y; lower_z[j][i] = bounds.lower.z;
        upper_x[j][i] = bounds.upper.x; upper_y[j][i] = bounds.upper.y; upper_z[j][i] = bounds.upper.z;
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

  public:
      
    __forceinline QuadQuad4x4(unsigned x, unsigned y, unsigned levels, unsigned geomID, unsigned primID)
      : bx(x), by(y), levels(levels-1), geomID(geomID), primID(primID) {}
    
    const BBox3fa leafBounds(size_t x, size_t y)
    {
      BBox3fa bounds = empty;
      x *= 2; y *= 2;
      bounds.extend(vertices(bx+x+0,by+y+0));
      bounds.extend(vertices(bx+x+1,by+y+0));
      bounds.extend(vertices(bx+x+2,by+y+0));
      bounds.extend(vertices(bx+x+0,by+y+1));
      bounds.extend(vertices(bx+x+1,by+y+1));
      bounds.extend(vertices(bx+x+2,by+y+1));
      bounds.extend(vertices(bx+x+0,by+y+2));
      bounds.extend(vertices(bx+x+1,by+y+2));
      bounds.extend(vertices(bx+x+2,by+y+2));
      return bounds;
    }
    
    const BBox3fa build()
    {
      BBox3fa bounds = empty;
      
      const BBox3fa bounds00_0 = leafBounds(0,0); n.set(0,0,bounds00_0); bounds.extend(bounds00_0);
      const BBox3fa bounds00_1 = leafBounds(1,0); n.set(0,1,bounds00_1); bounds.extend(bounds00_1);
      const BBox3fa bounds00_2 = leafBounds(0,1); n.set(0,2,bounds00_2); bounds.extend(bounds00_2);
      const BBox3fa bounds00_3 = leafBounds(1,1); n.set(0,3,bounds00_3); bounds.extend(bounds00_3);
      
      const BBox3fa bounds10_0 = leafBounds(2,0); n.set(1,0,bounds10_0); bounds.extend(bounds10_0);
      const BBox3fa bounds10_1 = leafBounds(3,0); n.set(1,1,bounds10_1); bounds.extend(bounds10_1);
      const BBox3fa bounds10_2 = leafBounds(2,1); n.set(1,2,bounds10_2); bounds.extend(bounds10_2);
      const BBox3fa bounds10_3 = leafBounds(3,1); n.set(1,3,bounds10_3); bounds.extend(bounds10_3);
      
      const BBox3fa bounds01_0 = leafBounds(0,2); n.set(2,0,bounds01_0); bounds.extend(bounds01_0);
      const BBox3fa bounds01_1 = leafBounds(1,2); n.set(2,1,bounds01_1); bounds.extend(bounds01_1);
      const BBox3fa bounds01_2 = leafBounds(0,3); n.set(2,2,bounds01_2); bounds.extend(bounds01_2);
      const BBox3fa bounds01_3 = leafBounds(1,3); n.set(2,3,bounds01_3); bounds.extend(bounds01_3);
      
      const BBox3fa bounds11_0 = leafBounds(2,2); n.set(3,0,bounds11_0); bounds.extend(bounds11_0);
      const BBox3fa bounds11_1 = leafBounds(3,2); n.set(3,1,bounds11_1); bounds.extend(bounds11_1);
      const BBox3fa bounds11_2 = leafBounds(2,3); n.set(3,2,bounds11_2); bounds.extend(bounds11_2);
      const BBox3fa bounds11_3 = leafBounds(3,3); n.set(3,3,bounds11_3); bounds.extend(bounds11_3);
      
      return bounds;
    }
    
  public:
    unsigned bx,by;            //!< coordinates of subtree
    Array2D<Vec3fa> vertices; //!< pointer to vertices
    unsigned levels;           //!< number of stored levels
    unsigned primID;
    unsigned geomID;
    Node16 n;  //!< child nodes
  };
}
