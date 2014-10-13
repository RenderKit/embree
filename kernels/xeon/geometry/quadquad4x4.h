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

#define QUADQUAD4X4_COMPRESS_BOUNDS 0

namespace embree
{
  struct QuadQuad4x4
  {
    struct CompressedBounds16;
    struct UncompressedBounds16;

#if QUADQUAD4X4_COMPRESS_BOUNDS
    typedef CompressedBounds16 Bounds16;
#else
    typedef UncompressedBounds16 Bounds16;
#endif

    struct UncompressedBounds16
    {
      static const size_t N = 4;

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

    struct CompressedBounds16
    {
      static const size_t N = 4;

      __forceinline void set(const BBox3fa& bounds) {
        offset = bounds.lower;
        scale = 255.0f/bounds.size(); // FIXME: potential division by zero
      }

      /*! Sets bounding box of child. */
      __forceinline void set(size_t j, size_t i, const BBox3fa& bounds) 
      {
        assert(j < 4);
        assert(i < N);
        const Vec3fa lower = clamp((bounds.lower-offset)*scale,Vec3fa(0.0f),Vec3fa(255.0f));
        const Vec3fa upper = clamp((bounds.upper-offset)*scale,Vec3fa(0.0f),Vec3fa(255.0f));
        lower_x[j*4+i] = (unsigned char) lower.x;
        lower_y[j*4+i] = (unsigned char) lower.y;
        lower_z[j*4+i] = (unsigned char) lower.z;
        upper_x[j*4+i] = (unsigned char) upper.x;
        upper_y[j*4+i] = (unsigned char) upper.y;
        upper_z[j*4+i] = (unsigned char) upper.z;
      }
      
      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t i, size_t nearX, size_t nearY, size_t nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;

        const ssef near_x = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearX)));
        const ssef near_y = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearY)));
        const ssef near_z = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearZ)));
        const ssef far_x  = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farX)));
        const ssef far_y  = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farY)));
        const ssef far_z  = ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farZ)));

#if defined (__AVX2__)
        const ssef tNearX = msub(near_x), rdir.x, org_rdir.x);
        const ssef tNearY = msub(near_y), rdir.y, org_rdir.y);
        const ssef tNearZ = msub(near_z), rdir.z, org_rdir.z);
        const ssef tFarX  = msub(far_x ), rdir.x, org_rdir.x);
        const ssef tFarY  = msub(far_y ), rdir.y, org_rdir.y);
        const ssef tFarZ  = msub(far_z ), rdir.z, org_rdir.z);
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

#if defined (__AVX2__)

        const avxf near_x = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearX)));
        const avxf near_y = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearY)));
        const avxf near_z = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearZ)));
        const avxf far_x  = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farX)));
        const avxf far_y  = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farY)));
        const avxf far_z  = avxf(_mm256_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farZ)));

        const avxf tNearX = msub(near_x, rdir.x, org_rdir.x);
        const avxf tNearY = msub(near_y, rdir.y, org_rdir.y);
        const avxf tNearZ = msub(near_z, rdir.z, org_rdir.z);
        const avxf tFarX  = msub(far_x , rdir.x, org_rdir.x);
        const avxf tFarY  = msub(far_y , rdir.y, org_rdir.y);
        const avxf tFarZ  = msub(far_z , rdir.z, org_rdir.z);
#else

        const avxf near_x = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearX))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+nearX))));
        const avxf near_y = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearY))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+nearY))));
        const avxf near_z = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+nearZ))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+nearZ))));
        const avxf far_x  = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farX ))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+farX ))));
        const avxf far_y  = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farY ))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+farY ))));
        const avxf far_z  = avxf(ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i]+farZ ))), ssef(_mm_cvtepu8_epi32(*(ssei*)(&this->lower_x[i+4]+farZ ))));

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

  public:
      
    __forceinline QuadQuad4x4(unsigned px, unsigned py, unsigned width, unsigned geomID, unsigned primID)
      : px(px), py(py), width(width), geomID(geomID), primID(primID) {}
    
    void displace(Scene* scene)
    {
      SubdivMesh* mesh = (SubdivMesh*) scene->get(geomID);
      if (mesh->displFunc == NULL) return;

      /* calculate uv coordinates */
      Vec2f uv[9][9];
      for (size_t y=0; y<9; y++) {
        float fy = float(py+y)/float(width+1);
        for (size_t x=0; x<9; x++) {
          float fx = float(px+x)/float(width+1);
          uv[y][x] = Vec2f(fx,fy);
        }
      }
      
      /* call displacement shader */
      Vec3fa displ[9][9];
      mesh->displFunc(mesh->userPtr,geomID,primID,(RTCFloat2*)uv,(RTCFloat3a*)displ,9*9);

      /* add displacements */
      for (size_t y=0; y<9; y++) {
        for (size_t x=0; x<9; x++) {
          const Vec3fa dP = displ[y][x];
#if defined(DEBUG)
          if (!inside(mesh->displBounds,dP))
            THROW_RUNTIME_ERROR("displacement out of bounds");
#endif
          v[y][x] += dP;
        }
      }
    }

    const BBox3fa fullBounds() const
    {
      BBox3fa bounds = empty;
      for (size_t i=0; i<9*9; i++) bounds.extend(v[0][i]);
      return bounds;
    }

    const BBox3fa leafBounds(size_t x, size_t y) const
    {
      BBox3fa bounds = empty;
      x *= 2; y *= 2;
      bounds.extend(v[y+0][x+0]);
      bounds.extend(v[y+0][x+1]);
      bounds.extend(v[y+0][x+2]);
      bounds.extend(v[y+1][x+0]);
      bounds.extend(v[y+1][x+1]);
      bounds.extend(v[y+1][x+2]);
      bounds.extend(v[y+2][x+0]);
      bounds.extend(v[y+2][x+1]);
      bounds.extend(v[y+2][x+2]);
      return bounds;
    }
    
    const BBox3fa build()
    {
#if QUADQUAD4X4_COMPRESS_BOUNDS
      n.set(fullBounds());
#endif

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
    Bounds16 n;               //!< bounds of all QuadQuads
    Vec3fa v[9][9];           //!< pointer to vertices
    unsigned px, py;          //!< position inside subdivpatch
    unsigned width;           //!< subdivision width of subdivpatch
    unsigned primID;
    unsigned geomID;
  };
}
