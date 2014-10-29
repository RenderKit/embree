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
#include "common/scene_subdivision.h"

#define QUADQUAD4X4_COMPRESS_BOUNDS 0 // FIXME: not working yet in SSE mode

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
        scale = max(Vec3fa(1E-20f),bounds.size()/255.0f);
      }

      /*! Sets bounding box of child. */
      __forceinline void set(size_t j, size_t i, const BBox3fa& bounds) 
      {
        assert(j < 4);
        assert(i < N);
        const Vec3fa lower = clamp(floor((bounds.lower-offset)/scale),Vec3fa(0.0f),Vec3fa(255.0f));
        const Vec3fa upper = clamp(ceil ((bounds.upper-offset)/scale),Vec3fa(0.0f),Vec3fa(255.0f));
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

  public:
      
    __forceinline QuadQuad4x4(unsigned px, unsigned py, unsigned width, unsigned geomID, unsigned primID)
      : px(px), py(py), width(width), geomID(geomID), primID(primID) {}
    
    void displace(Scene* scene)
    {
      SubdivMesh* mesh = (SubdivMesh*) scene->get(geomID);
      if (mesh->displFunc == NULL) return;

      /* calculate uv coordinates */
      __aligned(64) float qu[9][9], qv[9][9];
      __aligned(64) float qx[9][9], qy[9][9], qz[9][9];
      for (size_t y=0; y<9; y++) {
        float fy = float(py+y)/float(width+1);
        for (size_t x=0; x<9; x++) {
          float fx = float(px+x)/float(width+1);
          qu[y][x] = fx;
          qv[y][x] = fy;
          qx[y][x] = v[y][x].x;
          qy[y][x] = v[y][x].y;
          qz[y][x] = v[y][x].z;
        }
      }
      
      /* call displacement shader */
      mesh->displFunc(mesh->userPtr,geomID,primID,(float*)qu,(float*)qv,(float*)qx,(float*)qy,(float*)qz,9*9);

      /* add displacements */
      for (size_t y=0; y<9; y++) {
        for (size_t x=0; x<9; x++) {
          const Vec3fa P0 = v[y][x];
          const Vec3fa P1 = Vec3fa(qx[y][x],qy[y][x],qz[y][x]);
#if defined(DEBUG)
          if (!inside(mesh->displBounds,P1-P0))
            THROW_RUNTIME_ERROR("displacement out of bounds");
#endif
          v[y][x] = P1;
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

    __forceinline Vec3fa& point(const size_t x, const size_t y) { return v[y][x]; }
    
    const BBox3fa build(Scene* scene, const Array2D<Vec3fa>& p2, const Array2D<Vec3fa>& p3, bool Tt, bool Tr, bool Tb, bool Tl)
    {
      for (size_t y=0; y<=8; y++)
        for (size_t x=0; x<=8; x++)
          v[y][x] = p3(x,y);

      if (Tt) {
        for (size_t i=0; i<4; i++) point(2*i,0) = point(2*i+1,0) = p2(i,0);
      }

      if (Tr) {
        for (size_t i=0; i<4; i++) point(8,2*i) = point(8,2*i+1) = p2(4,i);
      }

      if (Tb) {
        for (size_t i=0; i<4; i++) point(2*i,8) = point(2*i+1,8) = p2(i,4);
      }

      if (Tl) {
        for (size_t i=0; i<4; i++) point(0,2*i) = point(0,2*i+1) = p2(0,i);
      }

      displace(scene); // FIXME: stick u/v

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

    const BBox3fa build(Scene* scene, const RegularCatmullClarkPatch& patch)
    {
      for (size_t y=0; y<=8; y++) {
        for (size_t x=0; x<=8; x++) {
          v[y][x] = patch.eval(0.125f*float(x),0.125*float(y));
        }
      }

      displace(scene); // FIXME: stick u/v

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
