// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
#include "../common/geometry.h"

namespace embree
{
  namespace isa
  {
    struct CurvePrecalculations1
    {
      float depth_scale;
      LinearSpace3fa ray_space;
      
      __forceinline CurvePrecalculations1() {}

      __forceinline CurvePrecalculations1(const Ray& ray, const void* ptr)
      {
        depth_scale = rsqrt(dot(ray.dir,ray.dir));
        ray_space = frame(depth_scale*ray.dir);
        ray_space.vz *= depth_scale;
        ray_space = ray_space.transposed();
      }
    };
    
    template<int K>
      struct CurvePrecalculationsK
    {
      vfloat<K> depth_scale;
      LinearSpace3fa ray_space[K];

      __forceinline CurvePrecalculationsK(const vbool<K>& valid, const RayK<K>& ray)
      {
        size_t mask = movemask(valid);
        depth_scale = rsqrt(dot(ray.dir,ray.dir));
        while (mask) {
          size_t k = __bscf(mask);
          LinearSpace3fa ray_space_k = frame(depth_scale[k]*Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]));
          ray_space_k.vz *= depth_scale[k];
          ray_space_k = ray_space_k.transposed();
          ray_space[k] = ray_space_k;
        }
      }

      __forceinline CurvePrecalculationsK(const RayK<K>& ray, size_t k)
      {
        Vec3fa ray_dir = Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
        depth_scale[k] = rsqrt(dot(ray_dir,ray_dir));
        ray_space  [k] = frame(depth_scale[k]*ray_dir).transposed();
      }
    };
  }

  struct VirtualCurvePrimitive
  {
    typedef void (*Intersect1Ty)(void* pre, void* ray, IntersectContext* context, const void* primitive);
    typedef bool (*Occluded1Ty )(void* pre, void* ray, IntersectContext* context, const void* primitive);
    
    typedef void (*Intersect4Ty)(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    typedef bool (*Occluded4Ty) (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    
    typedef void (*Intersect8Ty)(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    typedef bool (*Occluded8Ty) (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    
    typedef void (*Intersect16Ty)(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    typedef bool (*Occluded16Ty) (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
    
  public:
    struct Intersectors
    {
    Intersectors()
    : intersect1(nullptr), occluded1(nullptr),
        intersect4(nullptr), occluded4(nullptr),
        intersect8(nullptr), occluded8(nullptr),
        intersect16(nullptr), occluded16(nullptr) {}
      
      template<int K> void intersect(void* pre, void* ray, IntersectContext* context, const void* primitive);
      template<int K> bool occluded (void* pre, void* ray, IntersectContext* context, const void* primitive);

      template<int K> void intersect(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);
      template<int K> bool occluded (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive);

    public:
      Intersect1Ty intersect1;
      Occluded1Ty  occluded1;
      Intersect4Ty intersect4;
      Occluded4Ty  occluded4;
      Intersect8Ty intersect8;
      Occluded8Ty  occluded8;
      Intersect16Ty intersect16;
      Occluded16Ty  occluded16;
    };
    
    Intersectors vtbl[Geometry::GTY_END];
  };

  template<> __forceinline void VirtualCurvePrimitive::Intersectors::intersect<1>(void* pre, void* ray, IntersectContext* context, const void* primitive) { assert(intersect1); intersect1(pre,ray,context,primitive); }
  template<> __forceinline bool VirtualCurvePrimitive::Intersectors::occluded<1> (void* pre, void* ray, IntersectContext* context, const void* primitive) { assert(occluded1); return occluded1(pre,ray,context,primitive); }
      
  template<> __forceinline void VirtualCurvePrimitive::Intersectors::intersect<4>(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(intersect4); intersect4(pre,ray,k,context,primitive); }
  template<> __forceinline bool VirtualCurvePrimitive::Intersectors::occluded<4> (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(occluded4); return occluded4(pre,ray,k,context,primitive); }
      
#if defined(__AVX__)
  template<> __forceinline void VirtualCurvePrimitive::Intersectors::intersect<8>(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(intersect8); intersect8(pre,ray,k,context,primitive); }
  template<> __forceinline bool VirtualCurvePrimitive::Intersectors::occluded<8> (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(occluded8); return occluded8(pre,ray,k,context,primitive); }
#endif
  
#if defined(__AVX512F__)
  template<> __forceinline void VirtualCurvePrimitive::Intersectors::intersect<16>(void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(intersect16); intersect16(pre,ray,k,context,primitive); }
  template<> __forceinline bool VirtualCurvePrimitive::Intersectors::occluded<16> (void* pre, void* ray, size_t k, IntersectContext* context, const void* primitive) { assert(occluded16); return occluded16(pre,ray,k,context,primitive); }
#endif
}
