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

#include "common/default.h"
#include "common/scene.h"
#include "builders/primrefblock.h"

#include "common/ray.h"

#if defined(__SSE__)
#include "common/ray4.h"
#endif

#if defined(__AVX__)
#include "common/ray8.h"
#endif

namespace embree
{
  struct PrimitiveType
  {
    /*! constructs the triangle type */
    PrimitiveType (const char* name, size_t bytes, size_t blockSize, bool needVertices, int intCost) 
      : name(name), bytes(bytes), blockSize(blockSize), needVertices(needVertices), intCost(intCost) {}

    /*! Computes the number of blocks required to store a number of triangles. */
    virtual size_t blocks(size_t x) const = 0; // FIXME: are these still required

    /*! Returns the number of stored primitives in a block. */
    virtual size_t size(const char* This) const = 0;

    /*! Updates all primitives stored in a leaf */
    virtual BBox3fa update(char* prim, size_t num, void* geom) const { return BBox3fa(empty); } // FIXME: remove

    /*! Updates all primitives stored in a leaf */
    virtual std::pair<BBox3fa,BBox3fa> update2(char* prim, size_t num, void* geom) const { return std::pair<BBox3fa,BBox3fa>(empty,empty); } // FIXME: remove

  public:
    std::string name;       //!< name of this primitive type
    size_t bytes;           //!< number of bytes of the triangle data
    size_t blockSize;       //!< block size
    bool   needVertices;    //!< determines if we need the vertex array
    int    intCost;         //!< cost of one ray/primitive intersection
  };

  template<typename Intersector>
  struct ListIntersector1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
        Intersector::intersect(pre,ray,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, void* geom) 
    {
      while (true) {
	if (Intersector::occluded(pre,ray,*prim,geom))
	  return true;
	if (prim->last()) break;
	prim++;
      }
      return false;
    }
  };

  template<typename Intersector>
  struct ArrayIntersector1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        Intersector::intersect(pre,ray,prim[i],geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) {
	if (Intersector::occluded(pre,ray,prim[i],geom))
	  return true;
      }
      return false;
    }
  };

#if defined __SSE__

  template<typename Intersector>
  struct ListIntersector4
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
	Intersector::intersect(valid,pre,ray,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline sseb occluded(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom) 
    {
      sseb valid0 = valid;
      while (true) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,geom);
        if (none(valid0)) break;
	if (prim->last()) break;
	prim++;
      }
      return !valid0;
    }
  };

  template<typename Intersector>
  struct ArrayIntersector4
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(valid,pre,ray,prim[i],geom);
      }
    }

    static __forceinline sseb occluded(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom) 
    {
      sseb valid0 = valid;
      for (size_t i=0; i<num; i++) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],geom);
        if (none(valid0)) break;
      }
      return !valid0;
    }
  };


  template<typename Intersector>
  struct ListIntersector4_1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
	Intersector::intersect(valid,pre,ray,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline sseb occluded(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom) 
    {
      sseb valid0 = valid;
      while (true) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,geom);
        if (none(valid0)) break;
	if (prim->last()) break;
	prim++;
      }
      return !valid0;
    }

    static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
        Intersector::intersect(pre,ray,k,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, void* geom) 
    {
      while (true) {
	if (Intersector::occluded(pre,ray,k,*prim,geom))
	  return true;
	if (prim->last()) break;
	prim++;
      }
      return false;
    }
  };

  template<typename Intersector>
  struct ArrayIntersector4_1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(valid,pre,ray,prim[i],geom);
      }
    }

    static __forceinline sseb occluded(const sseb& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, void* geom) 
    {
      sseb valid0 = valid;
      for (size_t i=0; i<num; i++) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],geom);
        if (none(valid0)) break;
      }
      return !valid0;
    }

    static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(pre,ray,k,prim[i],geom);
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) {
	if (Intersector::occluded(pre,ray,k,prim[i],geom))
	  return true;
      }
      return false;
    }
  };

#endif

#if defined __AVX__

  template<typename Intersector>
  struct ListIntersector8
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
	Intersector::intersect(valid,pre,ray,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline avxb occluded(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom) 
    {
      avxb valid0 = valid;
      while (true) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,geom);
        if (none(valid0)) break;
	if (prim->last()) break;
	prim++;
      }
      return !valid0;
    }
  };

  template<typename Intersector>
  struct ArrayIntersector8
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(valid,pre,ray,prim[i],geom);
      }
    }

    static __forceinline avxb occluded(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom) 
    {
      avxb valid0 = valid;
      for (size_t i=0; i<num; i++) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],geom);
        if (none(valid0)) break;
      }
      return !valid0;
    }
  };


  template<typename Intersector>
  struct ListIntersector8_1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
	Intersector::intersect(valid,pre,ray,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline avxb occluded(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom) 
    {
      avxb valid0 = valid;
      while (true) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,geom);
        if (none(valid0)) break;
	if (prim->last()) break;
	prim++;
      }
      return !valid0;
    }

    static __forceinline void intersect(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, void* geom)
    {
      while (true) {
        Intersector::intersect(pre,ray,k,*prim,geom);
	if (prim->last()) break;
	prim++;
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, void* geom) 
    {
      while (true) {
	if (Intersector::occluded(pre,ray,k,*prim,geom))
	  return true;
	if (prim->last()) break;
	prim++;
      }
      return false;
    }
  };

  template<typename Intersector>
  struct ArrayIntersector8_1
  {
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    static __forceinline void intersect(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(valid,pre,ray,prim[i],geom);
      }
    }

    static __forceinline avxb occluded(const avxb& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, void* geom) 
    {
      avxb valid0 = valid;
      for (size_t i=0; i<num; i++) {
	valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],geom);
        if (none(valid0)) break;
      }
      return !valid0;
    }

    static __forceinline void intersect(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
	Intersector::intersect(pre,ray,k,prim[i],geom);
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) {
	if (Intersector::occluded(pre,ray,k,prim[i],geom))
	  return true;
      }
      return false;
    }
  };

#endif
}
