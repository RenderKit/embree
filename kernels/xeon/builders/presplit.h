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

#include "../../common/scene.h"
#include "../../common/primref.h"
#include "priminfo.h"
#include "../geometry/bezier1v.h"

#include "../../algorithms/parallel_reduce.h"
#include "../../algorithms/parallel_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    __forceinline void splitTriangle(const PrimRef& prim, int dim, float pos, 
                                     const Vec3fa& a, const Vec3fa& b, const Vec3fa& c, PrimRef& left_o, PrimRef& right_o)
    {
      BBox3fa left = empty, right = empty;
      const Vec3fa v[3] = { a,b,c };
      
      /* clip triangle to left and right box by processing all edges */
      Vec3fa v1 = v[2];
      for (size_t i=0; i<3; i++)
      {
        Vec3fa v0 = v1; v1 = v[i];
        float v0d = v0[dim], v1d = v1[dim];
        
        if (v0d <= pos) left. extend(v0); // this point is on left side
        if (v0d >= pos) right.extend(v0); // this point is on right side
        
        if ((v0d < pos && pos < v1d) || (v1d < pos && pos < v0d)) // the edge crosses the splitting location
        {
          assert((v1d-v0d) != 0.0f);
          Vec3fa c = v0 + (pos-v0d)/(v1d-v0d)*(v1-v0);
          left.extend(c);
          right.extend(c);
        }
      }
      //assert(!left.empty());  // happens if split does not hit triangle
      //assert(!right.empty()); // happens if split does not hit triangle
      
      /* clip against current bounds */
      BBox3fa bounds = prim.bounds();
      BBox3fa cleft (max(left .lower,bounds.lower),min(left .upper,bounds.upper));
      BBox3fa cright(max(right.lower,bounds.lower),min(right.upper,bounds.upper));
      
      new (&left_o ) PrimRef(cleft, prim.geomID(), prim.primID());
      new (&right_o) PrimRef(cright,prim.geomID(), prim.primID());
    }

    template<typename Split>
      inline void split_primref(const PrimInfo& pinfo, Split& split, PrimRef& prim, PrimRef* prims_o, size_t N)
    {
      struct Item {
        __forceinline Item () {}
        __forceinline Item (const BBox3fa& sceneBounds, const PrimRef& prim)
          : sceneBounds(sceneBounds), prim(prim) {}
        BBox3fa sceneBounds;
        PrimRef prim;
      };

      /* put first element onto heap */
      size_t i = 0;
      Item items[4096];
      items[i++] = Item(pinfo.geomBounds,prim);

      /* heap returns item with largest surface area */
      auto order = [] (const Item& a, const Item& b) {
        return halfArea(a.sceneBounds) < halfArea(b.sceneBounds);
      };
      
      while (i < N)
      {
        /* pop item with largest scene bounds */
        std::pop_heap (&items[0],&items[i],order); 
        const Item item = items[--i];

        /* split scene bounds into two halfes along largest dimension */
        const BBox3fa sceneBounds = item.sceneBounds;
        const size_t dim = maxDim(sceneBounds.size());
        const float center = 0.5f*(sceneBounds.lower[dim]+sceneBounds.upper[dim]);
        BBox3fa lsceneBounds = sceneBounds; lsceneBounds.upper[dim] = center;
        BBox3fa rsceneBounds = sceneBounds; rsceneBounds.lower[dim] = center;

        /* calculate valid split range for primitive */
        const BBox3fa bounds = item.prim.bounds();
        const float lower = 0.9f*bounds.lower[dim]+0.1*bounds.upper[dim];
        const float upper = 0.1f*bounds.lower[dim]+0.9*bounds.upper[dim];

        /* if split is on the left side of primitive, continue with right half of scene bounds */
        if (center < lower) {
          items[i++] = Item(rsceneBounds,item.prim);
          std::push_heap (&items[0],&items[i],order);
        }
        
        /* if split is on the right side of primitive, continue with left half of scene bounds */
        else if (center > upper)
        {
          items[i++] = Item(lsceneBounds,item.prim);
          std::push_heap (&items[0],&items[i],order);
        }

        /* if we hit the primitive, split it into two halfes */
        else
        {
          PrimRef lprim,rprim;
          split(item.prim,dim,center,lprim,rprim);
          assert(area(lprim.bounds()) > 0.0);
          assert(area(rprim.bounds()) > 0.0);

          items[i++] = Item(lsceneBounds,lprim);
          std::push_heap (&items[0],&items[i],order);

          items[i++] = Item(rsceneBounds,rprim);
          std::push_heap (&items[0],&items[i],order);
        }
      }

      /* copy split primitive to output array */
      prim = items[0].prim;
      for (size_t i=1; i<N; i++)
        prims_o[i-1] = items[i].prim;
    }
    
    template<typename Area, typename Split>
      inline const PrimInfo presplit(const PrimInfo& pinfo, mvector<PrimRef>& prims, const Area& area, const Split& split)
    {
      /* calculate total surface area */
      float A = 0.0f;
      for (size_t i=0; i<pinfo.size(); i++) // FIXME: parallelize
        A += area(prims[i]);

      /* try to calculate a number of splits per primitive, such that
       * we do not generate more primitives than the size of the prims
       * array */
      ParallelPrefixSumState<size_t> state;
      float f = float(prims.size())/float(pinfo.size())/0.9f;
      size_t N = 0, iter = 0;
      do {
        f *= 0.9f;
        N = pinfo.size() + parallel_prefix_sum (state, size_t(0), pinfo.size(), size_t(1024), size_t(0), [&] (const range<size_t>& r, const size_t sum) -> size_t
        { 
          size_t N=0;
          for (size_t i=r.begin(); i<r.end(); i++) {
            const float nf = ceil(f*pinfo.size()*area(prims[i])/A);
            const size_t n = min(ssize_t(4096), max(ssize_t(1), ssize_t(nf)));
            N+=n-1;
          }
          return N;
        },std::plus<size_t>());
        if (iter++ == 10) // to avoid infinite loop, e.g. for pinfo.size()==1 and prims.size()==1
          return pinfo;

      } while (pinfo.size()+N > prims.size());
      assert(pinfo.size()+N <= prims.size());

      /* split all primitives */
      parallel_prefix_sum (state, size_t(0), pinfo.size(), size_t(1024), size_t(0), [&] (const range<size_t>& r, size_t ofs) -> size_t
      {
        size_t N = 0;
        for (size_t i=r.begin(); i<r.end(); i++) {
          const float nf = ceil(f*pinfo.size()*area(prims[i])/A);
          const size_t n = min(ssize_t(4096), max(ssize_t(1), ssize_t(nf)));
          split_primref(pinfo,split,prims[i],&prims[pinfo.size()+ofs+N],n);
          N+=n-1;
        }
        return N;
      },std::plus<size_t>());

      /* compute new priminfo */
      const PrimInfo pinfo_o = parallel_reduce (size_t(0), size_t(N), PrimInfo(empty), [&] (const range<size_t>& r) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (size_t i=r.begin(); i<r.end(); i++)
          pinfo.add(prims[i].bounds());
        return pinfo;
      },[](const PrimInfo& a, const PrimInfo& b) { return PrimInfo::merge(a,b); });

      return pinfo_o;
    }

    template<typename Mesh>
      inline const PrimInfo presplit(Scene* scene, const PrimInfo& pinfo, mvector<PrimRef>& prims)
    {
      return pinfo;
    }

    __forceinline float triangleArea(const Vec2f& v0, const Vec2f& v1, const Vec2f& v2)
    {
      return abs((v1.x-v0.x)*(v2.y-v0.y)-(v2.x-v0.x)*(v1.y-v0.y));
    }

    template<>
      inline const PrimInfo presplit<TriangleMesh>(Scene* scene, const PrimInfo& pinfo, mvector<PrimRef>& prims)
    {
      return presplit(pinfo,prims, 
                      [&] (const PrimRef& prim) -> float { 
                        const size_t geomID = prim.geomID();
                        const size_t primID = prim.primID();
                        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
                        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
                        const Vec3fa v0 = mesh->vertex(tri.v[0]);
                        const Vec3fa v1 = mesh->vertex(tri.v[1]);
                        const Vec3fa v2 = mesh->vertex(tri.v[2]);
                        const float triAreaX = triangleArea(Vec2f(v0.y,v0.z),Vec2f(v1.y,v1.z),Vec2f(v2.y,v2.z));
                        const float triAreaY = triangleArea(Vec2f(v0.x,v0.z),Vec2f(v1.x,v1.z),Vec2f(v2.x,v2.z));
                        const float triAreaZ = triangleArea(Vec2f(v0.x,v0.y),Vec2f(v1.x,v1.y),Vec2f(v2.x,v2.y));
                        const float triBoxArea = triAreaX+triAreaY+triAreaZ;
                        const float boxArea = area(prim.bounds());
                        //assert(boxArea>=2.0f*triBoxArea);
                        //return max(0.0f,boxArea-0.9f*2.0f*triBoxArea);
                        return boxArea;
                      },
                      [&] (const PrimRef& prim, const int dim, const float pos, PrimRef& lprim, PrimRef& rprim) {
                        const size_t geomID = prim.geomID();
                        const size_t primID = prim.primID();
                        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
                        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
                        const Vec3fa v0 = mesh->vertex(tri.v[0]);
                        const Vec3fa v1 = mesh->vertex(tri.v[1]);
                        const Vec3fa v2 = mesh->vertex(tri.v[2]);
                        splitTriangle(prim,dim,pos,v0,v1,v2,lprim,rprim);
                      });
    }
  }
}

