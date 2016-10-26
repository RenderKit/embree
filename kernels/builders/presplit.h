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

#include "../common/scene.h"
#include "../common/primref.h"
#include "priminfo.h"
#include "../geometry/bezier1v.h"

#include "../../common/algorithms/parallel_reduce.h"
#include "../../common/algorithms/parallel_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    template<size_t N>
    __forceinline void splitPolygon(const BBox3fa& bounds, 
                                    const size_t dim, 
                                    const float pos, 
                                    const Vec3fa v[N+1],
                                    const Vec3fa inv_length[N],
                                    BBox3fa& left_o, 
                                    BBox3fa& right_o)
    {
      BBox3fa left = empty, right = empty;
      /* clip triangle to left and right box by processing all edges */
      for (size_t i=0; i<N; i++)
      {
        const Vec3fa &v0 = v[i]; 
        const Vec3fa &v1 = v[i+1]; 
        const float v0d = v0[dim];
        const float v1d = v1[dim];
        
        if (v0d <= pos) left. extend(v0); // this point is on left side
        if (v0d >= pos) right.extend(v0); // this point is on right side
        
        if ((v0d < pos && pos < v1d) || (v1d < pos && pos < v0d)) // the edge crosses the splitting location
        {
          assert((v1d-v0d) != 0.0f);
          const Vec3fa c = v0 + (pos-v0d)*inv_length[i][dim]*(v1-v0);
          left.extend(c);
          right.extend(c);
        }
      }
      
      /* clip against current bounds */
      left_o  = intersect(left,bounds);
      right_o = intersect(right,bounds);
    }
    
    template<size_t N>
      __forceinline void splitPolygon(const PrimRef& prim, 
                                      const size_t dim, 
                                      const float pos, 
                                      const Vec3fa v[N+1],
                                      PrimRef& left_o, 
                                      PrimRef& right_o)
    {
      BBox3fa left = empty, right = empty;
      for (size_t i=0; i<N; i++)
      {
        const Vec3fa &v0 = v[i]; 
        const Vec3fa &v1 = v[i+1]; 
        const float v0d = v0[dim];
        const float v1d = v1[dim];
        
        if (v0d <= pos) left. extend(v0); // this point is on left side
        if (v0d >= pos) right.extend(v0); // this point is on right side
        
        if ((v0d < pos && pos < v1d) || (v1d < pos && pos < v0d)) // the edge crosses the splitting location
        {
          assert((v1d-v0d) != 0.0f);
          const float inv_length = 1.0f/(v1d-v0d);
          const Vec3fa c = v0 + (pos-v0d)*inv_length*(v1-v0);
          left.extend(c);
          right.extend(c);
        }
      }
      
      /* clip against current bounds */
      new (&left_o ) PrimRef(intersect(left ,prim.bounds()),prim.geomID(), prim.primID());
      new (&right_o) PrimRef(intersect(right,prim.bounds()),prim.geomID(), prim.primID());
    }
    
    struct TriangleSplitter
    {
      __forceinline TriangleSplitter(const Scene* scene, const PrimRef& prim)
      {
        const TriangleMesh* mesh = (const TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
        TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
        v[0] = mesh->vertex(tri.v[0]);
        v[1] = mesh->vertex(tri.v[1]);
        v[2] = mesh->vertex(tri.v[2]);
        v[3] = mesh->vertex(tri.v[0]);
        inv_length[0] = Vec3fa(1.0f) / (v[1]-v[0]);
        inv_length[1] = Vec3fa(1.0f) / (v[2]-v[1]);
        inv_length[2] = Vec3fa(1.0f) / (v[0]-v[2]);
      }
      
      __forceinline void split(const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const {
        splitPolygon<3>(prim,dim,pos,v,left_o,right_o);
      }
      
      __forceinline void split(const BBox3fa& prim, const size_t dim, const float pos, BBox3fa& left_o, BBox3fa& right_o) const {
        splitPolygon<3>(prim,dim,pos,v,inv_length,left_o,right_o);
      }
      
    private:
      Vec3fa v[4];
      Vec3fa inv_length[3];
    };
    
    struct TriangleSplitterFactory
    {
      __forceinline TriangleSplitterFactory(const Scene* scene)
        : scene(scene) {}
      
      __forceinline TriangleSplitter create(const PrimRef& prim) const {
        return TriangleSplitter(scene,prim);
      }
      
    private:
      const Scene* scene;
    };
    
    struct QuadSplitter
    {
      __forceinline QuadSplitter(const Scene* scene, const PrimRef& prim)
      {
        const QuadMesh* mesh = (const QuadMesh*) scene->get(prim.geomID() & 0x00FFFFFF );  
        QuadMesh::Quad quad = mesh->quad(prim.primID());
        v[0] = mesh->vertex(quad.v[0]);
        v[1] = mesh->vertex(quad.v[1]);
        v[2] = mesh->vertex(quad.v[2]);
        v[3] = mesh->vertex(quad.v[3]);
        v[4] = mesh->vertex(quad.v[0]);
        inv_length[0] = Vec3fa(1.0f) / (v[1]-v[0]);
        inv_length[1] = Vec3fa(1.0f) / (v[2]-v[1]);
        inv_length[2] = Vec3fa(1.0f) / (v[3]-v[2]);
        inv_length[3] = Vec3fa(1.0f) / (v[0]-v[3]);
      }
      
      __forceinline void split(const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const {
        splitPolygon<4>(prim,dim,pos,v,left_o,right_o);
      }
      
      __forceinline void split(const BBox3fa& prim, const size_t dim, const float pos, BBox3fa& left_o, BBox3fa& right_o) const {
        splitPolygon<4>(prim,dim,pos,v,inv_length,left_o,right_o);
      }
      
    private:
      Vec3fa v[5];
      Vec3fa inv_length[4];
    };
    
    struct QuadSplitterFactory
    {
      __forceinline QuadSplitterFactory(const Scene* scene)
        : scene(scene) {}
      
      __forceinline QuadSplitter create(const PrimRef& prim) const {
        return QuadSplitter(scene,prim);
      }
      
    private:
      const Scene* scene;
    };
    
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
      dynamic_large_stack_array(Item,items,N,4096*sizeof(Item));
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
        const int dim = maxDim(sceneBounds.size());
        const float center = 0.5f*(sceneBounds.lower[dim]+sceneBounds.upper[dim]);
        BBox3fa lsceneBounds = sceneBounds; lsceneBounds.upper[dim] = center;
        BBox3fa rsceneBounds = sceneBounds; rsceneBounds.lower[dim] = center;

        /* calculate valid split range for primitive */
        const BBox3fa bounds = item.prim.bounds();
        const float lower = 0.9f*bounds.lower[dim]+0.1f*bounds.upper[dim];
        const float upper = 0.1f*bounds.lower[dim]+0.9f*bounds.upper[dim];

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
      if (A == 0.0f) A = 1.0f;

      /* try to calculate a number of splits per primitive, such that
       * we do not generate more primitives than the size of the prims
       * array */
      ParallelPrefixSumState<size_t> state;
      float f = 1.0f;
      size_t N = 0, iter = 0;
      do {
        f *= 0.7f;
        N = pinfo.size() + parallel_prefix_sum (state, size_t(0), pinfo.size(), size_t(1024), size_t(0), [&] (const range<size_t>& r, const size_t sum) -> size_t
        { 
          size_t N=0;
          for (size_t i=r.begin(); i<r.end(); i++) {
            const float nf = floor(f*prims.size()*area(prims[i])/A);
            size_t n = min(ssize_t(4096), max(ssize_t(1), ssize_t(nf)));
            //if (n<16) n = 1;
            N+=n-1;
          }
          return N;
        },std::plus<size_t>());
        if (iter++ == 5) break; // to avoid infinite loop, e.g. for pinfo.size()==1 and prims.size()==1
      } while (N > prims.size());
      //assert(N <= prims.size());
      assert(N >= pinfo.size());

      /* compute the target number of replications dNtarget */
      size_t dN = N-pinfo.size();
      size_t dNtarget = dN;
      if (N>prims.size()) dNtarget = prims.size()-pinfo.size(); // does never create more than prims.size() primitives
      if (dN == 0) { dN = 1; dNtarget = 1; } // special case to avoid division by zero
      assert(pinfo.size()+dNtarget <= prims.size());

      /* split all primitives */
      parallel_prefix_sum (state, size_t(0), pinfo.size(), size_t(1024), size_t(0), [&] (const range<size_t>& r, size_t ofs) -> size_t
      {
        size_t N = 0;
        for (size_t i=r.begin(); i<r.end(); i++) {
          const float nf = floor(f*prims.size()*area(prims[i])/A);
          size_t n = min(ssize_t(4096), max(ssize_t(1), ssize_t(nf)));
          //if (n<16) n = 1;
          const size_t base0 = (ofs+N+0  )*dNtarget/dN;
          const size_t basen = (ofs+N+n-1)*dNtarget/dN;
          assert(pinfo.size()+basen <= prims.size());
          split_primref(pinfo,split,prims[i],prims.data()+pinfo.size()+base0,basen-base0+1);
          //split_primref(pinfo,split,prims[i],&prims[pinfo.size()+ofs+N],n);
          N+=n-1;
        }
        return N;
      },std::plus<size_t>());

      /* compute new priminfo */
      N = pinfo.size()+dNtarget;
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

    template<>
      inline const PrimInfo presplit<TriangleMesh>(Scene* scene, const PrimInfo& pinfo, mvector<PrimRef>& prims)
    {
      return presplit(pinfo,prims, 
                      [&] (const PrimRef& prim) -> float { 
                        return area(prim.bounds());
                      },
                      [&] (const PrimRef& prim, const int dim, const float pos, PrimRef& lprim, PrimRef& rprim) {
                        TriangleSplitter(scene,prim).split(prim,dim,pos,lprim,rprim);
                      });
    }
  }
}

