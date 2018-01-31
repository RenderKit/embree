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

#include "primitive.h"
#include "bezier1i.h"

namespace embree
{
  struct BezierNi
  {
    enum { M = 8 };
    
    struct Type : public PrimitiveType {
      Type ();
      size_t size(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximum number of stored primitives */
    static __forceinline size_t max_size() { return M; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+M-1)/M; }

  public:

    /*! Default constructor. */
    __forceinline BezierNi () {}

    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set)
    {
      Vec3fa axis(0,0,1);
      uint64_t bestGeomPrimID = -1;
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=set.begin(); i<set.end(); i++)
      {
        const unsigned int geomID = prims[i].geomID();
        const unsigned int primID = prims[i].primID();
        const uint64_t geomprimID = prims[i].ID64();
        if (geomprimID >= bestGeomPrimID) continue;
        NativeCurves* mesh = (NativeCurves*) scene->get(geomID);
        const unsigned vtxID = mesh->curve(primID);
        const Vec3fa v0 = mesh->vertex(vtxID+0);
        const Vec3fa v1 = mesh->vertex(vtxID+1);
        const Vec3fa v2 = mesh->vertex(vtxID+2);
        const Vec3fa v3 = mesh->vertex(vtxID+3);
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axis = axis1;
          bestGeomPrimID = geomprimID;
        }
      }
      return frame(axis).transposed();
    }
    
    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t _end, Scene* scene)
    {
      /* find aligned space */
      size_t end = min(begin+M,_end);
      LinearSpace3fa s = computeAlignedSpace(scene,prims,range<size_t>(begin,end));

      /* calculate leaf bounds for this space */
      BBox3fa bounds = empty;
      for (size_t j=begin; j<end; j++) {
        bounds.extend(scene->get<NativeCurves>(prims[j].geomID())->bounds(s,prims[j].primID()));
      }

      /* normalize space for encoding */
      const Vec3fa bs = bounds.size();
      AffineSpace3fa a(255.0f*s.vx/bs.x,255.0f*s.vy/bs.y,255.0f*s.vy/bs.y,-255.0f*bounds.lower/bounds.size());
      space = AffineSpace3fa(a);
      N = end-begin;
                             
      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(space,primID);
        items.lower_x[i] = (unsigned char) clamp(floor(bounds.lower.x),0.0f,255.0f);
        items.upper_x[i] = (unsigned char) clamp(ceil (bounds.upper.x),0.0f,255.0f);
        items.lower_y[i] = (unsigned char) clamp(floor(bounds.lower.y),0.0f,255.0f);
        items.upper_y[i] = (unsigned char) clamp(ceil (bounds.upper.y),0.0f,255.0f);
        items.lower_z[i] = (unsigned char) clamp(floor(bounds.lower.z),0.0f,255.0f);
        items.upper_z[i] = (unsigned char) clamp(ceil (bounds.upper.z),0.0f,255.0f);
        items.geomID[i] = geomID;
        items.primID[i] = primID;
      }
    }

  public:
    AffineSpace3fa space;
    unsigned int N;
    struct Item {
      unsigned char lower_x[M];
      unsigned char upper_x[M];
      unsigned char lower_y[M];
      unsigned char upper_y[M];
      unsigned char lower_z[M];
      unsigned char upper_z[M];
      unsigned int geomID[M];
      unsigned int primID[M];
    } items;
  };
}
