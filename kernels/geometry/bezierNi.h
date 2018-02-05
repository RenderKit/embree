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

#if 1
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

    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set, const Vec3fa& offset, const Vec3fa& scale)
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
        const Vec3fa v0 = (mesh->vertex(vtxID+0)-offset)*scale;
        const Vec3fa v1 = (mesh->vertex(vtxID+1)-offset)*scale;
        const Vec3fa v2 = (mesh->vertex(vtxID+2)-offset)*scale;
        const Vec3fa v3 = (mesh->vertex(vtxID+3)-offset)*scale;
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa axis1 = normalize(p3 - p0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axis = axis1;
          bestGeomPrimID = geomprimID;
        }
      }
      return frame(axis);
    }
#endif

#if 0

    const LinearSpace3fa computeAlignedSpace(Scene* scene, const PrimRef* prims, const range<size_t>& set)
    {
      Vec3fa axis(0,0,0);
      
      /*! find curve with minimum ID that defines valid direction */
      for (size_t i=set.begin(); i<set.end(); i++)
      {
        const unsigned int geomID = prims[i].geomID();
        const unsigned int primID = prims[i].primID();
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
        axis += axis1;
      }
      axis = normalize(axis);
      
      return frame(axis).transposed();
    }
    
#endif
    
    /*! fill curve from curve list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t _end, Scene* scene)
    {
#if EMBREE_HAIR_LEAF_MODE == 0
      
      size_t end = min(begin+M,_end);
      N = end-begin;

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        AffineSpace3fa space = computeAlignedSpace(scene,prims,range<size_t>(begin));
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(space,primID);
                
        space.p -= bounds.lower;
        space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),bounds.upper-bounds.lower))*space;

        naabb.l.vx.x[i] = space.l.vx.x;
        naabb.l.vx.y[i] = space.l.vx.y;
        naabb.l.vx.z[i] = space.l.vx.z;

        naabb.l.vy.x[i] = space.l.vy.x;
        naabb.l.vy.y[i] = space.l.vy.y;
        naabb.l.vy.z[i] = space.l.vy.z;

        naabb.l.vz.x[i] = space.l.vz.x;
        naabb.l.vz.y[i] = space.l.vz.y;
        naabb.l.vz.z[i] = space.l.vz.z;

        naabb.p.x[i] = space.p.x;
        naabb.p.y[i] = space.p.y;
        naabb.p.z[i] = space.p.z;

        items.geomID[i] = geomID;
        items.primID[i] = primID;
      }
        
#endif

#if EMBREE_HAIR_LEAF_MODE == 1 

      /* find aligned space */
      size_t end = min(begin+M,_end);
      LinearSpace3fa s = computeAlignedSpace(scene,prims,range<size_t>(begin,end));

      /* calculate leaf gbounds for this space */
      BBox3fa gbounds = empty;
      for (size_t j=begin; j<end; j++) {
        gbounds.extend(scene->get<NativeCurves>(prims[j].geomID())->bounds(s,prims[j].primID()));
      }

      /* normalize space for encoding */
      const Vec3fa bs = gbounds.size();
      AffineSpace3fa a(255.0f*s.vx/bs,255.0f*s.vy/bs,255.0f*s.vz/bs,-255.0f*gbounds.lower/bs);
      space = AffineSpace3fa(a);
      N = end-begin;

      /* verify space */
      /*BBox3fa bounds1 = empty;
      for (size_t j=begin; j<end; j++) {
        BBox3fa b1 = scene->get<NativeCurves>(prims[j].geomID())->bounds(space,prims[j].primID());
        bounds1.extend(b1);
        }*/
        
      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(s,primID);
        const Vec3fa lower = 255.0f*(bounds.lower-gbounds.lower)/gbounds.size();
        const Vec3fa upper = 255.0f*(bounds.upper-gbounds.lower)/gbounds.size();
        
        //if (reduce_min(bounds.lower) < 0.0f || reduce_max(bounds.upper) > 255.0f) PRINT(bounds);
        //PRINT2(i,bounds.size());
        
        items.lower_x[i] = (unsigned char) clamp(floor(lower.x),0.0f,255.0f);
        items.upper_x[i] = (unsigned char) clamp(ceil (upper.x),0.0f,255.0f);
        items.lower_y[i] = (unsigned char) clamp(floor(lower.y),0.0f,255.0f);
        items.upper_y[i] = (unsigned char) clamp(ceil (upper.y),0.0f,255.0f);
        items.lower_z[i] = (unsigned char) clamp(floor(lower.z),0.0f,255.0f);
        items.upper_z[i] = (unsigned char) clamp(ceil (upper.z),0.0f,255.0f);
        items.geomID[i] = geomID;
        items.primID[i] = primID;
      }
#endif

#if EMBREE_HAIR_LEAF_MODE == 2
      
      size_t end = min(begin+M,_end);
      N = end-begin;

      /* encode all primitives */
      BBox3fa bounds = empty;
      for (size_t i=0; i<N; i++)
      {
        const PrimRef& prim = prims[begin+i];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        bounds.extend(scene->get<NativeCurves>(geomID)->bounds(primID));
      }

      /* calculate offset and scale */
      offset = bounds.lower;
      scale = 1.0f/(bounds.size()*sqrt(3.0f));

      /* encode all primitives */
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned int geomID = prim.geomID();
        const unsigned int primID = prim.primID();
        const LinearSpace3fa space2 = computeAlignedSpace(scene,prims,range<size_t>(begin),offset,scale);
        
        const LinearSpace3fa space3(trunc(126.0f*space2.vx),trunc(126.0f*space2.vy),trunc(126.0f*space2.vz));
        const BBox3fa bounds = scene->get<NativeCurves>(geomID)->bounds(offset,scale,max(length(space3.vx),length(space3.vy),length(space3.vz)),space3.transposed(),primID);
        
        bounds_vx.x[i] = (char) space3.vx.x;
        bounds_vx.y[i] = (char) space3.vx.y;
        bounds_vx.z[i] = (char) space3.vx.z;
        bounds_vx.lower[i] = (char) clamp(floor(bounds.lower.x),-126.0f,126.0f);
        bounds_vx.upper[i] = (char) clamp(ceil (bounds.upper.x),-126.0f,126.0f);

        bounds_vy.x[i] = (char) space3.vy.x;
        bounds_vy.y[i] = (char) space3.vy.y;
        bounds_vy.z[i] = (char) space3.vy.z;
        bounds_vy.lower[i] = (char) clamp(floor(bounds.lower.y),-126.0f,126.0f);
        bounds_vy.upper[i] = (char) clamp(ceil (bounds.upper.y),-126.0f,126.0f);

        bounds_vz.x[i] = (char) space3.vz.x;
        bounds_vz.y[i] = (char) space3.vz.y;
        bounds_vz.z[i] = (char) space3.vz.z;
        bounds_vz.lower[i] = (char) clamp(floor(bounds.lower.z),-126.0f,126.0f);
        bounds_vz.upper[i] = (char) clamp(ceil (bounds.upper.z),-126.0f,126.0f);
               
        items.geomID[i] = geomID;
        items.primID[i] = primID;
      }
        
#endif
    }

  public:
#if EMBREE_HAIR_LEAF_MODE == 0

    // 56 bytes
    AffineSpace3vf<M> naabb;
    unsigned int N;
    struct Item {
      unsigned int geomID[M];
      unsigned int primID[M];
    } items;
#endif

#if EMBREE_HAIR_LEAF_MODE == 1
    // 20 bytes 
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
#endif

#if EMBREE_HAIR_LEAF_MODE == 2
    // 26 bytes per primitive
    Vec3fa offset,scale;
    unsigned int N;
    struct Bla {
      unsigned int geomID[M];
      unsigned int primID[M];
    } items;
    struct CompressedOrientedBounds
    {
      char x[M];
      char y[M];
      char z[M];
      char lower[M];
      char upper[M];
    } bounds_vx, bounds_vy, bounds_vz;
#endif
  };
}
