// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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
#include "../common/scene_quad_mesh.h"

namespace embree
{
  /* Stores the vertices of M quads in struct of array layout */
  struct Quad1v
  { 
  public:
    struct Type : public PrimitiveType 
    {
      const char* name() const;
      size_t sizeActive(const char* This) const;
      size_t sizeTotal(const char* This) const;
      size_t getBytes(const char* This) const;
    };
    static Type type;

  public:

    /* Returns maximum number of stored quads */
    static __forceinline size_t max_size() { return 1; }
    
    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return 1; }
   
  public:

    /* Default constructor */
    __forceinline Quad1v() {}

    /* Construction from vertices and IDs */
    __forceinline Quad1v(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Vec3f& v3, const unsigned int geomID, const unsigned int primID)
      : v0(v0), v2(v2), v1(v1), geomId(geomID), v3(v3), primId(primID) {}
    
    /* Returns a mask that tells which quads are valid */
    __forceinline bool valid() const { return geomId != -1; }

    /* Returns true if the specified quad is valid */
    __forceinline bool valid(const size_t i) const { return geomId != -1; }

    /* Returns the number of stored quads */
    __forceinline size_t size() const { return 1; }

    /* Returns the geometry ID */
    __forceinline       unsigned int& geomID()       { return geomId; }
    __forceinline const unsigned int& geomID() const { return geomId; }
    __forceinline unsigned int geomID(const size_t i) const { return geomId; }

    /* Returns the primitive ID0 */
    __forceinline       unsigned int& primID()       { return primId; }
    __forceinline const unsigned int& primID() const { return primId; }
    __forceinline unsigned int primID(const size_t i) const { return primId; }
    
    /* Calculate the bounds of the quads */
    __forceinline BBox3fa bounds() const 
    {
      Vec3fa lower = (Vec3fa)min(v0,v1,v2,v3);
      Vec3fa upper = (Vec3fa)max(v0,v1,v2,v3);
      return BBox3fa(lower,upper);
    }
    
    /* Fill quad from quad list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene)
    {
      assert(end-begin==1);
      const PrimRef& prim = *prims;
      geomId = prim.geomID();
      primId = prim.primID();
      const QuadMesh* __restrict__ const mesh = scene->get<QuadMesh>(geomId);
      const QuadMesh::Quad& quad = mesh->quad(primId);
      v0 = mesh->vertex(quad.v[0]);
      v1 = mesh->vertex(quad.v[1]);
      v2 = mesh->vertex(quad.v[2]);
      v3 = mesh->vertex(quad.v[3]);      
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(QuadMesh* mesh)
    {
      const QuadMesh::Quad& quad = mesh->quad(primId);
      v0 = mesh->vertex(quad.v[0]);
      v1 = mesh->vertex(quad.v[1]);
      v2 = mesh->vertex(quad.v[2]);
      v3 = mesh->vertex(quad.v[3]);      
      return bounds();
    }

    __forceinline void init(unsigned int geomID,
			    unsigned int primID,
			    const Scene *const scene)
    {
      QuadMesh* mesh = (QuadMesh*)scene->get(geomID);
      const QuadMesh::Quad &q = mesh->quad(primID);
      v0 = mesh->vertex(q.v[0]);
      v1 = mesh->vertex(q.v[1]);
      v2 = mesh->vertex(q.v[2]);
      v3 = mesh->vertex(q.v[3]);
      unused0 = 0;
      unused1 = 0;     
      geomId  = geomID;
      primId  = primID;      
    }
    
  public:
    Vec3f v0;             // 1st vertex of the quad
    unsigned int unused0; // unused
    Vec3f v2;             // 3rd vertex of the quad
    unsigned int unused1; // unused
    Vec3f v1;             // 2nd vertex of the quad
    unsigned int geomId;  // geometry ID    
    Vec3f v3;             // 4nd vertex of the quad    
    unsigned int primId;  // primitive ID
  };

  //typename Quad1v::Type Quad1v::type;

}
