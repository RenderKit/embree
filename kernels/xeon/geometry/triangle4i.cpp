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

#include "triangle4i.h"
#include "common/scene.h"

namespace embree
{
  SceneTriangle4i SceneTriangle4i::type;
  TriangleMeshTriangle4i TriangleMeshTriangle4i::type;

  Triangle4iTy::Triangle4iTy () 
  : PrimitiveType("triangle4i",sizeof(Triangle4i),4,true,1) {} 
  
  size_t Triangle4iTy::blocks(size_t x) const {
    return (x+3)/4;
  }
  
  size_t Triangle4iTy::size(const char* This) const {
    return ((Triangle4i*)This)->size();
  }
  
  void SceneTriangle4i::pack(char* This, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const 
  {
    Scene* scene = (Scene*) geom;
    
    ssei geomID = -1, primID = -1;
    Vec3f* v0[4] = { NULL, NULL, NULL, NULL };
    ssei v1 = zero, v2 = zero;
    PrimRef& prim = *prims;
    
    for (size_t i=0; i<4; i++)
    {
      const TriangleMesh* mesh = scene->getTriangleMesh(prim.geomID());
      const TriangleMesh::Triangle& tri = mesh->triangle(prim.primID());
      if (prims) {
        geomID[i] = prim.geomID();
        primID[i] = prim.primID();
        v0[i] = (Vec3f*) &mesh->vertex(tri.v[0]); 
        v1[i] = (int*)&mesh->vertex(tri.v[1])-(int*)v0[i]; 
        v2[i] = (int*)&mesh->vertex(tri.v[2])-(int*)v0[i]; 
        prims++;
      } else {
        assert(i);
        geomID[i] = -1;
        primID[i] = -1;
        v0[i] = v0[i-1];
        v1[i] = 0; 
        v2[i] = 0;
      }
      if (prims) prim = *prims;
    }
    
    new (This) Triangle4i(v0,v1,v2,geomID,primID);
  }

  BBox3fa SceneTriangle4i::update(char* prim, size_t num, void* geom) const 
  {
    BBox3fa bounds = empty;
    Scene* scene = (Scene*) geom;
    
    for (size_t j=0; j<num; j++) 
    {
      Triangle4i& dst = ((Triangle4i*) prim)[j];
      
      for (size_t i=0; i<4; i++)
      {
        if (dst.primID[i] == -1) break;
        const unsigned geomID = dst.geomID[i];
        const unsigned primID = dst.primID[i];
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa p0 = mesh->vertex(tri.v[0]);
        const Vec3fa p1 = mesh->vertex(tri.v[1]);
        const Vec3fa p2 = mesh->vertex(tri.v[2]);
        bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
      }
    }
    return bounds; 
  }

  void TriangleMeshTriangle4i::pack(char* This, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const 
  {
    TriangleMesh* mesh = (TriangleMesh*) geom;
        
    ssei geomID = -1, primID = -1;
    Vec3f* v0[4] = { NULL, NULL, NULL, NULL };
    ssei v1 = zero, v2 = zero;
    PrimRef& prim = *prims;
    
    for (size_t i=0; i<4; i++)
    {
      const TriangleMesh::Triangle& tri = mesh->triangle(prim.primID());
      if (prims) {
        geomID[i] = prim.geomID();
        primID[i] = prim.primID();
        v0[i] = (Vec3f*) &mesh->vertex(tri.v[0]); 
        v1[i] = (int*)&mesh->vertex(tri.v[1])-(int*)v0[i]; 
        v2[i] = (int*)&mesh->vertex(tri.v[2])-(int*)v0[i]; 
        prims++;
      } else {
        assert(i);
        geomID[i] = -1;
        primID[i] = -1;
        v0[i] = v0[i-1];
        v1[i] = 0; 
        v2[i] = 0;
      }
      if (prims) prim = *prims;
    }
    
    new (This) Triangle4i(v0,v1,v2,geomID,primID);
  }
  
  BBox3fa TriangleMeshTriangle4i::update(char* prim, size_t num, void* geom) const 
  {
    BBox3fa bounds = empty;
    TriangleMesh* mesh = (TriangleMesh*) geom;
    
    for (size_t j=0; j<num; j++) 
    {
      Triangle4i& dst = ((Triangle4i*) prim)[j];
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;

      for (size_t i=0; i<4; i++)
      {
        if (dst.primID[i] == -1) break;
        const unsigned geomID = dst.geomID[i];
        const unsigned primID = dst.primID[i];
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa p0 = mesh->vertex(tri.v[0]);
        const Vec3fa p1 = mesh->vertex(tri.v[1]);
        const Vec3fa p2 = mesh->vertex(tri.v[2]);
        bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
      }
    }
    return bounds; 
  }
}
