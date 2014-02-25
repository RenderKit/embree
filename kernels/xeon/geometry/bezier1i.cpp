// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bezier1i.h"
#include "common/scene.h"

namespace embree
{
  SceneBezier1i SceneBezier1i::type;

  Bezier1iType::Bezier1iType () 
    : PrimitiveType("bezier1i",sizeof(Bezier1i),1,true,1) {} 
  
  size_t Bezier1iType::blocks(size_t x) const {
    return x;
  }
    
  size_t Bezier1iType::size(const char* This) const {
    return 1;
  }

  void SceneBezier1i::pack(char* dst, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const 
  {
    Scene* scene = (Scene*) geom;
    const PrimRef& prim = *prims;
    const unsigned geomID = prim.geomID();
#if defined(PRE_SUBDIVISION_HACK)
    const unsigned primID = prim.primID()/8;
#else
    const unsigned primID = prim.primID();
#endif
    const BezierCurves* curves = scene->getBezierCurves(geomID);
    const Vec3fa& p0 = curves->vertex(curves->curve(primID));
    new (dst) Bezier1i(&p0,geomID,primID,curves->mask);
    prims++;
  }
  
  void SceneBezier1i::pack(char* dst, const PrimRef* prims, size_t num, void* geom) const 
  {
    Scene* scene = (Scene*) geom;
    const PrimRef& prim = *prims;
    const unsigned geomID = prim.geomID();
#if defined(PRE_SUBDIVISION_HACK)
    const unsigned primID = prim.primID()/8;
#else
    const unsigned primID = prim.primID();
#endif
    const BezierCurves* curves = scene->getBezierCurves(geomID);
    const int vtx = curves->curve(primID);
    const Vec3fa& p0 = curves->vertex(vtx);
    new (dst) Bezier1i(&p0,geomID,primID,curves->mask);
    prims++;
  }
    
  BBox3fa SceneBezier1i::update(char* prim, size_t num, void* geom) const 
  {
    BBox3fa bounds = empty;
    Scene* scene = (Scene*) geom;
    
    for (size_t j=0; j<num; j++) 
    {
      Bezier1i& dst = ((Bezier1i*) prim)[j];
      const unsigned geomID = dst.geomID;
      const unsigned primID = dst.primID;
      const BezierCurves* curves = scene->getBezierCurves(geomID);
      const int vtx = curves->curve(primID);
      bounds.extend(curves->vertex(vtx+0));
      bounds.extend(curves->vertex(vtx+1));
      bounds.extend(curves->vertex(vtx+2));
      bounds.extend(curves->vertex(vtx+3));
      dst.mask = curves->mask;
    }
    return bounds; 
  }
}
