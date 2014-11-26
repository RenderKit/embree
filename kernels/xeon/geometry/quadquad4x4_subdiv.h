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

#include "quadquad4x4.h"

namespace embree
{
  struct QuadQuad4x4AdaptiveSubdivision
  {
    PrimRef* prims_o;
    FastAllocator& alloc;
    Scene* scene;
    const unsigned int geomID;
    const unsigned int primID;
    size_t count;

    QuadQuad4x4AdaptiveSubdivision (PrimRef* prims_o,
                                    FastAllocator& alloc,
                                    Scene* scene, // FIXME: pass displacement shader instead
                                    const SubdivMesh::HalfEdge* h, 
                                    const Vec3fa* vertices, 
                                    const unsigned int geomID, 
                                    const unsigned int primID)
    : prims_o(prims_o), alloc(alloc), scene(scene), geomID(geomID), primID(primID), count(0)
      {
#if 0
        const CatmullClarkPatch patch(h,vertices);
        const bool subdiv0 = !h->hasOpposite() || !h->opposite()->isRegularFace(); h = h->next(); // FIXME: should be false if no neighbour?
        const bool subdiv1 = !h->hasOpposite() || !h->opposite()->isRegularFace(); h = h->next();
        const bool subdiv2 = !h->hasOpposite() || !h->opposite()->isRegularFace(); h = h->next();
        const bool subdiv3 = !h->hasOpposite() || !h->opposite()->isRegularFace(); h = h->next();
        subdivide(patch,5,0.0f,1.0f,0.0f,1.0f,subdiv0,subdiv1,subdiv2,subdiv3);
#else
        const GeneralCatmullClarkPatch patch(h,vertices);
        subdivide(patch,5);
#endif
      }

    __forceinline size_t size() const {
      return count;
    }

    void subdivide(const GeneralCatmullClarkPatch& patch, int depth)
    {
      size_t N;
      CatmullClarkPatch patches[GeneralCatmullClarkPatch::SIZE]; 
      patch.subdivide(patches,N);

      const bool noleaf = depth > 1;
      bool csubdiv[GeneralCatmullClarkPatch::SIZE];
      for (size_t i=0; i<N; i++)
        csubdiv[i] = noleaf && !patches[i].dicable();

      for (size_t i=0; i<N; i++) 
        subdivide(patches[i],depth-1,(float(i)+0.0f)/float(N),(float(i)+1.0f)/float(N),0.0f,1.0f,false,csubdiv[(i+1)%N],csubdiv[(i-1)%N],false);
    }

    void subdivide(const CatmullClarkPatch& patch, int depth,
                   float u0, float u1, float v0, float v1,             // uv range
                   bool Tt, bool Tr, bool Tb, bool Tl)                 // tagged transition edges
    {
      if (unlikely(depth <= 0))
        return tessellate(patch,u0,u1,v0,v1,Tt,Tr,Tb,Tl);

      CatmullClarkPatch patches[4]; 
      patch.subdivide(patches);

      const bool noleaf = depth > 1;
      const bool subdivide0 = noleaf && !patches[0].dicable();
      const bool subdivide1 = noleaf && !patches[1].dicable();
      const bool subdivide2 = noleaf && !patches[2].dicable();
      const bool subdivide3 = noleaf && !patches[3].dicable();

      const float u01 = 0.5f*(u0+u1);
      const float v01 = 0.5f*(v0+v1);

      if (subdivide0) subdivide (patches[0],depth-1, u0,u01,v0,v01, false,false,false,false);
      else            tessellate(patches[0],         u0,u01,v0,v01, false,subdivide1,subdivide3,false);

      if (subdivide1) subdivide (patches[1],depth-1, u01,u1,v0,v01, false,false,false,false); 
      else            tessellate(patches[1],         u01,u1,v0,v01, false,false,subdivide2,subdivide0);
      
      if (subdivide2) subdivide (patches[2],depth-1, u01,u1,v01,v1, false,false,false,false); 
      else            tessellate(patches[2],         u01,u1,v01,v1, subdivide1,false,false,subdivide3);
      
      if (subdivide3) subdivide (patches[3],depth-1, u0,u01,v01,v1, false,false,false,false); 
      else            tessellate(patches[3],         u0,u01,v01,v1, subdivide0,subdivide2,false,false);
    }

    void tessellate(const CatmullClarkPatch& patch, 
                    float u0, float u1, float v0, float v1, 
                    bool Tt, bool Tr, bool Tb, bool Tl)
    {
      GregoryPatch patcheval; 
      patcheval.init(patch);

      const float l0 = patch.level[0];
      const float l1 = patch.level[1];
      const float l2 = patch.level[2];
      const float l3 = patch.level[3];
      const TessellationPattern pattern0(l0,Tt);
      const TessellationPattern pattern1(l1,Tr);
      const TessellationPattern pattern2(l2,Tb);
      const TessellationPattern pattern3(l3,Tl);
      const TessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
      const TessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
      const int nx = pattern_x.size();
      const int ny = pattern_y.size();
      
      //const float du = (u1-u0)*(1.0f/8.0f);
      //const float dv = (v1-v0)*(1.0f/8.0f);
      for (int y=0; y<ny; y+=8) 
      {
        for (int x=0; x<nx; x+=8) 
        {
          count++;
          if (prims_o == NULL) continue;
          QuadQuad4x4* leaf = (QuadQuad4x4*) alloc.malloc(sizeof(QuadQuad4x4),16);
          new (leaf) QuadQuad4x4(geomID,primID);
          const BBox3fa bounds = leaf->build(scene,patcheval,pattern0,pattern1,pattern2,pattern3,pattern_x,x,nx,pattern_y,y,ny,u0,u1,v0,v1);
          *prims_o = PrimRef(bounds,BVH4::encodeTypedLeaf(leaf,0));
          prims_o++;
        }
      }
    }
  };
}

