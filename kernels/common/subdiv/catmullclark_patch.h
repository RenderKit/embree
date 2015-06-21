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

#include "catmullclark_ring.h"

namespace embree
{
  template<typename Vertex, typename Vertex_t = Vertex>
    class __aligned(64) CatmullClarkPatchT
  {
  public:
    array_t<CatmullClark1RingT<Vertex,Vertex_t>,4> ring;

    __forceinline CatmullClarkPatchT () {}

  template<typename Loader>
  __forceinline CatmullClarkPatchT (const SubdivMesh::HalfEdge* first_half_edge, const Loader& loader) {
    init2(first_half_edge,loader);
  }

    __forceinline void init (const SubdivMesh::HalfEdge* first_half_edge, const BufferT<Vec3fa>& vertices) 
    {
      init2(first_half_edge,
            [&](const SubdivMesh::HalfEdge* p) { return Vec3fa(vertices[p->getStartVertexIndex()], p->getStartVertexIndex()); });
    }
    
    template<typename Loader>
      __forceinline void init2 (const SubdivMesh::HalfEdge* first_half_edge, const Loader& load) 
    {
      for (size_t i=0; i<4; i++)
        ring[i].init(first_half_edge+i,load);

      checkPositions();
    }

    __forceinline Vertex normal(const float uu, const float vv) const // FIXME: remove
    {
      const int iu = (int) uu;
      const int iv = (int) vv;
      const int index = iv > 0 ? 3-iu : iu;
      const Vertex tu = ring[index].getLimitTangent();
      const Vertex tv = ring[index].getSecondLimitTangent();
      return cross(tv,tu);
    }   

    __forceinline Vertex eval(const float uu, const float vv) const
    {
      const int iu = (int) uu;
      const int iv = (int) vv;
      const int index = iv > 0 ? 3-iu : iu;
      return ring[index].getLimitVertex();
    }   

    __forceinline Vertex getLimitVertex(const size_t index) const {
      return ring[index].getLimitVertex();
    }

    __forceinline Vertex getLimitTangent(const size_t index) const {
      return ring[index].getLimitTangent();
    }

    __forceinline Vertex getSecondLimitTangent(const size_t index) const {
      return ring[index].getSecondLimitTangent();
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa bounds (ring[0].bounds());
      for (size_t i=1; i<4; i++)
	bounds.extend(ring[i].bounds());
      return bounds;
    }

    /*! returns true if the patch is a B-spline patch */
    __forceinline bool isRegular1() const {
      return ring[0].isRegular1() && ring[1].isRegular1() && ring[2].isRegular1() && ring[3].isRegular1();
    }
  __forceinline bool isRegular2() const {
      return ring[0].isRegular2() && ring[1].isRegular2() && ring[2].isRegular2() && ring[3].isRegular2();
    }

    __forceinline bool hasBorder() const {
      return ring[0].hasBorder() || ring[1].hasBorder() || ring[2].hasBorder() || ring[3].hasBorder();
    }

    /*! returns true if the patch is a B-spline patch or final Quad */
    __forceinline bool isRegularOrFinal(const size_t depth) const {
      return ring[0].isRegularOrFinal(depth) && ring[1].isRegularOrFinal(depth) && ring[2].isRegularOrFinal(depth) && ring[3].isRegularOrFinal(depth);
    }

    /*! returns true of the patch is a Gregory patch */
    __forceinline bool isGregoryOrFinal(const size_t depth) const {
      const bool ring0 = ring[0].isGregoryOrFinal(depth);
      const bool ring1 = ring[1].isGregoryOrFinal(depth);
      const bool ring2 = ring[2].isGregoryOrFinal(depth);
      const bool ring3 = ring[3].isGregoryOrFinal(depth);
      return ring0 && ring1 && ring2 && ring3;
    }

    /*! returns true of the patch is a Gregory patch */
    __forceinline bool isGregory() const {
      const bool ring0 = ring[0].isGregory();
      const bool ring1 = ring[1].isGregory();
      const bool ring2 = ring[2].isGregory();
      const bool ring3 = ring[3].isGregory();
      return ring0 && ring1 && ring2 && ring3;
    }

    static __forceinline void init_regular(const CatmullClark1RingT<Vertex,Vertex_t>& p0,
					   const CatmullClark1RingT<Vertex,Vertex_t>& p1,
					   CatmullClark1RingT<Vertex,Vertex_t>& dest0,
					   CatmullClark1RingT<Vertex,Vertex_t>& dest1) 
    {
      assert(p1.face_valence > 2);
      dest1.vertex_level = dest0.vertex_level = p0.edge_level;
      dest1.face_valence = dest0.face_valence = 4;
      dest1.edge_valence = dest0.edge_valence = 8;
      dest1.border_index = dest0.border_index = -1;
      dest1.vtx = dest0.vtx = (Vertex_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;
      dest1.noForcedSubdivision = dest0.noForcedSubdivision = true;

      dest1.ring[2] = dest0.ring[0] = (Vertex_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[7] = (Vertex_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[6] = (Vertex_t)p1.vtx;
      dest1.ring[7] = dest0.ring[5] = (Vertex_t)p1.ring[4];
      dest1.ring[6] = dest0.ring[4] = (Vertex_t)p0.ring[p0.edge_valence-1];
      dest1.ring[5] = dest0.ring[3] = (Vertex_t)p0.ring[p0.edge_valence-2];
      dest1.ring[4] = dest0.ring[2] = (Vertex_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vertex_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[3] = p1.crease_weight[1];
      dest1.crease_weight[3] = dest0.crease_weight[2] = 0.0f;
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];

      //////////////////////////////
      if (p0.eval_unique_identifier <= p1.eval_unique_identifier)
        {
          dest0.eval_start_index = 3;
          dest1.eval_start_index = 0;
          dest0.eval_unique_identifier = p0.eval_unique_identifier;
          dest1.eval_unique_identifier = p0.eval_unique_identifier;
        }
      else
        {
          dest0.eval_start_index = 1;
          dest1.eval_start_index = 2;
          dest0.eval_unique_identifier = p1.eval_unique_identifier;
          dest1.eval_unique_identifier = p1.eval_unique_identifier;
        }
      //////////////////////////////
    }


    static __forceinline void init_border(const CatmullClark1RingT<Vertex,Vertex_t> &p0,
                                          const CatmullClark1RingT<Vertex,Vertex_t> &p1,
                                          CatmullClark1RingT<Vertex,Vertex_t> &dest0,
                                          CatmullClark1RingT<Vertex,Vertex_t> &dest1) 
    {
      dest1.vertex_level = dest0.vertex_level = p0.edge_level;
      dest1.face_valence = dest0.face_valence = 3;
      dest1.edge_valence = dest0.edge_valence = 6;
      dest0.border_index = 2;
      dest1.border_index = 4;
      dest1.vtx  = dest0.vtx = (Vertex_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;
      dest1.noForcedSubdivision = dest0.noForcedSubdivision = true;

      dest1.ring[2] = dest0.ring[0] = (Vertex_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[5] = (Vertex_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[4] = (Vertex_t)p1.vtx;
      dest1.ring[5] = dest0.ring[3] = (Vertex_t)p0.ring[p0.border_index+1]; // dummy
      dest1.ring[4] = dest0.ring[2] = (Vertex_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vertex_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[2] = p1.crease_weight[1];
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];

      //////////////////////////////
      if (p0.eval_unique_identifier <= p1.eval_unique_identifier)
        {
          dest0.eval_start_index = 1;
          dest1.eval_start_index = 2;
          dest0.eval_unique_identifier = p0.eval_unique_identifier;
          dest1.eval_unique_identifier = p0.eval_unique_identifier;
        }
      else
        {
          dest0.eval_start_index = 2;
          dest1.eval_start_index = 0;
          dest0.eval_unique_identifier = p1.eval_unique_identifier;
          dest1.eval_unique_identifier = p1.eval_unique_identifier;
        }
      //////////////////////////////
    }

    static __forceinline void init_regular(const Vertex_t &center, const Vertex_t center_ring[8], const size_t offset, CatmullClark1RingT<Vertex,Vertex_t> &dest)
    {
      dest.vertex_level = 0.0f;
      dest.face_valence = 4;
      dest.edge_valence = 8;
      dest.border_index = -1;
      dest.vtx     = (Vertex_t)center;
      dest.vertex_crease_weight = 0.0f;
      dest.noForcedSubdivision = true;
      for (size_t i=0; i<8; i++) 
	dest.ring[i] = (Vertex_t)center_ring[(offset+i)%8];
      for (size_t i=0; i<4; i++) 
        dest.crease_weight[i] = 0.0f;

      //////////////////////////////
      dest.eval_start_index       = (8-offset)>>1;
      if (dest.eval_start_index >= dest.face_valence) dest.eval_start_index -= dest.face_valence;
      assert( dest.eval_start_index < dest.face_valence );
      dest.eval_unique_identifier = 0;
      //////////////////////////////
    }

    __noinline void subdivide(array_t<CatmullClarkPatchT,4>& patch) const
    {
      ring[0].subdivide(patch[0].ring[0]);
      ring[1].subdivide(patch[1].ring[1]);
      ring[2].subdivide(patch[2].ring[2]);
      ring[3].subdivide(patch[3].ring[3]);

      patch[0].ring[0].edge_level = 0.5f*ring[0].edge_level;
      patch[0].ring[1].edge_level = 0.25f*(ring[1].edge_level+ring[3].edge_level);
      patch[0].ring[2].edge_level = 0.25f*(ring[0].edge_level+ring[2].edge_level);
      patch[0].ring[3].edge_level = 0.5f*ring[3].edge_level;

      patch[1].ring[0].edge_level = 0.5f*ring[0].edge_level;
      patch[1].ring[1].edge_level = 0.5f*ring[1].edge_level;
      patch[1].ring[2].edge_level = 0.25f*(ring[0].edge_level+ring[2].edge_level);
      patch[1].ring[3].edge_level = 0.25f*(ring[1].edge_level+ring[3].edge_level);

      patch[2].ring[0].edge_level = 0.25f*(ring[0].edge_level+ring[2].edge_level);
      patch[2].ring[1].edge_level = 0.5f*ring[1].edge_level;
      patch[2].ring[2].edge_level = 0.5f*ring[2].edge_level;
      patch[2].ring[3].edge_level = 0.25f*(ring[1].edge_level+ring[3].edge_level);

      patch[3].ring[0].edge_level = 0.25f*(ring[0].edge_level+ring[2].edge_level);
      patch[3].ring[1].edge_level = 0.25f*(ring[1].edge_level+ring[3].edge_level);
      patch[3].ring[2].edge_level = 0.5f*ring[2].edge_level;
      patch[3].ring[3].edge_level = 0.5f*ring[3].edge_level;

      const bool regular0 = ring[0].has_last_face() && ring[1].face_valence > 2;
      if (likely(regular0))
        init_regular(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);
      else
        init_border(patch[0].ring[0],patch[1].ring[1],patch[0].ring[1],patch[1].ring[0]);

      const bool regular1 = ring[1].has_last_face() && ring[2].face_valence > 2;
      if (likely(regular1))
        init_regular(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);
      else
        init_border(patch[1].ring[1],patch[2].ring[2],patch[1].ring[2],patch[2].ring[1]);

      const bool regular2 = ring[2].has_last_face() && ring[3].face_valence > 2;
      if (likely(regular2))
        init_regular(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);
      else
        init_border(patch[2].ring[2],patch[3].ring[3],patch[2].ring[3],patch[3].ring[2]);

      const bool regular3 = ring[3].has_last_face() && ring[0].face_valence > 2;
      if (likely(regular3))
        init_regular(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);
      else
        init_border(patch[3].ring[3],patch[0].ring[0],patch[3].ring[0],patch[0].ring[3]);

      Vertex_t center = (ring[0].vtx + ring[1].vtx + ring[2].vtx + ring[3].vtx) * 0.25f;
      Vertex_t center_ring[8];

      // counter-clockwise
      center_ring[0] = (Vertex_t)patch[3].ring[3].ring[0];
      center_ring[7] = (Vertex_t)patch[3].ring[3].vtx;
      center_ring[6] = (Vertex_t)patch[2].ring[2].ring[0];
      center_ring[5] = (Vertex_t)patch[2].ring[2].vtx;
      center_ring[4] = (Vertex_t)patch[1].ring[1].ring[0];
      center_ring[3] = (Vertex_t)patch[1].ring[1].vtx;
      center_ring[2] = (Vertex_t)patch[0].ring[0].ring[0];
      center_ring[1] = (Vertex_t)patch[0].ring[0].vtx;

      init_regular(center,center_ring,0,patch[0].ring[2]);
      init_regular(center,center_ring,2,patch[1].ring[3]);
      init_regular(center,center_ring,4,patch[2].ring[0]);
      init_regular(center,center_ring,6,patch[3].ring[1]);

#if 0
      PRINT("CHECK_RINGS");
      std::cout.precision(10);

      /* outer rings */
      if (!equalRingEval(patch[0].ring[1],patch[1].ring[0]))
        {
          PRINT(patch[0].ring[1]);
          PRINT(patch[1].ring[0]);          
          THROW_RUNTIME_ERROR("equalRingEval(patch[0].ring[1],patch[1].ring[0])");
        }

      if (!equalRingEval(patch[1].ring[2],patch[2].ring[1]))
        {
          PRINT(patch[1].ring[2]);
          PRINT(patch[2].ring[1]);          
          THROW_RUNTIME_ERROR("equalRingEval(patch[1].ring[2],patch[2].ring[1])");
        }

      if (!equalRingEval(patch[2].ring[3],patch[3].ring[2]))
        {
          PRINT(patch[2].ring[3]);
          PRINT(patch[3].ring[2]);          
          THROW_RUNTIME_ERROR("equalRingEval(patch[2].ring[3],patch[3].ring[2])");
        }
      
      if (!equalRingEval(patch[3].ring[0],patch[0].ring[3]))
        {
          PRINT(patch[3].ring[0]);
          PRINT(patch[0].ring[3]);          
          THROW_RUNTIME_ERROR("equalRingEval(patch[3].ring[0],patch[0].ring[3])");
        }

      /* inner rings */
      
      if (!equalRingEval(patch[0].ring[2],patch[1].ring[3])) 
        { 
          PRINT(patch[0].ring[2]); 
          PRINT(patch[1].ring[3]);           
          THROW_RUNTIME_ERROR("equalRingEval(patch[0].ring[2],patch[1].ring[3])"); 
        } 

      if (!equalRingEval(patch[1].ring[3],patch[2].ring[0])) 
        { 
          PRINT(patch[1].ring[3]); 
          PRINT(patch[2].ring[0]);           
          THROW_RUNTIME_ERROR("equalRingEval(patch[1].ring[3],patch[2].ring[0])"); 
        } 

      if (!equalRingEval(patch[2].ring[0],patch[3].ring[1])) 
        { 
          PRINT(patch[2].ring[0]); 
          PRINT(patch[3].ring[1]);           
          THROW_RUNTIME_ERROR("equalRingEval(patch[2].ring[0],patch[3].ring[1])"); 
        } 

      if (!equalRingEval(patch[3].ring[1],patch[0].ring[2])) 
        { 
          PRINT(patch[3].ring[1]); 
          PRINT(patch[0].ring[2]);           
          THROW_RUNTIME_ERROR("equalRingEval(patch[3].ring[1],patch[0].ring[2])"); 
        }       
#endif

#if _DEBUG
      patch[0].checkPositions();
      patch[1].checkPositions();
      patch[2].checkPositions();
      patch[3].checkPositions();
#endif
    }

    bool checkPositions() const
    {
      if( ring[0].hasValidPositions() &&
	  ring[1].hasValidPositions() &&
	  ring[2].hasValidPositions() &&
	  ring[3].hasValidPositions() ) return true;
      return false;
    }

    __forceinline void init( FinalQuad& quad ) const
    {
      quad.vtx[0] = (Vertex_t)ring[0].vtx;
      quad.vtx[1] = (Vertex_t)ring[1].vtx;
      quad.vtx[2] = (Vertex_t)ring[2].vtx;
      quad.vtx[3] = (Vertex_t)ring[3].vtx;
    };

    friend __forceinline std::ostream &operator<<(std::ostream &o, const CatmullClarkPatchT &p)
    {
      o << "CatmullClarkPatch { " << std::endl;
      for (size_t i=0; i<4; i++)
	o << "ring" << i << ": " << p.ring[i] << std::endl;
      o << "}" << std::endl;
      return o;
    }
  };

  typedef CatmullClarkPatchT<Vec3fa,Vec3fa_t> CatmullClarkPatch3fa;

  template<typename Vertex, typename Vertex_t = Vertex>
    class __aligned(64) GeneralCatmullClarkPatchT
  {
  public:
    typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
    static const size_t SIZE = SubdivMesh::MAX_VALENCE;
    array_t<GeneralCatmullClark1RingT<Vertex,Vertex_t>,SIZE> ring;
    size_t N;

    __forceinline GeneralCatmullClarkPatchT () 
      : N(0) {}

  template<typename Loader>
  __forceinline GeneralCatmullClarkPatchT (const SubdivMesh::HalfEdge* first_half_edge, const Loader& loader) {
    init2(first_half_edge,loader);
  }

    __forceinline size_t size() const { 
      return N; 
    }

    __forceinline bool isQuadPatch() const {
      return (N == 4) && ring[0].only_quads && ring[1].only_quads && ring[2].only_quads && ring[3].only_quads;
    }

    __forceinline bool isRegular() const 
    {
      return (N == 4) && ring[0].isRegular() && ring[1].isRegular() && ring[2].isRegular() && ring[3].isRegular();
    }


    __forceinline void init (const SubdivMesh::HalfEdge* first_half_edge, const BufferT<Vec3fa>& vertices) 
    {
      init2(first_half_edge,
            [&](const SubdivMesh::HalfEdge* p) { return Vec3fa(vertices[p->getStartVertexIndex()], p->getStartVertexIndex()); });
    }

    template<typename Loader>
      __forceinline void init2 (const SubdivMesh::HalfEdge* h, const Loader& load) 
    {
      size_t i = 0;
      const SubdivMesh::HalfEdge* edge = h; 
      do {
	ring[i].init(edge,load);
        edge = edge->next();
        i++;
      } while ((edge != h) && (i < SIZE));
      N = i;
    }

    static __forceinline void init_regular(const CatmullClark1RingT<Vertex,Vertex_t>& p0,
					   const CatmullClark1RingT<Vertex,Vertex_t>& p1,
					   CatmullClark1RingT<Vertex,Vertex_t>& dest0,
					   CatmullClark1RingT<Vertex,Vertex_t>& dest1) 
    {
      assert(p1.face_valence > 2);
      dest1.vertex_level = dest0.vertex_level = p0.edge_level;
      dest1.face_valence = dest0.face_valence = 4;
      dest1.edge_valence = dest0.edge_valence = 8;
      dest1.border_index = dest0.border_index = -1;
      dest1.vtx = dest0.vtx = (Vertex_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;
      dest1.noForcedSubdivision = dest0.noForcedSubdivision = true;

      dest1.ring[2] = dest0.ring[0] = (Vertex_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[7] = (Vertex_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[6] = (Vertex_t)p1.vtx;
      dest1.ring[7] = dest0.ring[5] = (Vertex_t)p1.ring[4];
      dest1.ring[6] = dest0.ring[4] = (Vertex_t)p0.ring[p0.edge_valence-1];
      dest1.ring[5] = dest0.ring[3] = (Vertex_t)p0.ring[p0.edge_valence-2];
      dest1.ring[4] = dest0.ring[2] = (Vertex_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vertex_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[3] = p1.crease_weight[1];
      dest1.crease_weight[3] = dest0.crease_weight[2] = 0.0f;
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];

      //////////////////////////////
      if (p0.eval_unique_identifier <= p1.eval_unique_identifier)
        {
          dest0.eval_start_index = 3;
          dest1.eval_start_index = 0;
          dest0.eval_unique_identifier = p0.eval_unique_identifier;
          dest1.eval_unique_identifier = p0.eval_unique_identifier;
        }
      else
        {
          dest0.eval_start_index = 1;
          dest1.eval_start_index = 2;
          dest0.eval_unique_identifier = p1.eval_unique_identifier;
          dest1.eval_unique_identifier = p1.eval_unique_identifier;
        }
      //////////////////////////////
      
    }


    static __forceinline void init_border(const CatmullClark1RingT<Vertex,Vertex_t> &p0,
                                          const CatmullClark1RingT<Vertex,Vertex_t> &p1,
                                          CatmullClark1RingT<Vertex,Vertex_t> &dest0,
                                          CatmullClark1RingT<Vertex,Vertex_t> &dest1) 
    {
      dest1.vertex_level = dest0.vertex_level = p0.edge_level;
      dest1.face_valence = dest0.face_valence = 3;
      dest1.edge_valence = dest0.edge_valence = 6;
      dest0.border_index = 2;
      dest1.border_index = 4;
      dest1.vtx  = dest0.vtx = (Vertex_t)p0.ring[0];
      dest1.vertex_crease_weight = dest0.vertex_crease_weight = 0.0f;
      dest1.noForcedSubdivision = dest0.noForcedSubdivision = true;

      dest1.ring[2] = dest0.ring[0] = (Vertex_t)p0.ring[1];
      dest1.ring[1] = dest0.ring[5] = (Vertex_t)p1.ring[0];
      dest1.ring[0] = dest0.ring[4] = (Vertex_t)p1.vtx;
      dest1.ring[5] = dest0.ring[3] = (Vertex_t)p0.ring[p0.border_index+1]; // dummy
      dest1.ring[4] = dest0.ring[2] = (Vertex_t)p0.vtx;
      dest1.ring[3] = dest0.ring[1] = (Vertex_t)p0.ring[2];

      dest1.crease_weight[1] = dest0.crease_weight[0] = 0.0f;
      dest1.crease_weight[0] = dest0.crease_weight[2] = p1.crease_weight[1];
      dest1.crease_weight[2] = dest0.crease_weight[1] = p0.crease_weight[0];

      //////////////////////////////
      if (p0.eval_unique_identifier <= p1.eval_unique_identifier)
        {
          dest0.eval_start_index = 1;
          dest1.eval_start_index = 2;
          dest0.eval_unique_identifier = p0.eval_unique_identifier;
          dest1.eval_unique_identifier = p0.eval_unique_identifier;
        }
      else
        {
          dest0.eval_start_index = 2;
          dest1.eval_start_index = 0;
          dest0.eval_unique_identifier = p1.eval_unique_identifier;
          dest1.eval_unique_identifier = p1.eval_unique_identifier;
        }
      //////////////////////////////      
    }

    static __forceinline void init_regular(const Vertex_t &center, const array_t<Vertex_t,2*SIZE>& center_ring, const float vertex_level, const size_t N, const size_t offset, CatmullClark1RingT<Vertex,Vertex_t> &dest)
    {
      assert(N<(CatmullClark1RingT<Vertex,Vertex_t>::MAX_FACE_VALENCE));
      assert(2*N<(CatmullClark1RingT<Vertex,Vertex_t>::MAX_EDGE_VALENCE));
      dest.vertex_level = vertex_level;
      dest.face_valence = N;
      dest.edge_valence = 2*N;
      dest.border_index = -1;
      dest.vtx     = (Vertex_t)center;
      dest.vertex_crease_weight = 0.0f;
      dest.noForcedSubdivision = true;
      for (size_t i=0; i<2*N; i++) {
        dest.ring[i] = (Vertex_t)center_ring[(2*N+offset+i-1)%(2*N)];
        assert(isvalid(dest.ring[i]));
      }
      for (size_t i=0; i<N; i++) 
        dest.crease_weight[i] = 0.0f;

      //////////////////////////////
      assert(offset <= 2*N);
      dest.eval_start_index       = (2*N-offset)>>1;
      if (dest.eval_start_index >= dest.face_valence) dest.eval_start_index -= dest.face_valence;

      assert( dest.eval_start_index < dest.face_valence );
      dest.eval_unique_identifier = 0;
      //////////////////////////////
    }
 
    __noinline void subdivide(array_t<CatmullClarkPatch,SIZE>& patch, size_t& N_o) const
    {
      N_o = N;
      assert( N );
      for (size_t i=0; i<N; i++) {
        size_t ip1 = (i+1)%N; // FIXME: %
        ring[i].subdivide(patch[i].ring[0]);
        patch[i]  .ring[0].edge_level = 0.5f*ring[i].edge_level;
        patch[ip1].ring[3].edge_level = 0.5f*ring[i].edge_level;

	assert( patch[i].ring[0].hasValidPositions() );

      }
      assert(N < 2*SIZE);
      Vertex_t center = Vertex_t(0.0f);
      array_t<Vertex_t,2*SIZE> center_ring;
      float center_vertex_level = 2.0f; // guarantees that irregular vertices get always isolated also for non-quads

      for (size_t i=0; i<N; i++)
      {
        size_t ip1 = (i+1)%N; // FIXME: %
        size_t im1 = (i+N-1)%N; // FIXME: %
        bool regular = ring[i].has_last_face() && ring[ip1].face_valence > 2;
        if (likely(regular)) init_regular(patch[i].ring[0],patch[ip1].ring[0],patch[i].ring[1],patch[ip1].ring[3]); 
        else                 init_border (patch[i].ring[0],patch[ip1].ring[0],patch[i].ring[1],patch[ip1].ring[3]);

	assert( patch[i].ring[1].hasValidPositions() );
	assert( patch[ip1].ring[3].hasValidPositions() );

	float level = 0.25f*(ring[im1].edge_level+ring[ip1].edge_level);
        patch[i].ring[1].edge_level = patch[ip1].ring[2].edge_level = level;
	center_vertex_level = max(center_vertex_level,level);

        center += ring[i].vtx;
        center_ring[2*i+0] = (Vertex_t)patch[i].ring[0].vtx;
        center_ring[2*i+1] = (Vertex_t)patch[i].ring[0].ring[0];
      }
      center /= float(N);

      for (size_t i=0; i<N; i++) {
        init_regular(center,center_ring,center_vertex_level,N,2*i,patch[i].ring[2]);
        
	assert( patch[i].ring[2].hasValidPositions() );
      }
    }

    void init(CatmullClarkPatch& patch) const
    {
      assert(size() == 4);
      ring[0].convert(patch.ring[0]);
      ring[1].convert(patch.ring[1]);
      ring[2].convert(patch.ring[2]);
      ring[3].convert(patch.ring[3]);
    }

    friend __forceinline std::ostream &operator<<(std::ostream &o, const GeneralCatmullClarkPatchT &p)
    {
      o << "GeneralCatmullClarkPatch { " << std::endl;
      for (size_t i=0; i<p.N; i++)
	o << "ring" << i << ": " << p.ring[i] << std::endl;
      o << "}" << std::endl;
      return o;
    }
  };

  typedef GeneralCatmullClarkPatchT<Vec3fa,Vec3fa_t> GeneralCatmullClarkPatch3fa;
}


