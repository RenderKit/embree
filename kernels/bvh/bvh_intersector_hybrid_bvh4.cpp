// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "bvh_intersector_hybrid.cpp"

namespace embree
{
  namespace isa
  {
    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector4 Definitions
    ////////////////////////////////////////////////////////////////////////////////
  
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4HybridMoeller,         BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 4 COMMA false> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4iIntersector4HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMiIntersectorKMoeller <SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4vIntersector4HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA TriangleMvIntersectorKPluecker<SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4iIntersector4HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA TriangleMiIntersectorKPluecker<SIMD_MODE(4) COMMA 4 COMMA true> > >));

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4vMBIntersector4HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMvMBIntersectorKMoeller <SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4iMBIntersector4HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMiMBIntersectorKMoeller <SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4vMBIntersector4HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA TriangleMvMBIntersectorKPluecker<SIMD_MODE(4) COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR4(BVH4Triangle4iMBIntersector4HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA TriangleMiMBIntersectorKPluecker<SIMD_MODE(4) COMMA 4 COMMA true> > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4vIntersector4HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoeller <4 COMMA 4 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4vIntersector4HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoeller <4 COMMA 4 COMMA false> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4iIntersector4HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiIntersectorKMoeller <4 COMMA 4 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4vIntersector4HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKPluecker<4 COMMA 4 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4iIntersector4HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA QuadMiIntersectorKPluecker<4 COMMA 4 COMMA true > > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4iMBIntersector4HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiMBIntersectorKMoeller <4 COMMA 4 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR4(BVH4Quad4iMBIntersector4HybridPluecker,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<4 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 4 COMMA true > > >));
   
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR4(BVH4Subdivpatch1Intersector4, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1Intersector4>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR4(BVH4Subdivpatch1EagerIntersector4, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1EagerIntersector4>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR4(BVH4Subdivpatch1CachedIntersector4, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector4>));
    IF_ENABLED_USER(DEFINE_INTERSECTOR4(BVH4VirtualIntersector4Chunk, BVHNIntersectorKChunk<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA ObjectIntersector4> >));
    IF_ENABLED_USER(DEFINE_INTERSECTOR4(BVH4VirtualMBIntersector4Chunk, BVHNIntersectorKChunk<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA ObjectIntersector4MB> >));


    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector8 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoeller,         BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 8 COMMA false> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4iIntersector8HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMiIntersectorKMoeller <SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA TriangleMvIntersectorKPluecker<SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4iIntersector8HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA TriangleMiIntersectorKPluecker<SIMD_MODE(4) COMMA 8 COMMA true> > >));

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4vMBIntersector8HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMvMBIntersectorKMoeller <SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4iMBIntersector8HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMiMBIntersectorKMoeller <SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4vMBIntersector8HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA TriangleMvMBIntersectorKPluecker<SIMD_MODE(4) COMMA 8 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH4Triangle4iMBIntersector8HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA TriangleMiMBIntersectorKPluecker<SIMD_MODE(4) COMMA 8 COMMA true> > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4vIntersector8HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoeller<4 COMMA 8 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4vIntersector8HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoeller<4 COMMA 8 COMMA false> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4iIntersector8HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiIntersectorKMoeller<4 COMMA 8 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4vIntersector8HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKPluecker<4 COMMA 8 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4iIntersector8HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<8 COMMA QuadMiIntersectorKPluecker<4 COMMA 8 COMMA true > > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4iMBIntersector8HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiMBIntersectorKMoeller <4 COMMA 8 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR8(BVH4Quad4iMBIntersector8HybridPluecker,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA true COMMA ArrayIntersectorK_1<8 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 8 COMMA true> > >));
   
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR8(BVH4Subdivpatch1Intersector8, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1Intersector8>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR8(BVH4Subdivpatch1EagerIntersector8, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1EagerIntersector8>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR8(BVH4Subdivpatch1CachedIntersector8, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector8>));

    IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH4VirtualIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8> >));
    IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH4VirtualMBIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8MB> >));

#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector16 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoeller,         BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 16 COMMA false> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4iIntersector16HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMiIntersectorKMoeller <SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4vIntersector16HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA TriangleMvIntersectorKPluecker<SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4iIntersector16HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA TriangleMiIntersectorKPluecker<SIMD_MODE(4) COMMA 16 COMMA true> > >));

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4vMBIntersector16HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMvMBIntersectorKMoeller <SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4iMBIntersector16HybridMoeller,  BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMiMBIntersectorKMoeller <SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4vMBIntersector16HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA TriangleMvMBIntersectorKPluecker<SIMD_MODE(4) COMMA 16 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR16(BVH4Triangle4iMBIntersector16HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA TriangleMiMBIntersectorKPluecker<SIMD_MODE(4) COMMA 16 COMMA true> > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4vIntersector16HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoeller <4 COMMA 16 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4vIntersector16HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoeller <4 COMMA 16 COMMA false> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4iIntersector16HybridMoeller,        BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiIntersectorKMoeller <4 COMMA 16 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4vIntersector16HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKPluecker<4 COMMA 16 COMMA true > > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4iIntersector16HybridPluecker,       BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA QuadMiIntersectorKPluecker<4 COMMA 16 COMMA true > > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4iMBIntersector16HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiMBIntersectorKMoeller <4 COMMA 16 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR16(BVH4Quad4iMBIntersector16HybridPluecker,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA true  COMMA ArrayIntersectorK_1<16 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 16 COMMA true> > >));
   
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR16(BVH4Subdivpatch1Intersector16, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1Intersector16>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR16(BVH4Subdivpatch1EagerIntersector16, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1EagerIntersector16>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR16(BVH4Subdivpatch1CachedIntersector16, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector16>));

    IF_ENABLED_USER(DEFINE_INTERSECTOR16(BVH4VirtualIntersector16Chunk, BVHNIntersectorKChunk<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA ObjectIntersector16> >));
    IF_ENABLED_USER(DEFINE_INTERSECTOR16(BVH4VirtualMBIntersector16Chunk, BVHNIntersectorKChunk<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA ObjectIntersector16MB> >));

#endif
  }
}

