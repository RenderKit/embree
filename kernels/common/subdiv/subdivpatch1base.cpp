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

#include "../scene_subdiv_mesh.h"
#include "subdivpatch1base.h"
#include "../scene.h"

namespace embree
{

 
  SubdivPatch1Base::SubdivPatch1Base (const unsigned int gID,
                                      const unsigned int pID,
                                      const unsigned int subPatch,
                                      const SubdivMesh *const mesh,
                                      const Vec2f uv[4],
                                      const float edge_level[4],
                                      const int subdiv[4],
                                      const int simd_width)
    : geom(gID),prim(pID),flags(0),type(INVALID_PATCH)
  {
    static_assert(sizeof(SubdivPatch1Base) == 5 * 64, "SubdivPatch1Base has wrong size");
    mtx.reset();

    const SubdivMesh::HalfEdge* edge = mesh->getHalfEdge(pID);

    if (edge->patch_type == SubdivMesh::REGULAR_QUAD_PATCH) 
    {
#if PATCH_USE_BEZIER_PATCH 
      type = BEZIER_PATCH;
      new (patch_v) BezierPatch3fa(BSplinePatch3fa(CatmullClarkPatch3fa(edge,mesh->getVertexBuffer())));
#else
      type = BSPLINE_PATCH;
      new (patch_v) BSplinePatch3fa(CatmullClarkPatch3fa(edge,mesh->getVertexBuffer()));
#endif      
    }
#if PATCH_USE_GREGORY == 2
    else if (edge->patch_type == SubdivMesh::IRREGULAR_QUAD_PATCH) 
    {
      type = GREGORY_PATCH;
      new (patch_v) DenseGregoryPatch3fa(GregoryPatch3fa(CatmullClarkPatch3fa(edge,mesh->getVertexBuffer())));
    }
#endif
    else
    {
      type = EVAL_PATCH;
      this->edge = mesh->getHalfEdge(pID);
      this->subPatch = subPatch;
    }

    for (size_t i=0; i<4; i++) {
      u[i] = (unsigned short)(uv[i].x * 65535.0f);
      v[i] = (unsigned short)(uv[i].y * 65535.0f);
    }

    updateEdgeLevels(edge_level,subdiv,mesh,simd_width);
  }


  /*! Construction from vertices and IDs. */
  SubdivPatch1Base::SubdivPatch1Base (const CatmullClarkPatch3fa& ipatch,
                                      const int fas_depth,
                                      const unsigned int gID,
                                      const unsigned int pID,
                                      const SubdivMesh *const mesh,
                                      const Vec2f uv[4],
                                      const float edge_level[4],
                                      const int neighborSubdiv[4],
                                      const BezierCurve3fa *border, 
                                      const int border_flags,
                                      const int simd_width) 
    : geom(gID),prim(pID),flags(0)
  {
    static_assert(sizeof(SubdivPatch1Base) == 5 * 64, "SubdivPatch1Base has wrong size");
    mtx.reset();

    for (size_t i=0; i<4; i++) {
      u[i] = (unsigned short)(uv[i].x * 65535.0f);
      v[i] = (unsigned short)(uv[i].y * 65535.0f);
    }

    updateEdgeLevels(edge_level,neighborSubdiv,mesh,simd_width);
    
    /* determine whether patch is regular or not */
    if (fas_depth == 0 && ipatch.isRegular1() && !ipatch.hasBorder()) /* only select b-spline/bezier in the interior and not FAS-based patches*/
    {
#if 0
      type = BEZIER_PATCH;
      new (patch_v) BezierPatch3fa(BSplinePatch3fa(ipatch));
#else
      type = BSPLINE_PATCH;
      new (patch_v) BSplinePatch3fa(ipatch);
#endif      
    }
    else
    {
      type = GREGORY_PATCH;
      GregoryPatch3fa gpatch; 
      gpatch.init_crackfix( ipatch, fas_depth, neighborSubdiv, border, border_flags ); 
      new (patch_v) DenseGregoryPatch3fa(gpatch);
    }
  }

  void SubdivPatch1Base::updateEdgeLevels(const float edge_level[4], const int subdiv[4], const SubdivMesh *const mesh, const int simd_width)
  {
    /* init discrete edge tessellation levels and grid resolution */
    assert( edge_level[0] >= 0.0f );
    assert( edge_level[1] >= 0.0f );
    assert( edge_level[2] >= 0.0f );
    assert( edge_level[3] >= 0.0f );

    level[0] = max(ceilf(adjustTessellationLevel(edge_level[0],subdiv[0])),1.0f);
    level[1] = max(ceilf(adjustTessellationLevel(edge_level[1],subdiv[1])),1.0f);
    level[2] = max(ceilf(adjustTessellationLevel(edge_level[2],subdiv[2])),1.0f);
    level[3] = max(ceilf(adjustTessellationLevel(edge_level[3],subdiv[3])),1.0f);

    grid_u_res = max(level[0],level[2])+1; // n segments -> n+1 points
    grid_v_res = max(level[1],level[3])+1;

    /* workaround for 2x2 intersection stencil */
#if !defined(__MIC__)
    grid_u_res = max(grid_u_res,3); // FIXME: this triggers stitching
    grid_v_res = max(grid_v_res,3);
#endif
    grid_size_simd_blocks = ((grid_u_res*grid_v_res+simd_width-1)&(-simd_width)) / simd_width;
    grid_bvh_size_64b_blocks = getSubTreeSize64bBlocks( 0 );
    const size_t grid_size_xyzuv = (grid_size_simd_blocks * simd_width) * 4;
    grid_subtree_size_64b_blocks = grid_bvh_size_64b_blocks + ((grid_size_xyzuv+15) / 16);

    /* need stiching? */
    flags &= ~TRANSITION_PATCH;
    const unsigned int int_edge_points0 = (unsigned int)level[0] + 1;
    const unsigned int int_edge_points1 = (unsigned int)level[1] + 1;
    const unsigned int int_edge_points2 = (unsigned int)level[2] + 1;
    const unsigned int int_edge_points3 = (unsigned int)level[3] + 1;
    if (int_edge_points0 < (unsigned int)grid_u_res ||
	int_edge_points2 < (unsigned int)grid_u_res ||
	int_edge_points1 < (unsigned int)grid_v_res ||
	int_edge_points3 < (unsigned int)grid_v_res) {
      flags |= TRANSITION_PATCH;
    }
  }
}
