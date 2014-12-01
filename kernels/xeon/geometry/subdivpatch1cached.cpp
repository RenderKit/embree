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

#include "subdivpatch1cached.h"
#include "common/scene.h"

namespace embree
{
  SubdivPatch1Cached::Type SubdivPatch1Cached::type;
  
  SubdivPatch1Cached::Type::Type () 
    : PrimitiveType("subdivpatch1",sizeof(SubdivPatch1Cached),1,false,1) {} 
  
  size_t SubdivPatch1Cached::Type::blocks(size_t x) const {
    return x;
  }
    
  size_t SubdivPatch1Cached::Type::size(const char* This) const {
    return 1;
  }

  /*! Construction from vertices and IDs. */
    SubdivPatch1Cached::SubdivPatch1Cached (const SubdivMesh::HalfEdge * first_half_edge,
                                            const Vec3fa *vertices,
                                            const unsigned int gID,
                                            const unsigned int pID,
                                            const SubdivMesh *const mesh) 
      : geom(gID),
        prim(pID),  
        flags(0)
    {
      assert(sizeof(SubdivPatch1Cached) == 5 * 64);

      u_range = Vec2f(0.0f,1.0f);
      v_range = Vec2f(0.0f,1.0f);

      /* init irregular patch */

      CatmullClarkPatch ipatch ( first_half_edge, vertices ); 

      /* init discrete edge tessellation levels and grid resolution */

      assert( ipatch.ring[0].edge_level >= 0.0f );
      assert( ipatch.ring[1].edge_level >= 0.0f );
      assert( ipatch.ring[2].edge_level >= 0.0f );
      assert( ipatch.ring[3].edge_level >= 0.0f );

#if 1
      level[0] = max(ceilf(ipatch.ring[0].edge_level),1.0f);
      level[1] = max(ceilf(ipatch.ring[1].edge_level),1.0f);
      level[2] = max(ceilf(ipatch.ring[2].edge_level),1.0f);
      level[3] = max(ceilf(ipatch.ring[3].edge_level),1.0f);
#else
      /* debugging */
      level[0] = 4;
      level[1] = 4;
      level[2] = 4;
      level[3] = 4;
#endif

      grid_u_res = max(level[0],level[2])+1; // n segments -> n+1 points
      grid_v_res = max(level[1],level[3])+1;

      grid_size_8wide_blocks       = ((grid_u_res*grid_v_res+7)&(-8)) / 8;

      DBG_PRINT( grid_size_8wide_blocks );
      grid_subtree_size_64b_blocks = 4; // single bvh4 leaf with 3x3 grid => 4 cachelines

      /* need stiching? */

      const unsigned int int_edge_points0 = (unsigned int)level[0] + 1;
      const unsigned int int_edge_points1 = (unsigned int)level[1] + 1;
      const unsigned int int_edge_points2 = (unsigned int)level[2] + 1;
      const unsigned int int_edge_points3 = (unsigned int)level[3] + 1;

      if (int_edge_points0 < (unsigned int)grid_u_res ||
	  int_edge_points2 < (unsigned int)grid_u_res ||
	  int_edge_points1 < (unsigned int)grid_v_res ||
	  int_edge_points3 < (unsigned int)grid_v_res)
	flags |= TRANSITION_PATCH;
      

      /* has displacements? */
      if (mesh->displFunc != NULL)
	flags |= HAS_DISPLACEMENT;


      /* tessellate into 3x3 grid blocks for larger grid resolutions, generate bvh4 subtree over 3x3 grid blocks*/
      if (grid_size_8wide_blocks > 1)
	grid_subtree_size_64b_blocks = getSubTreeSize64bBlocks( 4 ); // u,v,x,y,z 

      /* determine whether patch is regular or not */

      if (ipatch.isRegular()) 
	{
	  flags |= REGULAR_PATCH;
	  patch.init( ipatch );
	}
      else
	{
	  GregoryPatch gpatch; 
	  gpatch.init( ipatch ); 
	  gpatch.exportDenseConrolPoints( patch.v );
	}
#if 0
      DBG_PRINT( grid_u_res );
      DBG_PRINT( grid_v_res );
      DBG_PRINT( grid_size_16wide_blocks );
      DBG_PRINT( grid_mask );
      DBG_PRINT( grid_subtree_size_64b_blocks );
#endif
    }

}
