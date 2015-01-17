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

#include "common/scene_subdiv_mesh.h"
#include "subdivpatch1base.h"

namespace embree
{

    // if (grid_size_simd_blocks == 1)
    //   {
    //     mic_m m_active = 0xffff;
    //     for (unsigned int i=grid_u_res-1;i<16;i+=grid_u_res)
    //       m_active ^= (unsigned int)1 << i;
    //     m_active &= ((unsigned int)1 << (grid_u_res * (grid_v_res-1)))-1;
    //     grid_mask = m_active;
    //   }

  void SubdivPatch1Base::updateEdgeLevels(const float edge_level[4],const SubdivMesh *const mesh)
  {
    /* init discrete edge tessellation levels and grid resolution */

    assert( edge_level[0] >= 0.0f );
    assert( edge_level[1] >= 0.0f );
    assert( edge_level[2] >= 0.0f );
    assert( edge_level[3] >= 0.0f );
      
    level[0] = max(ceilf(edge_level[0]),1.0f);
    level[1] = max(ceilf(edge_level[1]),1.0f);
    level[2] = max(ceilf(edge_level[2]),1.0f);
    level[3] = max(ceilf(edge_level[3]),1.0f);

    grid_u_res = max(level[0],level[2])+1; // n segments -> n+1 points
    grid_v_res = max(level[1],level[3])+1;

#if defined(__MIC__)
    grid_size_simd_blocks        = ((grid_u_res*grid_v_res+15)&(-16)) / 16;
    grid_subtree_size_64b_blocks = 5; // single leaf with u,v,x,y,z      
#else
    /* 8-wide SIMD is default on Xeon */
    grid_size_simd_blocks        = ((grid_u_res*grid_v_res+7)&(-8)) / 8;
    grid_subtree_size_64b_blocks = (sizeof(Quad2x2)+63) / 64; // single Quad2x2

#endif
    /* need stiching? */

    flags &= ~TRANSITION_PATCH;

    const unsigned int int_edge_points0 = (unsigned int)level[0] + 1;
    const unsigned int int_edge_points1 = (unsigned int)level[1] + 1;
    const unsigned int int_edge_points2 = (unsigned int)level[2] + 1;
    const unsigned int int_edge_points3 = (unsigned int)level[3] + 1;
      
    if (int_edge_points0 < (unsigned int)grid_u_res ||
	int_edge_points2 < (unsigned int)grid_u_res ||
	int_edge_points1 < (unsigned int)grid_v_res ||
	int_edge_points3 < (unsigned int)grid_v_res)
      flags |= TRANSITION_PATCH;

    /* tessellate into grid blocks for larger grid resolutions, generate bvh4 subtree over grid blocks*/

#if defined(__MIC__)
    const size_t leafBlocks = 4;
#else
    const size_t leafBlocks = (sizeof(Quad2x2)+63) / 64;
#endif
    grid_subtree_size_64b_blocks = getSubTreeSize64bBlocks( leafBlocks ); // u,v,x,y,z 

    /* has displacements? */
    flags &= ~HAS_DISPLACEMENT;
    if (mesh->displFunc != NULL)
      flags |= HAS_DISPLACEMENT;

  }


  /*! Construction from vertices and IDs. */
  SubdivPatch1Base::SubdivPatch1Base (const CatmullClarkPatch& ipatch,
                                      const unsigned int gID,
                                      const unsigned int pID,
                                      const SubdivMesh *const mesh,
                                      const Vec2f uv[4],
                                      const float edge_level[4]) 
    : geom(gID),
      prim(pID),  
      flags(0)
  {
    assert(sizeof(SubdivPatch1Base) == 5 * 64);

    for (size_t i=0;i<4;i++)
      {
        /* need to reverse input here */
        u[i] = (unsigned short)(uv[i].y * 65535.0f);
        v[i] = (unsigned short)(uv[i].x * 65535.0f);
      }


    updateEdgeLevels(edge_level,mesh);
     

    /* determine whether patch is regular or not */

    if (ipatch.isRegularOrFinal(0) && mesh->displFunc == NULL)
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



  void debugGridBorders(const SubdivPatch1Base &patch,
                        const SubdivMesh* const geom)
    {
      PING;
      assert( patch.grid_size_simd_blocks >= 1 );
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float grid_x[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_y[(patch.grid_size_simd_blocks+1)*8];
      __aligned(64) float grid_z[(patch.grid_size_simd_blocks+1)*8]; 
        
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*8];
     
#else
      const size_t array_elements = (patch.grid_size_simd_blocks + 1) * 8;
      float *const ptr = (float*)_malloca(5 * array_elements * sizeof(float) + 64);
      float *const grid_arrays = (float*)ALIGN_PTR(ptr,64);

      float *grid_x = &grid_arrays[array_elements * 0];
      float *grid_y = &grid_arrays[array_elements * 1];
      float *grid_z = &grid_arrays[array_elements * 2];
      float *grid_u = &grid_arrays[array_elements * 3];
      float *grid_v = &grid_arrays[array_elements * 4];

        
#endif   

      DBG_PRINT( patch.grid_size_simd_blocks );

      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);

      DBG_PRINT(patch.grid_u_res);
      DBG_PRINT(patch.grid_v_res);
      
      DBG_PRINT("top");
      for (size_t x=0;x<patch.grid_u_res;x++)
        {
          const size_t offset = patch.gridOffset(0,x);
          std::cout << x << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

      DBG_PRINT("right");
      for (size_t y=0;y<patch.grid_v_res;y++)
        {
          const size_t offset = patch.gridOffset(y,patch.grid_u_res-1);
          std::cout << y << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

      DBG_PRINT("buttom");
      for (ssize_t x=patch.grid_u_res-1;x>=0;x--)
        {
          const size_t offset = patch.gridOffset(patch.grid_v_res-1,x);
          std::cout << x << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }


      DBG_PRINT("left");
      for (ssize_t y=patch.grid_v_res-1;y>=0;y--)
        {
          const size_t offset = patch.gridOffset(y,0);
          std::cout << y << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif      
    }

}
