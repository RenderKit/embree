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
#include "common/scene.h"

namespace embree
{

  /*! Construction from vertices and IDs. */
  SubdivPatch1Base::SubdivPatch1Base (const CatmullClarkPatch3fa& ipatch,
                                      const unsigned int gID,
                                      const unsigned int pID,
                                      const SubdivMesh *const mesh,
                                      const Vec2f uv[4],
                                      const float edge_level[4]) 
    : geom(gID),prim(pID),flags(0),root_ref(0)
  {
    assert(sizeof(SubdivPatch1Base) == 5 * 64);
    mtx.reset();

    for (size_t i=0;i<4;i++)
      {
        /* need to reverse input here */
        u[i] = (unsigned short)(uv[i].x * 65535.0f);
        v[i] = (unsigned short)(uv[i].y * 65535.0f);
      }

    updateEdgeLevels(edge_level,mesh);
     
    /* determine whether patch is regular or not */

    if (ipatch.isRegular()) /* bezier vs. gregory */
      {
#if 1
        flags |= BEZIER_PATCH;
        GregoryPatch gpatch; 
        gpatch.init_bezier( ipatch ); 
        gpatch.exportDenseConrolPoints( patch.v );
#else
        flags |= BSPLINE_PATCH;
        patch.init( ipatch );
#endif
      }
    else
      {
        flags |= GREGORY_PATCH;
        GregoryPatch gpatch; 
        gpatch.init( ipatch ); 
        gpatch.exportDenseConrolPoints( patch.v );
      }
  }

  SubdivPatch1Base::SubdivPatch1Base (const unsigned int gID,
                                      const unsigned int pID,
                                      const SubdivMesh *const mesh) 
    : geom(gID),prim(pID),flags(0),root_ref(0)
  {
  }

  void SubdivPatch1Base::updateEdgeLevels(const float edge_level[4],const SubdivMesh *const mesh)
  {
    /* init discrete edge tessellation levels and grid resolution */

    assert( edge_level[0] >= 0.0f );
    assert( edge_level[1] >= 0.0f );
    assert( edge_level[2] >= 0.0f );
    assert( edge_level[3] >= 0.0f );

#if defined(__MIC__)
    const size_t SIMD_WIDTH = 16;
#else
    const size_t SIMD_WIDTH = 8;
#endif

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
    grid_bvh_size_64b_blocks = getSubTreeSize64bBlocks( 0 );
    
#if COMPACT == 1
    //PRINT( grid_bvh_size_64b_blocks );
    //PRINT( grid_u_res);
    //PRINT( grid_v_res);

    const size_t grid_size_xyzuv = (grid_size_simd_blocks * SIMD_WIDTH) * 4;
    grid_subtree_size_64b_blocks = grid_bvh_size_64b_blocks + ((grid_size_xyzuv+15) / 16);
#else
    grid_subtree_size_64b_blocks = getSubTreeSize64bBlocks( leafBlocks ); // u,v,x,y,z 
#endif


    /* has displacements? */
    flags &= ~HAS_DISPLACEMENT;
    if (mesh->displFunc != nullptr)
      flags |= HAS_DISPLACEMENT;

  }



  void SubdivPatch1Base::evalToOBJ(Scene *scene,size_t &vertex_index, size_t &numTotalTriangles)
  {
#if defined(__MIC__)
    THROW_RUNTIME_ERROR("EVALTOOBJ NOT SUPPORTED ON MIC");
#else

#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
    __aligned(64) float grid_x[(grid_size_simd_blocks+1)*8]; 
    __aligned(64) float grid_y[(grid_size_simd_blocks+1)*8];
    __aligned(64) float grid_z[(grid_size_simd_blocks+1)*8]; 
    
    __aligned(64) float grid_u[(grid_size_simd_blocks+1)*8]; 
    __aligned(64) float grid_v[(grid_size_simd_blocks+1)*8];

    __aligned(64) float grid_nx[(grid_size_simd_blocks+1)*8]; 
    __aligned(64) float grid_ny[(grid_size_simd_blocks+1)*8];
    __aligned(64) float grid_nz[(grid_size_simd_blocks+1)*8]; 
 
#else
      const size_t array_elements = (grid_size_simd_blocks + 1) * 8;
      float *const ptr = (float*)_malloca(8 * array_elements * sizeof(float) + 64);
      float *const grid_arrays = (float*)ALIGN_PTR(ptr,64);

      float *grid_x = &grid_arrays[array_elements * 0];
      float *grid_y = &grid_arrays[array_elements * 1];
      float *grid_z = &grid_arrays[array_elements * 2];
      float *grid_u = &grid_arrays[array_elements * 3];
      float *grid_v = &grid_arrays[array_elements * 4];

      float *grid_nx = &grid_arrays[array_elements * 5];
      float *grid_ny = &grid_arrays[array_elements * 6];
      float *grid_nz = &grid_arrays[array_elements * 7];

#endif
      SubdivMesh *mesh = (SubdivMesh *)scene->getSubdivMesh(geom);
  
      evalGrid(*this,grid_x,grid_y,grid_z,grid_u,grid_v,mesh);

#if defined(__AVX__)
    for (size_t i=0;i<grid_size_simd_blocks;i++)
      {
        avxf uu = load8f(&grid_u[8*i]);
        avxf vv = load8f(&grid_v[8*i]);
        avx3f normal = normalize(patch.normal(uu,vv));
        *(avxf*)&grid_nx[8*i] = normal.x;
        *(avxf*)&grid_ny[8*i] = normal.y;
        *(avxf*)&grid_nz[8*i] = normal.z;        
      }
#else
    for (size_t i=0;i<grid_size_simd_blocks*2;i++) // 4-wide blocks for SSE
      {
        ssef uu      = load4f(&grid_u[4*i]);
        ssef vv      = load4f(&grid_v[4*i]);
        sse3f normal = normalize(patch.normal(uu,vv));
        *(ssef*)&grid_nx[4*i] = normal.x;
        *(ssef*)&grid_ny[4*i] = normal.y;
        *(ssef*)&grid_nz[4*i] = normal.z;        
      }
#endif
      
      std::cout << "# grid_u_res " << grid_u_res << std::endl;
      std::cout << "# grid_v_res " << grid_v_res << std::endl;

      /* vertex */
      for (size_t v=0;v<grid_v_res;v++)
        for (size_t u=0;u<grid_u_res;u++)
          {
            size_t offset = v * grid_u_res + u;
            std::cout << "v " << grid_x[offset] << " " << grid_y[offset] << " " << grid_z[offset] << std::endl;
          }

      /* normal */
      for (size_t v=0;v<grid_v_res;v++)
        for (size_t u=0;u<grid_u_res;u++)
          {
            size_t offset = v * grid_u_res + u;
            std::cout << "vn " << grid_nx[offset] << " " << grid_ny[offset] << " " << grid_nz[offset] << std::endl;
          }

      for (size_t v=0;v<grid_v_res-1;v++)
        for (size_t u=0;u<grid_u_res-1;u++)
          {
            size_t offset0 = vertex_index + v * grid_u_res + u;
            size_t offset1 = offset0 + grid_u_res;
            //std::cout << "f " << offset0+1 << " " << offset0+2 << " " << offset1+2 << " " << offset1+1 << std::endl;
            std::cout << "f " << offset0+1 << "//" << offset0+1 << " " << offset0+2<< "//" << offset0+2 << " " << offset1+2<< "//" << offset1+2 << " " << offset1+1<< "//" << offset1+1 << std::endl;
	    numTotalTriangles += 2;
          }
      vertex_index += grid_u_res*grid_v_res;
      
#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif      

#endif
  }

}
