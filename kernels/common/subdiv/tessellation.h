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

namespace embree
{
  
  __forceinline void stitchGridEdges(const unsigned int low_rate,
				    const unsigned int high_rate,
				    float * __restrict__ const uv_array,
				    const unsigned int uv_array_step)
  {
    assert(low_rate < high_rate);
    assert(high_rate >= 2);
    
    const float inv_low_rate = rcp((float)(low_rate-1));
    const unsigned int dy = low_rate  - 1; 
    const unsigned int dx = high_rate - 1;
    
    int p = 2*dy-dx;  
    
    unsigned int offset = 0;
    unsigned int y = 0;
    float value = 0.0f;
    for(unsigned int x=0;x<high_rate-1; x++) // '<=' would be correct but we will leave the 1.0f at the end
    {
      uv_array[offset] = value;
      
      offset += uv_array_step;      
      if(unlikely(p > 0))
      {
	y++;
	value = (float)y * inv_low_rate;
	p -= 2*dx;
      }
      p += 2*dy;
    }
  }
  
  __forceinline void stitchUVGrid(const float edge_levels[4],
				  const unsigned int grid_u_res,
				  const unsigned int grid_v_res,
				  float * __restrict__ const u_array,
				  float * __restrict__ const v_array)
  {
    const unsigned int int_edge_points0 = (unsigned int)edge_levels[0] + 1;
    const unsigned int int_edge_points1 = (unsigned int)edge_levels[1] + 1;
    const unsigned int int_edge_points2 = (unsigned int)edge_levels[2] + 1;
    const unsigned int int_edge_points3 = (unsigned int)edge_levels[3] + 1;
    
    if (unlikely(int_edge_points0 < grid_u_res))
      stitchGridEdges(int_edge_points0,grid_u_res,u_array,1);
    
    if (unlikely(int_edge_points2 < grid_u_res))
      stitchGridEdges(int_edge_points2,grid_u_res,&u_array[(grid_v_res-1)*grid_u_res],1);
    
    if (unlikely(int_edge_points1 < grid_v_res))
      stitchGridEdges(int_edge_points1,grid_v_res,&v_array[grid_u_res-1],grid_u_res);
    
    if (unlikely(int_edge_points3 < grid_v_res))
      stitchGridEdges(int_edge_points3,grid_v_res,v_array,grid_u_res);  
  }
  
  __forceinline void gridUVTessellator(const float edge_levels[4],
				       const unsigned int grid_u_res,
				       const unsigned int grid_v_res,
				       float * __restrict__ const u_array,
				       float * __restrict__ const v_array)
  {
    assert( grid_u_res >= 1);
    assert( grid_v_res >= 1);
    assert( edge_levels[0] >= 1.0f );
    assert( edge_levels[1] >= 1.0f );
    assert( edge_levels[2] >= 1.0f );
    assert( edge_levels[3] >= 1.0f );
    
#if defined(__MIC__)

    const int16 grid_u_segments = int16(grid_u_res)-1;
    const int16 grid_v_segments = int16(grid_v_res)-1;
    
    const float16 inv_grid_u_segments = rcp(float16(grid_u_segments));
    const float16 inv_grid_v_segments = rcp(float16(grid_v_segments));
    
    unsigned int index = 0;
    int16 v_i( zero );
    for (unsigned int y=0;y<grid_v_res;y++,index+=grid_u_res,v_i += 1)
    {
      int16 u_i ( step );
      
      const bool16 m_v = v_i < grid_v_segments;
      
      for (unsigned int x=0;x<grid_u_res;x+=16, u_i += 16)
      {
        const bool16 m_u = u_i < grid_u_segments;

	const float16 u = select(m_u, float16(u_i) * inv_grid_u_segments, 1.0f);
	const float16 v = select(m_v, float16(v_i) * inv_grid_v_segments, 1.0f);
	ustore16f(&u_array[index + x],u);
	ustore16f(&v_array[index + x],v);	   
      }
    }       

#else
 
#if defined(__AVX__)
    const int8 grid_u_segments = int8(grid_u_res)-1;
    const int8 grid_v_segments = int8(grid_v_res)-1;
    
    const float8 inv_grid_u_segments = rcp(float8(grid_u_segments));
    const float8 inv_grid_v_segments = rcp(float8(grid_v_segments));
    
    unsigned int index = 0;
    int8 v_i( zero );
    for (unsigned int y=0;y<grid_v_res;y++,index+=grid_u_res,v_i += 1)
    {
      int8 u_i ( step );
      
      const bool8 m_v = v_i < grid_v_segments;
      
      for (unsigned int x=0;x<grid_u_res;x+=8, u_i += 8)
      {
        const bool8 m_u = u_i < grid_u_segments;
	const float8 u = select(m_u, float8(u_i) * inv_grid_u_segments, 1.0f);
	const float8 v = select(m_v, float8(v_i) * inv_grid_v_segments, 1.0f);
	storeu8f(&u_array[index + x],u); // FIXME: store not always 8 bytes aligned !!
	storeu8f(&v_array[index + x],v);	   
      }
    }       
 #else   
    const unsigned int grid_u_segments = grid_u_res-1;
    const unsigned int grid_v_segments = grid_v_res-1;

    const float inv_grid_u_segments = rcp((float)grid_u_segments);
    const float inv_grid_v_segments = rcp((float)grid_v_segments);
    
    /* initialize grid */
    unsigned int index = 0;
    for (unsigned int y=0;y<grid_v_res;y++)
    {
      const float v = (float)y * inv_grid_v_segments;
      for (unsigned int x=0;x<grid_u_res;x++,index++)
      {
	u_array[index] = (float)x * inv_grid_u_segments;
	v_array[index] = v;
      }
    }
    const unsigned int num_points = index;
    
    /* set right and buttom border to exactly 1.0f */
    for (unsigned int y=0,i=grid_u_res-1;y<grid_v_res;y++,i+=grid_u_res)
      u_array[i] = 1.0f;
    for (unsigned int x=0;x<grid_u_res;x++)
      v_array[num_points-1-x] = 1.0f;
 #endif

#endif       
  }
 

}

