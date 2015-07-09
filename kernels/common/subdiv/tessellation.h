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
  /* adjust discret tessellation level for feature-adaptive pre-subdivision */
  __forceinline float adjustTessellationLevel(float l, const int sublevel = 0)
  {
    for (size_t i=0; i<sublevel; i++) l *= 0.5f;
    float r = ceilf(l);      
    for (size_t i=0; i<sublevel; i++) r *= 2.0f;
    return r;
  }
  
  __forceinline int stitch(const int x, const int fine, const int coarse) {
    return (2*x+1)*coarse/(2*fine);
  }

  __forceinline void stitchGridEdges(const unsigned int low_rate,
                                     const unsigned int high_rate,
                                     const unsigned int x0,
                                     const unsigned int x1,
				    float * __restrict__ const uv_array,
				    const unsigned int uv_array_step)
  {
#if 1
    const float inv_low_rate = rcp((float)(low_rate-1));
    for (size_t x=x0; x<=x1; x++) {
      uv_array[(x-x0)*uv_array_step] = float(stitch(x,high_rate-1,low_rate-1))*inv_low_rate;
    }
    if (unlikely(x1 == high_rate-1))
      uv_array[(x1-x0)*uv_array_step] = 1.0f;
#else
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
      if (unlikely(p > 0))
      {
	y++;
	value = (float)y * inv_low_rate;
	p -= 2*dx;
      }
      p += 2*dy;
    }
#endif
  }
  
  __forceinline void stitchUVGrid(const float edge_levels[4],
                                  const unsigned int swidth,
                                  const unsigned int sheight,
                                  const unsigned int x0,
                                  const unsigned int y0,
				  const unsigned int grid_u_res,
				  const unsigned int grid_v_res,
				  float * __restrict__ const u_array,
				  float * __restrict__ const v_array)
  {
    const unsigned int x1 = x0+grid_u_res-1;
    const unsigned int y1 = y0+grid_v_res-1;
    const unsigned int int_edge_points0 = (unsigned int)edge_levels[0] + 1;
    const unsigned int int_edge_points1 = (unsigned int)edge_levels[1] + 1;
    const unsigned int int_edge_points2 = (unsigned int)edge_levels[2] + 1;
    const unsigned int int_edge_points3 = (unsigned int)edge_levels[3] + 1;
    
    if (unlikely(y0 == 0 && int_edge_points0 < swidth))
      stitchGridEdges(int_edge_points0,swidth,x0,x1,u_array,1);
    
    if (unlikely(y1 == sheight-1 && int_edge_points2 < swidth))
      stitchGridEdges(int_edge_points2,swidth,x0,x1,&u_array[(grid_v_res-1)*grid_u_res],1);
    
    if (unlikely(x0 == 0 && int_edge_points1 < sheight))
      stitchGridEdges(int_edge_points1,sheight,y0,y1,&v_array[grid_u_res-1],grid_u_res);
    
    if (unlikely(x1 == swidth-1 && int_edge_points3 < sheight))
      stitchGridEdges(int_edge_points3,sheight,y0,y1,v_array,grid_u_res);  
  }
  
  __forceinline void gridUVTessellator(const float edge_levels[4],  
                                       const unsigned int swidth,
                                       const unsigned int sheight,
                                       const unsigned int x0,
                                       const unsigned int y0,
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

    const int16 grid_u_segments = int16(swidth)-1;
    const int16 grid_v_segments = int16(sheight)-1;
    
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

	const float16 u = select(m_u, float16(x0+u_i) * inv_grid_u_segments, 1.0f);
	const float16 v = select(m_v, float16(y0+v_i) * inv_grid_v_segments, 1.0f);
	ustore16f(&u_array[index + x],u);
	ustore16f(&v_array[index + x],v);	   
      }
    }       

#else
 
#if defined(__AVX__)
    const int8 grid_u_segments = int8(swidth)-1;
    const int8 grid_v_segments = int8(sheight)-1;
    
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
	const float8 u = select(m_u, float8(x0+u_i) * inv_grid_u_segments, 1.0f);
	const float8 v = select(m_v, float8(y0+v_i) * inv_grid_v_segments, 1.0f);
	storeu8f(&u_array[index + x],u);
	storeu8f(&v_array[index + x],v);	   
      }
    }       
 #else   
    const int4 grid_u_segments = int4(swidth)-1;
    const int4 grid_v_segments = int4(sheight)-1;
    
    const float4 inv_grid_u_segments = rcp(float4(grid_u_segments));
    const float4 inv_grid_v_segments = rcp(float4(grid_v_segments));
    
    unsigned int index = 0;
    int4 v_i( zero );
    for (unsigned int y=0;y<grid_v_res;y++,index+=grid_u_res,v_i += 1)
    {
      int4 u_i ( step );
      
      const bool4 m_v = v_i < grid_v_segments;
      
      for (unsigned int x=0;x<grid_u_res;x+=4, u_i += 4)
      {
        const bool4 m_u = u_i < grid_u_segments;
	const float4 u = select(m_u, float4(x0+u_i) * inv_grid_u_segments, 1.0f);
	const float4 v = select(m_v, float4(y0+v_i) * inv_grid_v_segments, 1.0f);
	storeu4f(&u_array[index + x],u);
	storeu4f(&v_array[index + x],v);	   
      }
    }       
 #endif

#endif       
  }
 

}

