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
 
#include "grid_soa.h"

namespace embree
{
  namespace isa
  {  
    GridSOA::GridSOA(const SubdivPatch1Base& patch, 
                     const size_t x0, const size_t x1, const size_t y0, const size_t y1, const size_t swidth, const size_t sheight,
                     const SubdivMesh* const geom, const size_t bvhBytes, BBox3fa* bounds_o)
      : root(BVH4::emptyNode), width(x1-x0+1), height(y1-y0+1), dim_offset(0), geomID(patch.geom), primID(patch.prim), bvhBytes(bvhBytes)
    {      
      dim_offset = calculate_grid_size(width,height);

      dynamic_large_stack_array(float,local_grid_u,dim_offset+vfloat::size,64*64);
      dynamic_large_stack_array(float,local_grid_v,dim_offset+vfloat::size,64*64);
      dynamic_large_stack_array(float,local_grid_x,dim_offset+vfloat::size,64*64);
      dynamic_large_stack_array(float,local_grid_y,dim_offset+vfloat::size,64*64);
      dynamic_large_stack_array(float,local_grid_z,dim_offset+vfloat::size,64*64);

      /* compute vertex grid (+displacement) */
      evalGrid(patch,x0,x1,y0,y1,swidth,sheight,
               local_grid_x,local_grid_y,local_grid_z,local_grid_u,local_grid_v,geom);

      /* copy temporary data to tessellation cache */
      float* const grid_x  = (float*)(gridData() + 0*dim_offset);
      float* const grid_y  = (float*)(gridData() + 1*dim_offset);
      float* const grid_z  = (float*)(gridData() + 2*dim_offset);
      int  * const grid_uv = (int*  )(gridData() + 3*dim_offset);
      
      /* copy points */
      memcpy(grid_x, local_grid_x, dim_offset*sizeof(float));
      memcpy(grid_y, local_grid_y, dim_offset*sizeof(float));
      memcpy(grid_z, local_grid_z, dim_offset*sizeof(float));
      
      /* encode UVs */
      for (size_t i=0; i<dim_offset; i+=vfloat::size) {
        const vint iu = (vint) clamp(vfloat::load(&local_grid_u[i])*0xFFFF, vfloat(0.0f), vfloat(0xFFFF));
        const vint iv = (vint) clamp(vfloat::load(&local_grid_v[i])*0xFFFF, vfloat(0.0f), vfloat(0xFFFF));
        vint::storeu(&grid_uv[i], (iv << 16) | iu); 
      }

      /* create BVH */
      root = buildBVH(bvhData(),gridData(),bvhBytes,bounds_o);
    }

    size_t GridSOA::getBVHBytes(const GridRange& range, const unsigned int leafBytes)
    {
      if (range.hasLeafSize()) 
        return leafBytes;
      
      __aligned(64) GridRange r[4];
      const unsigned int children = range.splitIntoSubRanges(r);
      
      size_t bytes = sizeof(BVH4::Node);
      for (unsigned int i=0;i<children;i++)
        bytes += getBVHBytes(r[i],leafBytes);
      return bytes;
    }
    
    BVH4::NodeRef GridSOA::buildBVH(char* node_array, float* grid_array, const size_t bvhBytes, BBox3fa* bounds_o)
    {
      BVH4::NodeRef root = 0; size_t allocator = 0;
      GridRange range(0,width-1,0,height-1);
      BBox3fa bounds = buildBVH(root,node_array,grid_array,range,allocator);
      if (bounds_o) *bounds_o = bounds;
      assert(allocator == bvhBytes);
      return root;
    }

    BBox3fa GridSOA::buildBVH(BVH4::NodeRef& curNode, char* node_array, float* grid_array, const GridRange& range, size_t& allocator)
    {
      /*! create leaf node */
      if (unlikely(range.hasLeafSize()))
      {
        const float* const grid_x_array = grid_array + 0 * dim_offset;
        const float* const grid_y_array = grid_array + 1 * dim_offset;
        const float* const grid_z_array = grid_array + 2 * dim_offset;
        
        /* compute the bounds just for the range! */
        BBox3fa bounds( empty );
        for (size_t v = range.v_start; v<=range.v_end; v++) 
        {
          for (size_t u = range.u_start; u<=range.u_end; u++)
          {
            const float x = grid_x_array[ v * width + u];
            const float y = grid_y_array[ v * width + u];
            const float z = grid_z_array[ v * width + u];
            bounds.extend( Vec3fa(x,y,z) );
          }
        }
        assert(is_finite(bounds));

        /* shift 2x2 quads that wrap around to the left */ // FIXME: causes intersection filter to be called multiple times for some triangles
        unsigned int u_start = range.u_start, u_end = range.u_end;
        unsigned int v_start = range.v_start, v_end = range.v_end;
        unsigned int u_size = u_end-u_start; assert(u_size > 0);
        unsigned int v_size = v_end-v_start; assert(v_size > 0);
        if (unlikely(u_size < 2 && u_start > 0)) u_start--;
        if (unlikely(v_size < 2 && v_start > 0)) v_start--;
        
        /* we store pointer to first subgrid vertex as leaf node */
        const size_t value = 16*(v_start * width + u_start + 1); // +1 to not create empty leaf
        curNode = BVH4::encodeTypedLeaf((void*)value,0);
        return bounds;
      }
      
      /* create internal node */
      else 
      {
        /* allocate new bvh4 node */
        BVH4::Node* node = (BVH4::Node *)&node_array[allocator];
        allocator += sizeof(BVH4::Node);
        node->clear();
        
        /* split range */
        GridRange r[4];
        const size_t children = range.splitIntoSubRanges(r);
      
        /* recurse into subtrees */
        BBox3fa bounds( empty );
        for (size_t i=0; i<children; i++)
        {
          BBox3fa box = buildBVH( node->child(i), node_array, grid_array, r[i], allocator);
          node->set(i,box);
          bounds.extend(box);
        }
        
        curNode = BVH4::encodeNode(node);
        assert(is_finite(bounds));
        return bounds;
      }
    }
  }
}
