// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
    GridSOA::GridSOA(const SubdivPatch1Base* patches, unsigned time_steps, 
                     const unsigned x0, const unsigned x1, const unsigned y0, const unsigned y1, const unsigned swidth, const unsigned sheight,
                     const SubdivMesh* const geom, const size_t bvhBytes, const size_t gridBytes, BBox3fa* bounds_o)
      : root(BVH4::emptyNode), time_steps(time_steps), width(x1-x0+1), height(y1-y0+1), dim_offset(width*height), 
        geomID(patches->geom), primID(patches->prim), bvhBytes(unsigned(bvhBytes)), gridOffset(max(1u,time_steps-1)*bvhBytes), gridBytes(unsigned(gridBytes))
    {      
      /* the generate loops need padded arrays, thus first store into these temporary arrays */
      unsigned temp_size = width*height+VSIZEX;
      dynamic_large_stack_array(float,local_grid_u,temp_size,64*64*sizeof(float));
      dynamic_large_stack_array(float,local_grid_v,temp_size,64*64*sizeof(float));
      dynamic_large_stack_array(float,local_grid_x,temp_size,64*64*sizeof(float));
      dynamic_large_stack_array(float,local_grid_y,temp_size,64*64*sizeof(float));
      dynamic_large_stack_array(float,local_grid_z,temp_size,64*64*sizeof(float));
      dynamic_large_stack_array(float,local_grid_uv,temp_size,64*64*sizeof(float));

      /* first create the grids for each time step */
      for (size_t t=0; t<time_steps; t++)
      {
        /* compute vertex grid (+displacement) */
        evalGrid(patches[t],x0,x1,y0,y1,swidth,sheight,
                 local_grid_x,local_grid_y,local_grid_z,local_grid_u,local_grid_v,geom);
        
        /* encode UVs */
        for (unsigned i=0; i<dim_offset; i+=VSIZEX) {
          const vintx iu = (vintx) clamp(vfloatx::load(&local_grid_u[i])*0xFFFF, vfloatx(0.0f), vfloatx(0xFFFF));
          const vintx iv = (vintx) clamp(vfloatx::load(&local_grid_v[i])*0xFFFF, vfloatx(0.0f), vfloatx(0xFFFF));
          vintx::storeu(&local_grid_uv[i], (iv << 16) | iu);
        }
        
        /* copy temporary data to compact grid */
        float* const grid_x  = (float*)(gridData(t) + 0*dim_offset);
        float* const grid_y  = (float*)(gridData(t) + 1*dim_offset);
        float* const grid_z  = (float*)(gridData(t) + 2*dim_offset);
        int  * const grid_uv = (int*  )(gridData(t) + 3*dim_offset);
        memcpy(grid_x, local_grid_x, dim_offset*sizeof(float));
        memcpy(grid_y, local_grid_y, dim_offset*sizeof(float));
        memcpy(grid_z, local_grid_z, dim_offset*sizeof(float));
        memcpy(grid_uv,local_grid_uv,dim_offset*sizeof(int));       
      }

      /* create normal BVH when no motion blur is active */
      if (time_steps == 1) {
        root = buildBVH(0,bounds_o);
      }

      /* otherwise build MBlur BVH */
      else
      {
        for (size_t t=0; t<time_steps-1; t++)
        {
          std::pair<BBox3fa,BBox3fa> bounds;
          root = buildMBlurBVH(t,&bounds);
          if (bounds_o) {
            bounds_o[t+0] = bounds.first;
            bounds_o[t+1] = bounds.second;
          }
        }
      }
    }

    size_t GridSOA::getBVHBytes(const GridRange& range, const size_t nodeBytes, const size_t leafBytes)
    {
      if (range.hasLeafSize()) 
        return leafBytes;
      
      __aligned(64) GridRange r[4];
      const size_t children = range.splitIntoSubRanges(r);
      
      size_t bytes = nodeBytes;
      for (size_t i=0; i<children; i++)
        bytes += getBVHBytes(r[i],nodeBytes,leafBytes);
      return bytes;
    }
    
    BVH4::NodeRef GridSOA::buildBVH(size_t time, BBox3fa* bounds_o)
    {
      BVH4::NodeRef root = 0; size_t allocator = 0;
      GridRange range(0,width-1,0,height-1);
      BBox3fa bounds = buildBVH(root,time,range,allocator);
      if (bounds_o) *bounds_o = bounds;
      assert(allocator == bvhBytes);
      return root;
    }

    BBox3fa GridSOA::buildBVH(BVH4::NodeRef& curNode, size_t time, const GridRange& range, size_t& allocator)
    {
      /*! create leaf node */
      if (unlikely(range.hasLeafSize()))
      {
        /* shift 2x2 quads that wrap around to the left */ // FIXME: causes intersection filter to be called multiple times for some triangles
        size_t u_start = range.u_start, u_end = range.u_end;
        size_t v_start = range.v_start, v_end = range.v_end;
        size_t u_size = u_end-u_start; assert(u_size > 0);
        size_t v_size = v_end-v_start; assert(v_size > 0);
        if (unlikely(u_size < 2 && u_start > 0)) u_start--;
        if (unlikely(v_size < 2 && v_start > 0)) v_start--;
        
        /* we store index of first subgrid vertex as leaf node */
        curNode = BVH4::encodeTypedLeaf(encodeLeaf(u_start,v_start),0);

        /* return bounding box */
        return calculateBounds(time,range);
      }
      
      /* create internal node */
      else 
      {
        /* allocate new bvh4 node */
        BVH4::Node* node = (BVH4::Node *)&bvhData(time)[allocator];
        allocator += sizeof(BVH4::Node);
        node->clear();
        
        /* split range */
        GridRange r[4];
        const unsigned children = range.splitIntoSubRanges(r);
      
        /* recurse into subtrees */
        BBox3fa bounds( empty );
        for (unsigned i=0; i<children; i++)
        {
          BBox3fa box = buildBVH( node->child(i), time, r[i], allocator);
          node->set(i,box);
          bounds.extend(box);
        }
        
        curNode = BVH4::encodeNode(node);
        assert(is_finite(bounds));
        return bounds;
      }
    }

    BVH4::NodeRef GridSOA::buildMBlurBVH(size_t time, std::pair<BBox3fa,BBox3fa>* bounds_o)
    {
      BVH4::NodeRef root = 0; size_t allocator = 0;
      GridRange range(0,width-1,0,height-1);
      std::pair<BBox3fa,BBox3fa> bounds = buildMBlurBVH(root,time,range,allocator);
      if (bounds_o) *bounds_o = bounds;
      assert(allocator == bvhBytes);
      return root;
    }

    std::pair<BBox3fa,BBox3fa> GridSOA::buildMBlurBVH(BVH4::NodeRef& curNode, size_t time, const GridRange& range, size_t& allocator)
    {
      /*! create leaf node */
      if (unlikely(range.hasLeafSize()))
      {
        /* shift 2x2 quads that wrap around to the left */ // FIXME: causes intersection filter to be called multiple times for some triangles
        size_t u_start = range.u_start, u_end = range.u_end;
        size_t v_start = range.v_start, v_end = range.v_end;
        size_t u_size = u_end-u_start; assert(u_size > 0);
        size_t v_size = v_end-v_start; assert(v_size > 0);
        if (unlikely(u_size < 2 && u_start > 0)) u_start--;
        if (unlikely(v_size < 2 && v_start > 0)) v_start--;
        
        /* we store index of first subgrid vertex as leaf node */
        const size_t value = 16*(v_start * width + u_start + 1); // +1 to not create empty leaf
        curNode = BVH4::encodeTypedLeaf((void*)value,0);

        /* return bounding box */
        const BBox3fa bounds0 = calculateBounds(time+0,range);
        const BBox3fa bounds1 = calculateBounds(time+1,range);
        return std::make_pair(bounds0,bounds1);
      }
      
      /* create internal node */
      else 
      {
        /* allocate new bvh4 node */
        BVH4::NodeMB* node = (BVH4::NodeMB *)&bvhData(time)[allocator];
        allocator += sizeof(BVH4::NodeMB);
        node->clear();
        
        /* split range */
        GridRange r[4];
        const unsigned children = range.splitIntoSubRanges(r);
      
        /* recurse into subtrees */
        std::pair<BBox3fa,BBox3fa> bounds(empty,empty);
        for (unsigned i=0; i<children; i++)
        {
          std::pair<BBox3fa,BBox3fa> box = buildMBlurBVH( node->child(i), time, r[i], allocator);
          node->set(i,box.first,box.second);
          bounds.first. extend(box.first);
          bounds.second.extend(box.second);
        }
        
        curNode = BVH4::encodeNode(node);
        assert(is_finite(bounds.first));
        assert(is_finite(bounds.second));
        return bounds;
      }
    }
  }
}
