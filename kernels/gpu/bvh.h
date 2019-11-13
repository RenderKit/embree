// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#if defined(EMBREE_DPCPP_SUPPORT)
#include "common.h"
#include "AABB.h"
#include "AABB3f.h"

/* ====== BVH16 config ====== */

#define BINS 16

#define BVH_LEAF_MASK_SHIFT  3
#define BVH_LEAF_MASK        (1<<BVH_LEAF_MASK_SHIFT)
#define BVH_LOWER_BITS_MASK  ((1<<(BVH_LEAF_MASK_SHIFT+1))-1)
#define BVH_NODE_N           16
#define BVH_NODE_N_LOG       4

#define BVH_MAX_STACK_ENTRIES 64


/* ====== QUANTIZATION config ====== */

#define QUANT_BITS            8
#define QUANT_MIN             0
#define QUANT_MAX             255
#define QUANT_MAX_MANT        (255.0f/256.0f)

namespace embree
{
  namespace gpu
  {

    /* ======================================================================= */
    /* ============================== BVH BASE =============================== */
    /* ======================================================================= */
    
    struct BVHBase
    {
      AABB3f bounds; 
      unsigned long rootNodeOffset; 
      uint nodeDataStart;
      uint nodeDataCur;
      uint leafDataStart;
      uint leafDataCur;
      uint proceduralDataStart;
      uint proceduralDataCur;
      /* uint backPointerDataStart; */
      /* uint backPointerDataEnd; */
    };

    /* ========================================================================== */
    /* ============================== BVH NODEREF =============================== */
    /* ========================================================================== */

    struct NodeRef
    {
      __forceinline NodeRef () {}

      __forceinline NodeRef (uint offset) : offset(offset) {}

      __forceinline operator uint() const { return offset; }

      __forceinline size_t isLeaf() const { return offset & BVH_LEAF_MASK; }
      
      __forceinline uint getNumLeafPrims() { return (offset & (BVH_LEAF_MASK-1))+1; }      
      __forceinline uint getLeafOffset()   { return offset & (~BVH_LOWER_BITS_MASK); }
      
    private:
      uint offset;
    };


    
    /* ======================================================================== */
    /* ============================== BVH NODES =============================== */
    /* ======================================================================== */

    struct BVHNodeN
    {              
      uint offset[BVH_NODE_N];  
      uint parent[BVH_NODE_N]; 
      float lower_x[BVH_NODE_N]; 
      float upper_x[BVH_NODE_N]; 
      float lower_y[BVH_NODE_N]; 
      float upper_y[BVH_NODE_N]; 
      float lower_z[BVH_NODE_N]; 
      float upper_z[BVH_NODE_N]; 

      inline void initBVHNodeN(uint slotID)
      {
	const float pos_inf =  INFINITY;
	const float neg_inf = -INFINITY;	
	offset[slotID]  =  (uint)(-1);  
	parent[slotID]  =  (uint)(-1); 
	lower_x[slotID] =  pos_inf; 
	upper_x[slotID] =  neg_inf;
	lower_y[slotID] =  pos_inf; 
	upper_y[slotID] =  neg_inf;
	lower_z[slotID] =  pos_inf; 
	upper_z[slotID] =  neg_inf;  
      }


      inline void setBVHNodeN(const struct AABB &aabb, uint slot)
      {
	lower_x[slot] = aabb.lower.x();
	lower_y[slot] = aabb.lower.y();
	lower_z[slot] = aabb.lower.z();
	upper_x[slot] = aabb.upper.x();
	upper_y[slot] = aabb.upper.y();
	upper_z[slot] = aabb.upper.z();
      }

      inline void setBVHNodeN_offset(const struct AABB &aabb, const uint _offset, const uint _parent, uint slot)
      {
	offset[slot] = _offset;
	parent[slot] = _parent;  
	lower_x[slot] = aabb.lower.x();
	lower_y[slot] = aabb.lower.y();
	lower_z[slot] = aabb.lower.z();
	upper_x[slot] = aabb.upper.x();
	upper_y[slot] = aabb.upper.y();
	upper_z[slot] = aabb.upper.z();
      }


    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const BVHNodeN& node) {
      for (uint i=0;i<BVH_NODE_N;i++)
	{
	  out << " i " << i << " offset " << node.offset[i] << " lower_x " << node.lower_x[i] << " upper_x " << node.upper_x[i] << " lower_y " << node.lower_y[i] << " upper_y " << node.upper_y[i] << " lower_z " << node.lower_z[i] << " upper_z " << node.upper_z[i] << cl::sycl::endl;
	}      
      return out; 
    }
    
    
    /* ======================================================================== */
    /* ============================== QBVH NODE =============================== */
    /* ======================================================================== */
    
    struct QBVHNodeN
    {
      /* special layout requires just two subgroup block loads for entire node */
      
      uint   offset[BVH_NODE_N];
      float4 org;
      float4 scale;

      struct {
	uchar lower;
	uchar upper;
      } bounds_x[BVH_NODE_N];

      struct {
	uchar lower;
	uchar upper;
      } bounds_y[BVH_NODE_N];

      struct {  
	uchar lower;
	uchar upper;    
      } bounds_z[BVH_NODE_N];
      
      inline AABB getBounds(const uint i)
      {
	const uchar4 ilower(bounds_x[i].lower,bounds_y[i].lower,bounds_z[i].lower,0);
	const uchar4 iupper(bounds_x[i].upper,bounds_y[i].upper,bounds_z[i].upper,0);	      
	const float4 lowerf  = ilower.convert<float,cl::sycl::rounding_mode::rtn>();
	const float4 upperf  = iupper.convert<float,cl::sycl::rounding_mode::rtp>();	
	AABB aabb;
	aabb.lower = cl::sycl::fma(lowerf,scale,org);
	aabb.upper = cl::sycl::fma(upperf,scale,org);
	return aabb;
      }

      inline AABB getBounds()
      {
	AABB aabb;
	aabb.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  {
	    if (offset[i] == -1) break;
	    aabb.extend(getBounds(i));
	  }
	return aabb;
      }

      
      inline static void init(cl::sycl::intel::sub_group &sg,
			      QBVHNodeN &node,			      
			      AABB *childrenAABB,
			      uint numChildren,
			      const uint ID,
			      const cl::sycl::stream &out)
      {
	AABB child;
	const uint subgroupLocalID = sg.get_local_id()[0];
	if (subgroupLocalID < numChildren)
	  child = childrenAABB[ID];
	else
	  child.init();
	AABB aabb = child.sub_group_reduce(sg);
	
	const float4 minF = aabb.lower;
	const float4 diff = aabb.size()*(1.0f+2.0f*FLT_MIN);
	float4 decode_scale = diff / (float4)((float)QUANT_MAX);
	decode_scale = cselect(decode_scale != 0.0f, decode_scale, float4(2.0f*FLT_MIN));
	float4 encode_scale = (float4)((float)QUANT_MAX) / diff;
	encode_scale = cselect(diff > 0.0f, encode_scale, float4(0.0f));
	
	if (subgroupLocalID < BVH_NODE_N)
	{

	  int m_valid = subgroupLocalID < numChildren;
	  
	  const float4 lower = child.lower;
	  const float4 upper = child.upper;
	  
	  float4 lowerf = floor((lower - minF)*encode_scale);
	  float4 upperf =  ceil((upper - minF)*encode_scale);

	  int4 ilower = max(lowerf.convert<int,cl::sycl::rounding_mode::rtn>(),int4(QUANT_MIN));
	  int4 iupper = min(upperf.convert<int,cl::sycl::rounding_mode::rtp>(),int4(QUANT_MAX));

	  /* lower/upper correction */
	  int4 m_lower_correction = (cl::sycl::fma(ilower.convert<float,cl::sycl::rounding_mode::rtn>(),decode_scale,minF)) > lower;
	  int4 m_upper_correction = (cl::sycl::fma(iupper.convert<float,cl::sycl::rounding_mode::rtp>(),decode_scale,minF)) < upper;
	  
	  ilower = max(cselect(m_lower_correction,ilower-1,ilower),QUANT_MIN);
	  iupper = min(cselect(m_upper_correction,iupper+1,iupper),QUANT_MAX);	  

	  /* disable invalid lanes */	  

#if 1 // workaround for compiler bug	  
	  ilower.x() = cselect(m_valid,ilower.x(),QUANT_MAX);
	  ilower.y() = cselect(m_valid,ilower.y(),QUANT_MAX);
	  ilower.z() = cselect(m_valid,ilower.z(),QUANT_MAX);
	  
	  iupper.x() = cselect(m_valid,iupper.x(),QUANT_MIN);
	  iupper.y() = cselect(m_valid,iupper.y(),QUANT_MIN);
	  iupper.z() = cselect(m_valid,iupper.z(),QUANT_MIN);	  
#else
	  ilower = cselect(int4(m_valid),ilower,int4(QUANT_MAX)); 
	  iupper = cselect(int4(m_valid),iupper,int4(QUANT_MIN));
#endif	  
	  
	  node.offset[subgroupLocalID] = -1;
	  node.bounds_x[subgroupLocalID].lower = ilower.x();
	  node.bounds_y[subgroupLocalID].lower = ilower.y();
	  node.bounds_z[subgroupLocalID].lower = ilower.z();
	  node.bounds_x[subgroupLocalID].upper = iupper.x();
	  node.bounds_y[subgroupLocalID].upper = iupper.y();
	  node.bounds_z[subgroupLocalID].upper = iupper.z();	  
	  node.org   = minF;
	  node.scale = decode_scale;	  
	}
      }

    };

    
    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const QBVHNodeN& node) {
      out << " org "     << node.org
	  << " scale "   << node.scale
	  << cl::sycl::endl;
      
      for (uint i=0;i<BVH_NODE_N;i++)
	{
	  out << " offset  " << node.offset[i]
	      << " lower_x " << (int)node.bounds_x[i].lower
	      << " upper_x " << (int)node.bounds_x[i].upper
	      << " lower_y " << (int)node.bounds_y[i].lower
	      << " upper_y " << (int)node.bounds_y[i].upper
	      << " lower_z " << (int)node.bounds_z[i].lower
	      << " upper_z " << (int)node.bounds_z[i].upper
	      << cl::sycl::endl;
	}      
      return out; 
    }

    /* =============================================================================== */
    /* ============================== NODE INTERSECTION =============================== */
    /* =============================================================================== */
    struct NodeIntersectionData {
      inline NodeIntersectionData(float dist, uint valid, uint offset) : dist(dist), valid(valid), offset(offset) {}      
      float dist;
      uint valid;
      uint offset;
    };

    inline NodeIntersectionData intersectQBVHNodeN(const cl::sycl::intel::sub_group &sg,
						   const QBVHNodeN &node,
						   const uint3 &dir_mask,
						   const float3 &inv_dir,
						   const float3 &inv_dir_org,
						   const float tnear,
						   const float tfar)
    {
#if 0
      const gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_base + cur);	      
      uint  offset          = node.offset[subgroupLocalID];	      	      
      const float3 org      = node.org.xyz();
      const float3 scale    = node.scale.xyz();
#else
      cl::sycl::multi_ptr<uint,cl::sycl::access::address_space::global_space> node_ptr((uint*)&node);
      const uint2 block0 = sg.load<2,uint>(node_ptr);
      uint offset = (uint)block0.x();
      const float3 org(gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 0)),
		       gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 1)),
		       gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 2)));
      const float3 scale(gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 4)),
			 gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 5)),
			 gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 6)));	      
#endif

      cl::sycl::multi_ptr<ushort,cl::sycl::access::address_space::global_space> quant_ptr((ushort*)&node + 32); // + 64 bytes
      const ushort4 block1 = sg.load<4,ushort>(quant_ptr);
      const cl::sycl::uchar8 block2 = block1.as<cl::sycl::uchar8>();

      const uchar3 ilower(block2.s2(),block2.s4(),block2.s6());
      const uchar3 iupper(block2.s3(),block2.s5(),block2.s7());

      const float3 lowerf  = ilower.convert<float,cl::sycl::rounding_mode::rtn>();
      const float3 upperf  = iupper.convert<float,cl::sycl::rounding_mode::rtp>();
      const float3 _lower  = cfma(lowerf,scale,org);
      const float3 _upper  = cfma(upperf,scale,org);

      const float lower_x = cselect((uint)dir_mask.x(),(float)_upper.x(),(float)_lower.x());
      const float upper_x = cselect((uint)dir_mask.x(),(float)_lower.x(),(float)_upper.x());
      const float lower_y = cselect((uint)dir_mask.y(),(float)_upper.y(),(float)_lower.y());
      const float upper_y = cselect((uint)dir_mask.y(),(float)_lower.y(),(float)_upper.y());
      const float lower_z = cselect((uint)dir_mask.z(),(float)_upper.z(),(float)_lower.z());
      const float upper_z = cselect((uint)dir_mask.z(),(float)_lower.z(),(float)_upper.z());	     
	      
      const float lowerX = cfma((float)inv_dir.x(), lower_x, (float)inv_dir_org.x());
      const float upperX = cfma((float)inv_dir.x(), upper_x, (float)inv_dir_org.x());
      const float lowerY = cfma((float)inv_dir.y(), lower_y, (float)inv_dir_org.y());
      const float upperY = cfma((float)inv_dir.y(), upper_y, (float)inv_dir_org.y());
      const float lowerZ = cfma((float)inv_dir.z(), lower_z, (float)inv_dir_org.z());
      const float upperZ = cfma((float)inv_dir.z(), upper_z, (float)inv_dir_org.z());

      const float fnear = cl::sycl::fmax( cl::sycl::fmax(lowerX,lowerY), cl::sycl::fmax(lowerZ,tnear) );
      const float ffar  = cl::sycl::fmin( cl::sycl::fmin(upperX,upperY), cl::sycl::fmin(upperZ,tfar)  );
      const uint valid = (fnear <= ffar) & (offset != -1); //((uchar)ilower.x() <= (uchar)iupper.x());	       
      return NodeIntersectionData(fnear, valid, offset);
    }
    
  };
};

#endif
