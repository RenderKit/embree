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

#define BVH_LEAF_MASK_SHIFT   31
#define BVH_LEAF_MASK         (1<<BVH_LEAF_MASK_SHIFT)
#define BVH_NUM_ITEMS_SHIFT    4
#define BVH_LOWER_BITS_MASK   ((1<<(BVH_NUM_ITEMS_SHIFT))-1)
#define BVH_MAX_NUM_ITEMS     (BVH_LOWER_BITS_MASK+1)
#define BVH_NODE_N            16
#define BVH_NODE_N_LOG         4
#define BVH_MAX_STACK_ENTRIES 64
#define BVH_INVALID_NODE_REF  -1


/* ====== BVH Builder and Quantization config ====== */

#define BINS 16
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
      
      __forceinline uint getNumLeafPrims() const  { return (offset & BVH_LOWER_BITS_MASK)+1; }      
      __forceinline uint getLeafOffset() const  { return offset & (~(BVH_LOWER_BITS_MASK|BVH_LEAF_MASK)); }
      
      
    private:
      uint offset;
    };    
    
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
      
      __forceinline AABB getBounds(const uint i)
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

      __forceinline AABB getBounds()
      {
	AABB aabb;
	aabb.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  {
	    if (offset[i] == BVH_INVALID_NODE_REF) break;
	    aabb.extend(getBounds(i));
	  }
	return aabb;
      }

      
      __forceinline static void init(cl::sycl::intel::sub_group &sg,
				     QBVHNodeN &node,			      
				     AABB *childrenAABB,
				     uint numChildren,
				     const uint ID)
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
	  
	    node.offset[subgroupLocalID] = BVH_INVALID_NODE_REF;
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

    
    __forceinline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const QBVHNodeN& node) {
      out << " org "     << node.org
	  << " scale "   << node.scale
	  << cl::sycl::endl;
      
      for (uint i=0;i<BVH_NODE_N;i++)
	{
	  out << " offset  " << node.offset[i]
	      << " {("
	      << (int)node.bounds_x[i].lower << ","
	      << (int)node.bounds_y[i].lower << ","
	      << (int)node.bounds_z[i].lower
	      << "),("
	      << (int)node.bounds_x[i].upper << ","
	      << (int)node.bounds_y[i].upper << ","
	      << (int)node.bounds_z[i].upper
	      <<  ")}"	    
	      << cl::sycl::endl;
	}      
      return out; 
    }


    
    /* =========================================================================== */
    /* ============================== QBVH NODE MB =============================== */
    /* =========================================================================== */

    struct QBVHNodeNMB
    {              
      NodeRef offset[BVH_NODE_N];
      float lower_t[BVH_NODE_N];
      float upper_t[BVH_NODE_N];
      float4 org0, scale0;
      float4 org1, scale1;
     
      uchar lower_x0[BVH_NODE_N]; 
      uchar upper_x0[BVH_NODE_N]; 
      uchar lower_y0[BVH_NODE_N]; 
      uchar upper_y0[BVH_NODE_N]; 
      uchar lower_z0[BVH_NODE_N]; 
      uchar upper_z0[BVH_NODE_N];
      uchar lower_x1[BVH_NODE_N]; 
      uchar upper_x1[BVH_NODE_N]; 
      uchar lower_y1[BVH_NODE_N]; 
      uchar upper_y1[BVH_NODE_N]; 
      uchar lower_z1[BVH_NODE_N]; 
      uchar upper_z1[BVH_NODE_N];
      

      __forceinline AABB getBounds0(const uint i) const
      {
	const uchar4 ilower(lower_x0[i],lower_y0[i],lower_z0[i],0);
	const uchar4 iupper(upper_x0[i],upper_y0[i],upper_z0[i],0);	      
	const float4 lowerf  = ilower.convert<float,cl::sycl::rounding_mode::rtn>();
	const float4 upperf  = iupper.convert<float,cl::sycl::rounding_mode::rtp>();	
	AABB aabb;
	aabb.lower = cl::sycl::fma(lowerf,scale0,org0);
	aabb.upper = cl::sycl::fma(upperf,scale0,org0);
	return aabb;
      }

      __forceinline AABB getBounds1(const uint i) const 
      {
	const uchar4 ilower(lower_x1[i],lower_y1[i],lower_z1[i],0);
	const uchar4 iupper(upper_x1[i],upper_y1[i],upper_z1[i],0);	      
	const float4 lowerf  = ilower.convert<float,cl::sycl::rounding_mode::rtn>();
	const float4 upperf  = iupper.convert<float,cl::sycl::rounding_mode::rtp>();	
	AABB aabb;
	aabb.lower = cl::sycl::fma(lowerf,scale1,org1);
	aabb.upper = cl::sycl::fma(upperf,scale1,org1);
	return aabb;
      }
      
      __forceinline AABB getBounds0() const
      {
	AABB aabb;
	aabb.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  if (offset[i] != BVH_INVALID_NODE_REF)
	    aabb.extend(getBounds0(i));
	return aabb;
      }

      __forceinline AABB getBounds1() const
      {
	AABB aabb;
	aabb.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  if (offset[i] != BVH_INVALID_NODE_REF)
	    aabb.extend(getBounds1(i));
	return aabb;
      }
      

      __forceinline AABB bounds_lower(size_t i) const {
	const AABB b0 = getBounds0(i);
	const AABB b1 = getBounds1(i);
	const float t = lower_t[i];
	const float4 lower = gpu::lerp<float4>(b0.lower,b1.lower,t);
	const float4 upper = gpu::lerp<float4>(b0.upper,b1.upper,t);
	return AABB( lower, upper );
      }

      __forceinline AABB bounds_upper(size_t i) const {
	const AABB b0 = getBounds0(i);
	const AABB b1 = getBounds1(i);
	const float t = upper_t[i];
	const float4 lower = gpu::lerp<float4>(b0.lower,b1.lower,t);
	const float4 upper = gpu::lerp<float4>(b0.upper,b1.upper,t);
	return AABB( lower, upper );
      }
      
      __forceinline void clear()
      {
	for (size_t i=0;i<BVH_NODE_N;i++)
	  {
	    offset[i] = BVH_INVALID_NODE_REF;
	    lower_t[i] = 1.0f;
	    upper_t[i] = 0.0f;	    	    
	    lower_x0[i] = lower_y0[i] = lower_z0[i] = QUANT_MAX;
	    upper_x0[i] = upper_y0[i] = upper_z0[i] = QUANT_MIN;
	    lower_x1[i] = lower_y1[i] = lower_z1[i] = QUANT_MAX;
	    upper_x1[i] = upper_y1[i] = upper_z1[i] = QUANT_MIN;	    
	  }
      }
    };
    

    __forceinline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const QBVHNodeNMB& node) {
      for (uint i=0;i<BVH_NODE_N;i++)
	{
	  out << " i " << i
	      << " offset " << (uint)node.offset[i] << " offset " << node.offset[i].getLeafOffset() << " numPrims " << node.offset[i].getNumLeafPrims()	 
	      << " time (" << node.lower_t[i] << "," << node.upper_t[i] << ") "
	    << "bounds0 " << node.getBounds0(i) << " bounds1 " << node.getBounds1(i) << cl::sycl::endl;
	}      
      return out; 
    }

    /* =========================================================================== */
    /* ============================== BVH NODE MB =============================== */
    /* =========================================================================== */

    struct BVHNodeNMB
    {              
      NodeRef offset[BVH_NODE_N];
      float lower_t[BVH_NODE_N];
      float upper_t[BVH_NODE_N];             
      float lower_x[BVH_NODE_N]; 
      float upper_x[BVH_NODE_N]; 
      float lower_y[BVH_NODE_N]; 
      float upper_y[BVH_NODE_N]; 
      float lower_z[BVH_NODE_N]; 
      float upper_z[BVH_NODE_N];      
      float lower_dx[BVH_NODE_N]; 
      float upper_dx[BVH_NODE_N]; 
      float lower_dy[BVH_NODE_N]; 
      float upper_dy[BVH_NODE_N]; 
      float lower_dz[BVH_NODE_N]; 
      float upper_dz[BVH_NODE_N];

      __forceinline AABB3f getBounds0(size_t i) const {
	AABB3f b( float3(lower_x[i],lower_y[i],lower_z[i]),
		  float3(upper_x[i],upper_y[i],upper_z[i]) );
	return b;
      }

      __forceinline AABB3f getBounds1(size_t i) const {
	AABB3f b( float3(lower_x[i]+lower_dx[i],lower_y[i]+lower_dy[i],lower_z[i]+lower_dz[i]),
		  float3(upper_x[i]+upper_dx[i],upper_y[i]+upper_dy[i],upper_z[i]+upper_dz[i]) );
	return b;
      }
      
      __forceinline AABB3f bounds0() const {
	AABB3f b;
	b.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  b.extend(getBounds0(i));
	return b;
      }

      __forceinline AABB3f bounds1() const {
	AABB3f b;
	b.init();
	for (uint i=0;i<BVH_NODE_N;i++)
	  b.extend(getBounds1(i));	
	return b;
      }
      
      

      __forceinline AABB3f bounds_lower(size_t i) const {
	const AABB3f b0 = getBounds0(i);
	const AABB3f b1 = getBounds1(i);
	const float t = lower_t[i];
	const float3 lower = gpu::lerp<float3>(b0.lower,b1.lower,t);
	const float3 upper = gpu::lerp<float3>(b0.upper,b1.upper,t);
	return AABB3f( lower, upper );
      }

      __forceinline AABB3f bounds_upper(size_t i) const {
	const AABB3f b0 = getBounds0(i);
	const AABB3f b1 = getBounds1(i);
	const float t = upper_t[i];
	const float3 lower = gpu::lerp<float3>(b0.lower,b1.lower,t);
	const float3 upper = gpu::lerp<float3>(b0.upper,b1.upper,t);
	return AABB3f( lower, upper );
      }
      
      __forceinline void clear()
      {
	for (size_t i=0;i<BVH_NODE_N;i++)
	  {
	    offset[i] = BVH_INVALID_NODE_REF;
	    lower_t[i] = 1.0f;
	    upper_t[i] = 0.0f;	    	    
	    lower_x[i] = lower_y[i] = lower_z[i] = pos_inf;
	    upper_x[i] = upper_y[i] = upper_z[i] = neg_inf;
	    lower_dx[i] = lower_dy[i] = lower_dz[i] = pos_inf;
	    upper_dx[i] = upper_dy[i] = upper_dz[i] = neg_inf;	    
	  }
      }

      __forceinline QBVHNodeNMB convert()
      {
	QBVHNodeNMB res;
	res.clear();
	
	const AABB3f aabb0 = bounds0();
	const AABB3f aabb1 = bounds1();	
	const float3 minF0 = aabb0.lower;
	const float3 minF1 = aabb1.lower;	
	const float3 diff0 = aabb0.size()*(1.0f+2.0f*FLT_MIN);
	const float3 diff1 = aabb1.size()*(1.0f+2.0f*FLT_MIN);	
	float3 decode_scale0 = diff0 / (float3)((float)QUANT_MAX);
	float3 decode_scale1 = diff1 / (float3)((float)QUANT_MAX);

	decode_scale0 = cselect( decode_scale0 != 0.0f, decode_scale0, float3(2.0f*FLT_MIN) );
	decode_scale1 = cselect( decode_scale0 != 0.0f, decode_scale1, float3(2.0f*FLT_MIN) );
	
	float3 encode_scale0 = (float3)((float)QUANT_MAX) / diff0;
	float3 encode_scale1 = (float3)((float)QUANT_MAX) / diff1;
	
	encode_scale0 = cselect( diff0 > 0.0f, encode_scale0, float3(0.0f));
	encode_scale1 = cselect( diff1 > 0.0f, encode_scale1, float3(0.0f));
	
	res.org0 = float4(minF0.x(),minF0.y(),minF0.z(),0.0f);
	res.org1 = float4(minF1.x(),minF1.y(),minF1.z(),0.0f);
	
	res.scale0 = float4(decode_scale0.x(),decode_scale0.y(),decode_scale0.z(),0.0f);
	res.scale1 = float4(decode_scale1.x(),decode_scale1.y(),decode_scale1.z(),0.0f);
	
	for (uint i=0;i<BVH_NODE_N;i++)	  
	  {
	    if (offset[i] == BVH_INVALID_NODE_REF) continue;
	    
	    res.offset[i] = offset[i];
	    res.lower_t[i] = lower_t[i];
	    res.upper_t[i] = upper_t[i];
	      
	    const AABB3f child0 = getBounds0(i);
	    const AABB3f child1 = getBounds1(i);
	    	  
	    float3 lowerf0 = floor((child0.lower - minF0)*encode_scale0);
	    float3 lowerf1 = floor((child1.lower - minF1)*encode_scale1);
	    
	    float3 upperf0 =  ceil((child0.upper - minF0)*encode_scale0);
	    float3 upperf1 =  ceil((child1.upper - minF1)*encode_scale1);
	    
	    int3 ilower0 = max(lowerf0.convert<int,cl::sycl::rounding_mode::rtn>(),int3(QUANT_MIN));
	    int3 ilower1 = max(lowerf1.convert<int,cl::sycl::rounding_mode::rtn>(),int3(QUANT_MIN));
	    
	    int3 iupper0 = min(upperf0.convert<int,cl::sycl::rounding_mode::rtp>(),int3(QUANT_MAX));
	    int3 iupper1 = min(upperf1.convert<int,cl::sycl::rounding_mode::rtp>(),int3(QUANT_MAX));

	    /* lower/upper correction */
	    int3 m_lower_correction0 = (cl::sycl::fma(ilower0.convert<float,cl::sycl::rounding_mode::rtn>(),decode_scale0,minF0)) > child0.lower;
	    int3 m_lower_correction1 = (cl::sycl::fma(ilower1.convert<float,cl::sycl::rounding_mode::rtn>(),decode_scale1,minF1)) > child1.lower;	    
	    int3 m_upper_correction0 = (cl::sycl::fma(iupper0.convert<float,cl::sycl::rounding_mode::rtp>(),decode_scale0,minF0)) < child0.upper;
	    int3 m_upper_correction1 = (cl::sycl::fma(iupper1.convert<float,cl::sycl::rounding_mode::rtp>(),decode_scale1,minF0)) < child1.upper;
	  
	    ilower0 = max(cselect(m_lower_correction0,ilower0-1,ilower0),QUANT_MIN);
	    ilower1 = max(cselect(m_lower_correction1,ilower1-1,ilower1),QUANT_MIN);
	    
	    iupper0 = min(cselect(m_upper_correction0,iupper0+1,iupper0),QUANT_MAX);	  
	    iupper1 = min(cselect(m_upper_correction1,iupper1+1,iupper1),QUANT_MAX);	  

	    /* disable invalid lanes */	  

	    /* ilower0 = cselect(int3(m_valid),ilower0,int3(QUANT_MAX)); */
	    /* ilower1 = cselect(int3(m_valid),ilower1,int3(QUANT_MAX)); */
	    
	    /* iupper0 = cselect(int3(m_valid),iupper0,int3(QUANT_MIN)); */
	    /* iupper1 = cselect(int3(m_valid),iupper1,int3(QUANT_MIN)); */
	    	    
	    res.lower_x0[i] = ilower0.x();
	    res.lower_y0[i] = ilower0.y();
	    res.lower_z0[i] = ilower0.z();

	    res.lower_x1[i] = ilower1.x();
	    res.lower_y1[i] = ilower1.y();
	    res.lower_z1[i] = ilower1.z();
	    	    
	    res.upper_x0[i] = iupper0.x();
	    res.upper_y0[i] = iupper0.y();
	    res.upper_z0[i] = iupper0.z();

	    res.upper_x1[i] = iupper1.x();
	    res.upper_y1[i] = iupper1.y();
	    res.upper_z1[i] = iupper1.z();
	    
	  }
	
	return res;
      }
    };

    /* =============================================================================== */
    /* ============================== NODE INTERSECTION =============================== */
    /* =============================================================================== */

    struct NodeIntersectionData {
      __forceinline NodeIntersectionData(float dist, uint valid, uint offset) : dist(dist), valid(valid), offset(offset) {}      
      float dist;
      uint valid;
      uint offset;
    };

    __forceinline NodeIntersectionData intersectNode(const cl::sycl::intel::sub_group &sg,
						     const QBVHNodeN &node,
						     const Vec3i &dir_mask,
						     const Vec3f &inv_dir,
						     const Vec3f &inv_dir_org,
						     const float time,
						     const float tnear,
						     const float tfar)
    {
#if 0
      const uint subgroupLocalID = sg.get_local_id()[0];      
      const gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_base + cur);	      
      const uint  offset          = node.offset[subgroupLocalID];	      	      
      const Vec3f org      = node.org.xyz();
      const Vec3f scale    = node.scale.xyz();
#else
      cl::sycl::multi_ptr<uint,cl::sycl::access::address_space::global_space> node_ptr((uint*)&node);
      const uint2 block0 = sg.load<2,uint>(node_ptr);
      const uint offset = (uint)block0.x();
      const Vec3f org(gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 0)),
		       gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 1)),
		       gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 2)));
      const Vec3f scale(gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 4)),
			 gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 5)),
			 gpu::as_float(sg.broadcast<uint>((uint)block0.y(), 6)));	      
#endif

      cl::sycl::multi_ptr<ushort,cl::sycl::access::address_space::global_space> quant_ptr((ushort*)&node + 32); // + 64 bytes
      const ushort4 block1 = sg.load<4,ushort>(quant_ptr);
      const cl::sycl::uchar8 block2 = block1.as<cl::sycl::uchar8>();

      const uchar3 ilower(block2.s2(),block2.s4(),block2.s6());
      const uchar3 iupper(block2.s3(),block2.s5(),block2.s7());
      const float3 lowerf_  = ilower.convert<float,cl::sycl::rounding_mode::rtn>();
      const float3 upperf_  = iupper.convert<float,cl::sycl::rounding_mode::rtp>();

      const Vec3f lowerf(lowerf_.x(),lowerf_.y(),lowerf_.z());
      const Vec3f upperf(upperf_.x(),upperf_.y(),upperf_.z());
      const Vec3f _lower  = madd(lowerf,scale,org);
      const Vec3f _upper  = madd(upperf,scale,org);

      const float lower_x = cselect((uint)dir_mask.x,(float)_upper.x,(float)_lower.x);
      const float upper_x = cselect((uint)dir_mask.x,(float)_lower.x,(float)_upper.x);
      const float lower_y = cselect((uint)dir_mask.y,(float)_upper.y,(float)_lower.y);
      const float upper_y = cselect((uint)dir_mask.y,(float)_lower.y,(float)_upper.y);
      const float lower_z = cselect((uint)dir_mask.z,(float)_upper.z,(float)_lower.z);
      const float upper_z = cselect((uint)dir_mask.z,(float)_lower.z,(float)_upper.z);	     
	      
      const float lowerX = madd((float)inv_dir.x, lower_x, (float)inv_dir_org.x);
      const float upperX = madd((float)inv_dir.x, upper_x, (float)inv_dir_org.x);
      const float lowerY = madd((float)inv_dir.y, lower_y, (float)inv_dir_org.y);
      const float upperY = madd((float)inv_dir.y, upper_y, (float)inv_dir_org.y);
      const float lowerZ = madd((float)inv_dir.z, lower_z, (float)inv_dir_org.z);
      const float upperZ = madd((float)inv_dir.z, upper_z, (float)inv_dir_org.z);

      const float fnear = cl::sycl::fmax( cl::sycl::fmax(lowerX,lowerY), cl::sycl::fmax(lowerZ,tnear) );
      const float ffar  = cl::sycl::fmin( cl::sycl::fmin(upperX,upperY), cl::sycl::fmin(upperZ,tfar)  );
      const uint valid = (fnear <= ffar) & (offset != BVH_INVALID_NODE_REF); //((uchar)ilower.x() <= (uchar)iupper.x());	       
      return NodeIntersectionData(fnear, valid, offset);
    }


#if 0    
    __forceinline NodeIntersectionData intersectNode(const cl::sycl::intel::sub_group &sg,
						     const QBVHNodeNMB &node,
						     const Vec3i &dir_mask,
						     const Vec3f &inv_dir,
						     const Vec3f &inv_dir_org,
						     const float time,
						     const float tnear,
						     const float tfar)
    {
      const uint subgroupLocalID = sg.get_local_id()[0];      
      const uint  offset = node.offset[subgroupLocalID];
      
      const float time0 = node.lower_t[subgroupLocalID];
      const float time1 = node.upper_t[subgroupLocalID];
      
      const Vec3f lower0(node.lower_x[subgroupLocalID],node.lower_y[subgroupLocalID],node.lower_z[subgroupLocalID]);
      const Vec3f upper0(node.upper_x[subgroupLocalID],node.upper_y[subgroupLocalID],node.upper_z[subgroupLocalID]);
      const Vec3f lower1(node.lower_dx[subgroupLocalID],node.lower_dy[subgroupLocalID],node.lower_dz[subgroupLocalID]);
      const Vec3f upper1(node.upper_dx[subgroupLocalID],node.upper_dy[subgroupLocalID],node.upper_dz[subgroupLocalID]);
            
      const float lower0_x = cselect((uint)dir_mask.x,(float)upper0.x,(float)lower0.x);
      const float upper0_x = cselect((uint)dir_mask.x,(float)lower0.x,(float)upper0.x);
      const float lower1_x = cselect((uint)dir_mask.x,(float)upper1.x,(float)lower1.x);
      const float upper1_x = cselect((uint)dir_mask.x,(float)lower1.x,(float)upper1.x);
      
      const float lower0_y = cselect((uint)dir_mask.y,(float)upper0.y,(float)lower0.y);
      const float upper0_y = cselect((uint)dir_mask.y,(float)lower0.y,(float)upper0.y);
      const float lower1_y = cselect((uint)dir_mask.y,(float)upper1.y,(float)lower1.y);
      const float upper1_y = cselect((uint)dir_mask.y,(float)lower1.y,(float)upper1.y);
      
      const float lower0_z = cselect((uint)dir_mask.z,(float)upper0.z,(float)lower0.z);
      const float upper0_z = cselect((uint)dir_mask.z,(float)lower0.z,(float)upper0.z);
      const float lower1_z = cselect((uint)dir_mask.z,(float)upper1.z,(float)lower1.z);
      const float upper1_z = cselect((uint)dir_mask.z,(float)lower1.z,(float)upper1.z);

      const float lower_x = madd(lower1_x,time,lower0_x);
      const float upper_x = madd(upper1_x,time,upper0_x);
      const float lower_y = madd(lower1_y,time,lower0_y);
      const float upper_y = madd(upper1_y,time,upper0_y);
      const float lower_z = madd(lower1_z,time,lower0_z);
      const float upper_z = madd(upper1_z,time,upper0_z);      
	      
      const float lowerX = madd((float)inv_dir.x, lower_x, (float)inv_dir_org.x);
      const float upperX = madd((float)inv_dir.x, upper_x, (float)inv_dir_org.x);
      const float lowerY = madd((float)inv_dir.y, lower_y, (float)inv_dir_org.y);
      const float upperY = madd((float)inv_dir.y, upper_y, (float)inv_dir_org.y);
      const float lowerZ = madd((float)inv_dir.z, lower_z, (float)inv_dir_org.z);
      const float upperZ = madd((float)inv_dir.z, upper_z, (float)inv_dir_org.z);

      const float fnear = cl::sycl::fmax( cl::sycl::fmax(lowerX,lowerY), cl::sycl::fmax(lowerZ,tnear) );
      const float ffar  = cl::sycl::fmin( cl::sycl::fmin(upperX,upperY), cl::sycl::fmin(upperZ,tfar)  );
      const uint valid = (fnear <= ffar) & (offset != BVH_INVALID_NODE_REF) & (time0 <= time) & (time < time1);
      
      return NodeIntersectionData(fnear, valid, offset);
    }
#else

    __forceinline NodeIntersectionData intersectNode(const cl::sycl::intel::sub_group &sg,
						     const QBVHNodeNMB &node,
						     const Vec3i &dir_mask,
						     const Vec3f &inv_dir,
						     const Vec3f &inv_dir_org,
						     const float time,
						     const float tnear,
						     const float tfar)
    {
      const uint subgroupLocalID = sg.get_local_id()[0];      
      const uint  offset = node.offset[subgroupLocalID];
      
      const float time0 = node.lower_t[subgroupLocalID];
      const float time1 = node.upper_t[subgroupLocalID];

      const Vec3f org0(node.org0.x(),node.org0.y(),node.org0.z());
      const Vec3f org1(node.org1.x(),node.org1.y(),node.org1.z());      
      const Vec3f scale0(node.scale0.x(),node.scale0.y(),node.scale0.z());
      const Vec3f scale1(node.scale1.x(),node.scale1.y(),node.scale1.z());      
      

      const uchar3 ilower0(node.lower_x0[subgroupLocalID],node.lower_y0[subgroupLocalID],node.lower_z0[subgroupLocalID]);
      const uchar3 ilower1(node.lower_x1[subgroupLocalID],node.lower_y1[subgroupLocalID],node.lower_z1[subgroupLocalID]);

      const uchar3 iupper0(node.upper_x0[subgroupLocalID],node.upper_y0[subgroupLocalID],node.upper_z0[subgroupLocalID]);
      const uchar3 iupper1(node.upper_x1[subgroupLocalID],node.upper_y1[subgroupLocalID],node.upper_z1[subgroupLocalID]);


      const float3 lowerf0_  = ilower0.convert<float,cl::sycl::rounding_mode::rtn>();
      const float3 lowerf1_  = ilower1.convert<float,cl::sycl::rounding_mode::rtn>();      
      const float3 upperf0_  = iupper0.convert<float,cl::sycl::rounding_mode::rtp>();
      const float3 upperf1_  = iupper1.convert<float,cl::sycl::rounding_mode::rtp>();

      const Vec3f lowerf0(lowerf0_.x(),lowerf0_.y(),lowerf0_.z());
      const Vec3f lowerf1(lowerf1_.x(),lowerf1_.y(),lowerf1_.z());      
      const Vec3f upperf0(upperf0_.x(),upperf0_.y(),upperf0_.z());
      const Vec3f upperf1(upperf1_.x(),upperf1_.y(),upperf1_.z());
      
      const Vec3f lower0  = madd(lowerf0,scale0,org0);
      const Vec3f lower1  = madd(lowerf1,scale1,org1);
      
      const Vec3f upper0  = madd(upperf0,scale0,org0);
      const Vec3f upper1  = madd(upperf1,scale1,org1);
                            
      const float lower0_x = cselect((uint)dir_mask.x,(float)upper0.x,(float)lower0.x);
      const float upper0_x = cselect((uint)dir_mask.x,(float)lower0.x,(float)upper0.x);
      const float lower1_x = cselect((uint)dir_mask.x,(float)upper1.x,(float)lower1.x);
      const float upper1_x = cselect((uint)dir_mask.x,(float)lower1.x,(float)upper1.x);
      
      const float lower0_y = cselect((uint)dir_mask.y,(float)upper0.y,(float)lower0.y);
      const float upper0_y = cselect((uint)dir_mask.y,(float)lower0.y,(float)upper0.y);
      const float lower1_y = cselect((uint)dir_mask.y,(float)upper1.y,(float)lower1.y);
      const float upper1_y = cselect((uint)dir_mask.y,(float)lower1.y,(float)upper1.y);
      
      const float lower0_z = cselect((uint)dir_mask.z,(float)upper0.z,(float)lower0.z);
      const float upper0_z = cselect((uint)dir_mask.z,(float)lower0.z,(float)upper0.z);
      const float lower1_z = cselect((uint)dir_mask.z,(float)upper1.z,(float)lower1.z);
      const float upper1_z = cselect((uint)dir_mask.z,(float)lower1.z,(float)upper1.z);

      const float lower_x = madd(lower1_x,time,lower0_x);
      const float upper_x = madd(upper1_x,time,upper0_x);
      const float lower_y = madd(lower1_y,time,lower0_y);
      const float upper_y = madd(upper1_y,time,upper0_y);
      const float lower_z = madd(lower1_z,time,lower0_z);
      const float upper_z = madd(upper1_z,time,upper0_z);      
	      
      const float lowerX = madd((float)inv_dir.x, lower_x, (float)inv_dir_org.x);
      const float upperX = madd((float)inv_dir.x, upper_x, (float)inv_dir_org.x);
      const float lowerY = madd((float)inv_dir.y, lower_y, (float)inv_dir_org.y);
      const float upperY = madd((float)inv_dir.y, upper_y, (float)inv_dir_org.y);
      const float lowerZ = madd((float)inv_dir.z, lower_z, (float)inv_dir_org.z);
      const float upperZ = madd((float)inv_dir.z, upper_z, (float)inv_dir_org.z);

      const float fnear = cl::sycl::fmax( cl::sycl::fmax(lowerX,lowerY), cl::sycl::fmax(lowerZ,tnear) );
      const float ffar  = cl::sycl::fmin( cl::sycl::fmin(upperX,upperY), cl::sycl::fmin(upperZ,tfar)  );
      const uint valid = (fnear <= ffar) & (offset != BVH_INVALID_NODE_REF) & (time0 <= time) & (time < time1);
      
      return NodeIntersectionData(fnear, valid, offset);
    }
    
#endif
    
    
    
  };
};

#endif
