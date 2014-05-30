// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "common/alloc.h"
#include "common/accel.h"
#include "common/scene.h"
#include "geometry/primitive.h"
#include "bvh4i/bvh4i.h"

namespace embree
{


  class BVH4Hair : public BVH4i
  {
  public:
    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t encodingBits = 4;
    static const size_t offset_mask  = ((size_t)-1) << encodingBits;
    static const size_t leaf_shift   = 3;
    static const size_t leaf_mask    = 1<<leaf_shift;  
    static const size_t items_mask   = leaf_mask-1;  
    
    /*! Empty node */
    static const size_t emptyNode = leaf_mask;

    /*! Invalid node */
    static const size_t invalidNode = (size_t)-1;

    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 32;


    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t id) : _id(id) { }

      /*! Cast to size_t*/
      __forceinline operator size_t() const { return _id; }
     
      /*! checks if this is a leaf */
      __forceinline size_t isLeaf() const { return _id & leaf_mask; }

      /*! checks if this is a leaf */
      __forceinline size_t isLeaf(const size_t mask) const { return _id & mask; }
      
      /*! checks if this is a node */
      __forceinline size_t isNode() const { return (_id & leaf_mask) == 0; }
      
      /*! returns node pointer */
      __forceinline       void* node() const      { assert(isNode()); return (void*)((size_t)_id); }

      __forceinline       void* ptr() const      { return (void*)_id; }

      
      /*! returns leaf pointer */
      __forceinline char* leaf(size_t& num) const {
        assert(isLeaf());
        num = (_id & (size_t)items_mask);
        return (char*)(_id & offset_mask);
      }


      __forceinline size_t offset() const {
        return _id & offset_mask;
      }

      __forceinline size_t items() const {
        return _id & items_mask;
      }

      __forceinline size_t offsetIndex() const {
        return _id >> encodingBits;
      }

      
    private:
      size_t _id;
    };

    struct __aligned(256) UnalignedNode
    {
      mic_f matrixRowXYZW[3];

      NodeRef children[4];
      unsigned int geomID[4];
      unsigned int primID[4];

      static float identityMatrix[16];
      static float  invalidMatrix[16];

      template<int PFHINT>
	__forceinline void prefetchNode()
	{
	  prefetch<PFHINT>((char*)this + 0 * 64);
	  prefetch<PFHINT>((char*)this + 1 * 64);
	  prefetch<PFHINT>((char*)this + 2 * 64);
	  prefetch<PFHINT>((char*)this + 3 * 64);
	}

      __forceinline void setInvalid()
      {
	const mic_f c0 = broadcast4to16f(&invalidMatrix[ 0]);
	const mic_f c1 = broadcast4to16f(&invalidMatrix[ 4]);
	const mic_f c2 = broadcast4to16f(&invalidMatrix[ 8]);
	
	matrixRowXYZW[0] = c0;
	matrixRowXYZW[1] = c1;
	matrixRowXYZW[2] = c2;

	children[0] = children[1] = children[2] = children[3] = NULL;
	geomID[0] = geomID[1] = geomID[2] = geomID[3] = (unsigned int)-1;
	primID[0] = primID[1] = primID[2] = primID[3] = (unsigned int)-1;
      }

      __forceinline void setIdentityMatrix() 
      {
	const mic_f c0 = broadcast4to16f(&identityMatrix[ 0]);
	const mic_f c1 = broadcast4to16f(&identityMatrix[ 4]);
	const mic_f c2 = broadcast4to16f(&identityMatrix[ 8]);
	
	matrixRowXYZW[0] = c0;
	matrixRowXYZW[1] = c1;
	matrixRowXYZW[2] = c2;
      }

      __forceinline float &matrix(const size_t row,
				  const size_t column,
				  const size_t matrixID) 
      {
	assert(matrixID < 4);
	assert(row < 4);
	assert(column < 3);
	return matrixRowXYZW[column][4*matrixID+row];
      } 

      __forceinline const float &matrix(const size_t row,
					const size_t column,
					const size_t matrixID) const
      {
	assert(matrixID < 4);
	assert(row < 4);
	assert(column < 3);
	return matrixRowXYZW[column][4*matrixID+row];
      } 

      __forceinline void set_scale(const float sx, 
				   const float sy,
				   const float sz,
				   const size_t matrixID) 
      {
	matrix(0,0,matrixID) = sx;
	matrix(1,1,matrixID) = sy;
	matrix(2,2,matrixID) = sz;
      } 

      __forceinline void set_translation(const float tx, 
					 const float ty,
					 const float tz,
					 const size_t matrixID) 
      {
	matrix(3,0,matrixID) = tx;
	matrix(3,1,matrixID) = ty;
	matrix(3,2,matrixID) = tz;
      } 

      __forceinline void setMatrix(const BBox3fa &b, const size_t m)
      {
	const float dx = b.upper.x - b.lower.x;
	const float dy = b.upper.y - b.lower.y;
	const float dz = b.upper.z - b.lower.z;
	const float inv_dx = 1.0f / dx;
	const float inv_dy = 1.0f / dy;
	const float inv_dz = 1.0f / dz;
	const float min_x = b.lower.x;
	const float min_y = b.lower.y;
	const float min_z = b.lower.z;
	set_scale(inv_dx,inv_dy,inv_dz,m);
	set_translation(-min_x*inv_dx,-min_y*inv_dy,-min_z*inv_dz,m);	
      }

      __forceinline AffineSpace3fa getAffineSpace3fa(const LinearSpace3fa &mat, BBox3fa &b)
      {
	AffineSpace3fa scale = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),b.upper-b.lower));
	AffineSpace3fa trans = AffineSpace3fa::translate(-b.lower);
	return  scale * trans * AffineSpace3fa(mat);
      }
   
      __forceinline void setMatrix(const LinearSpace3fa &mat, BBox3fa &b, const size_t m)
      {

	AffineSpace3fa space = getAffineSpace3fa(mat,b);
#if 0
	matrix(0,0,m) = space.l.vx.x;
	matrix(1,0,m) = space.l.vy.x;
	matrix(2,0,m) = space.l.vz.x;

	matrix(0,1,m) = space.l.vx.y;
	matrix(1,1,m) = space.l.vy.y;
	matrix(2,1,m) = space.l.vz.y;

	matrix(0,2,m) = space.l.vx.z;
	matrix(1,2,m) = space.l.vy.z;
	matrix(2,2,m) = space.l.vz.z;
#else

	matrix(0,0,m) = space.l.vx.x;
	matrix(1,0,m) = space.l.vx.y;
	matrix(2,0,m) = space.l.vx.z;

	matrix(0,1,m) = space.l.vy.x;
	matrix(1,1,m) = space.l.vy.y;
	matrix(2,1,m) = space.l.vy.z;

	matrix(0,2,m) = space.l.vz.x;
	matrix(1,2,m) = space.l.vz.y;
	matrix(2,2,m) = space.l.vz.z;

#endif
	matrix(3,0,m) = space.p.x;
	matrix(3,1,m) = space.p.y;
	matrix(3,2,m) = space.p.z;

      }

      __forceinline void createNode(UnalignedNode *b, const size_t m)
      {
	child(m) = (size_t)b;
      }


      __forceinline void createLeaf(unsigned int offset, 
				    unsigned int items,
				    const size_t m)
      {
	assert(items <= BVH4Hair::N);
	child(m) = ((size_t)offset << encodingBits) | BVH4Hair::leaf_mask | items;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[i]; }

      void convertFromBVH4iNode(const BVH4i::Node &node, UnalignedNode *ptr);
     
    };

  BVH4Hair(const PrimitiveType& primTy, void* geometry = NULL) : BVH4i(primTy,geometry)
      {	
	assert( sizeof(UnalignedNode) == 256 );
	unaligned_nodes = NULL;
      }
    
    static Accel* BVH4HairBinnedSAH(Scene* scene);
    

    UnalignedNode *unaligned_nodes;

  };


  __forceinline std::ostream &operator<<(std::ostream &o, const BVH4Hair::UnalignedNode &n)
    {
      o << "ptr " << (void*)&n << std::endl;
      for (size_t m=0;m<4;m++)
	{
	  o << "matrix " << m << ": " << std::endl;
	  for (size_t y=0;y<4;y++)
	    {
	      for (size_t x=0;x<3;x++)
		o << n.matrix(y,x,m) << " ";
	      o << std::endl;
	    }
	}
      o << "children: ";
      for (size_t m=0;m<4;m++)
	{
	  o << n.child(m).ptr() << " ";
	  if (n.child(m).isLeaf())
	    o << "(LEAF: index " << n.child(m).offsetIndex() << " items " << n.child(m).items() << ") ";
	  else
	    o << "(NODE) ";
	}
      o << std::endl;

      o << "geomID: ";
      for (size_t m=0;m<4;m++)
	o << n.geomID[m] << " ";
      o << std::endl;

      o << "primID: ";
      for (size_t m=0;m<4;m++)
	o << n.primID[m] << " ";
      o << std::endl;
      

      return o;
    } 

};
