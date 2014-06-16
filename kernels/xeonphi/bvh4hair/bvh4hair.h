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

#pragma once

#include "common/alloc.h"
#include "common/accel.h"
#include "common/scene.h"
#include "geometry/primitive.h"

namespace embree
{


  class BVH4Hair : public Bounded 
  {
  public:
    /*! branching width of the tree */
    static const size_t N = 4;

    /*! Masks the bits that store the number of items per leaf. */
    static const unsigned int encodingBits     = 4;
    static const unsigned int offset_mask      = ((unsigned int)-1) << encodingBits;
    static const unsigned int leaf_shift       = 3;
    static const unsigned int leaf_mask        = 1<<leaf_shift;  
    static const unsigned int items_mask       = leaf_mask-1;  
    static const unsigned int alignednode_mask = 1 << (leaf_shift+1);

    
    /*! Maximal depth of the BVH. */
    static const size_t maxBuildDepth = 26;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+6;
    static const size_t maxDepth = maxBuildDepth + maxBuildDepthLeaf;

    /*! Empty node */
    static const unsigned int emptyNode = leaf_mask;

    /*! Invalid node */
    static const unsigned int invalidNode = (unsigned int)-1; //0;

    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (unsigned id) : _id(id) { }

      /*! Cast to unsigned */
      __forceinline operator unsigned int() const { return _id; }
     
      /*! checks if this is a leaf */
      __forceinline unsigned int isLeaf() const { return _id & leaf_mask; }

      /*! checks if this is a leaf */
      __forceinline unsigned int isLeaf(const unsigned int mask) const { return _id & mask; }
      
      /*! checks if this is a node */
      __forceinline unsigned isNode() const { return (_id & leaf_mask) == 0; }
      
      /*! returns node pointer */

      __forceinline       void* node(      void* base) const { return (      void*)((      char*)base + (size_t)_id); }
      __forceinline const void* node(const void* base) const { return (const void*)((const char*)base + (size_t)_id); }
      
      
      /*! returns leaf pointer */
      template<unsigned int scale=4>
	__forceinline const char* leaf(const void* base, unsigned int& num) const {
        assert(isLeaf());
        num = _id & items_mask;
        return (const char*)base + (_id & offset_mask)*scale;
      }

      /*! returns leaf pointer */
	template<unsigned int scale=4>
	__forceinline const char* leaf(const void* base) const {
	  assert(isLeaf());
	  return (const char*)base + (_id & offset_mask)*scale;
	}

	  __forceinline unsigned int offset() const {
	    return _id & offset_mask;
	  }

      __forceinline unsigned int offsetIndex() const {
        return _id >> encodingBits;
      }

      __forceinline unsigned int items() const {
        return _id & items_mask;
      }
      
      __forceinline unsigned int &id() { return _id; }
    private:
      unsigned int _id;
    };

#if 0
    struct __aligned(64) UnalignedNode
    {
      mic_f matrixRowXYZW[3];

      NodeRef children[16];

      static float identityMatrix[16];
      static float  invalidMatrix[16];

      template<int PFHINT>
	__forceinline void prefetchNode() const
	{
	  prefetch<PFHINT>((char*)this + 0 * 64);
	  prefetch<PFHINT>((char*)this + 1 * 64);
	  prefetch<PFHINT>((char*)this + 2 * 64);
	  prefetch<PFHINT>((char*)this + 3 * 64);
	}

      __forceinline mic_f getRow(size_t i) const
      {
	return matrixRowXYZW[i];
      }

      __forceinline mic_i getChildren() const
      {
	return load16i((int*)children);
      }

      __forceinline void setInvalid()
      {
	const mic_f c0 = broadcast4to16f(&invalidMatrix[ 0]);
	const mic_f c1 = broadcast4to16f(&invalidMatrix[ 4]);
	const mic_f c2 = broadcast4to16f(&invalidMatrix[ 8]);
	
	matrixRowXYZW[0] = c0;
	matrixRowXYZW[1] = c1;
	matrixRowXYZW[2] = c2;

	for (size_t i=0;i<16;i++)
	  children[i] = BVH4Hair::invalidNode;
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
	matrix(0,0,m) = space.l.vx.x;
	matrix(1,0,m) = space.l.vx.y;
	matrix(2,0,m) = space.l.vx.z;

	matrix(0,1,m) = space.l.vy.x;
	matrix(1,1,m) = space.l.vy.y;
	matrix(2,1,m) = space.l.vy.z;

	matrix(0,2,m) = space.l.vz.x;
	matrix(1,2,m) = space.l.vz.y;
	matrix(2,2,m) = space.l.vz.z;

	matrix(3,0,m) = space.p.x;
	matrix(3,1,m) = space.p.y;
	matrix(3,2,m) = space.p.z;

      }

      __forceinline void createNode(unsigned int i, const size_t m)
      {
	child(m) = i;
      }


      __forceinline void createLeaf(unsigned int offset, 
				    unsigned int items,
				    const size_t m)
      {
	assert(items <= BVH4Hair::N);
	child(m) = (offset << encodingBits) | BVH4Hair::leaf_mask | items;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return children[3+4*i]; }
      __forceinline const NodeRef& child(size_t i) const { return children[3+4*i]; }

      __forceinline       NodeRef& child_ref(size_t i)       { return children[i]; }
      __forceinline const NodeRef& child_ref(size_t i) const { return children[i]; }
     
    };
#else

    struct __aligned(64) UnalignedNode
    {

      static float identityMatrix[16];
      static float  invalidMatrix[16];

      struct NodeStruct {
        float x,y,z;           // x,y, and z coordinates of bounds
        NodeRef data;         
      } lower[4], upper[4];    

      char xfm[4][16];

      __forceinline mic_i getChildren() const
      {
	return load16i((int*)lower);
      }

      __forceinline mic_f getRow(size_t i) const
      {
	return load16f_int8(xfm[i]);
      }

      __forceinline void setInvalid(size_t i) 
      {
	lower[i].x = pos_inf;
	lower[i].y = pos_inf;
	lower[i].z = pos_inf;
	lower[i].data = invalidNode;

	upper[i].x = neg_inf;
	upper[i].y = neg_inf;
	upper[i].z = neg_inf;
	upper[i].data = invalidNode;
		
      }

      __forceinline void setInvalid() 
      {
	for (size_t i=0;i<4;i++)
	  setInvalid(i);

	for (size_t y=0;y<4;y++)
	  for (size_t x=0;x<16;x++)
	    xfm[y][x] = 0;
      }
      
      template<int PFHINT>
	__forceinline void prefetchNode() const
	{
	  prefetch<PFHINT>((char*)this + 0 * 64);
	  prefetch<PFHINT>((char*)this + 1 * 64);
	  prefetch<PFHINT>((char*)this + 2 * 64);
	}


      __forceinline const char &c_matrix(const size_t row,
					 const size_t column,
					 const size_t matrixID) const
      {
	assert(matrixID < 4);
	assert(row < 3);
	assert(column < 3);
	return xfm[row][matrixID*4+column];
      } 

      __forceinline char &c_matrix(const size_t row,
				   const size_t column,
				   const size_t matrixID) 
      {
	assert(matrixID < 4);
	assert(row < 3);
	assert(column < 3);
	return xfm[row][matrixID*4+column];
      } 

      __forceinline float matrix(const size_t row,
				 const size_t column,
				 const size_t matrixID) const
      {
	assert(matrixID < 4);
	assert(row < 3);
	assert(column < 3);
	return ((float)c_matrix(row,column,matrixID)) * 1.0f/127.0f;
      } 

      

      __forceinline void setMatrix(const BBox3fa &b, const size_t m)
      {
	lower[m].x = b.lower.x;
	lower[m].y = b.lower.y;
	lower[m].z = b.lower.z;

	upper[m].x = b.upper.x;
	upper[m].y = b.upper.y;
	upper[m].z = b.upper.z;

	c_matrix(0,0,m) = 127;
	c_matrix(1,1,m) = 127;
	c_matrix(2,2,m) = 127;
      }
   
      __forceinline void setMatrix(const LinearSpace3fa &mat, BBox3fa &b, const size_t m)
      {
	lower[m].x = b.lower.x;
	lower[m].y = b.lower.y;
	lower[m].z = b.lower.z;

	upper[m].x = b.upper.x;
	upper[m].y = b.upper.y;
	upper[m].z = b.upper.z;

#if 0
	for (size_t i=0;i<3;i++)
	  {
	    c_matrix(i,0,m) = (char)(127.0f * mat.vx[i]);
	    c_matrix(i,1,m) = (char)(127.0f * mat.vy[i]);
	    c_matrix(i,2,m) = (char)(127.0f * mat.vz[i]);
	  }
#else
	for (size_t i=0;i<3;i++)
	  {
	    c_matrix(0,i,m) = (char)(127.0f * mat.vx[i]);
	    c_matrix(1,i,m) = (char)(127.0f * mat.vy[i]);
	    c_matrix(2,i,m) = (char)(127.0f * mat.vz[i]);
	  }

#endif
	

      }

      __forceinline void createNode(unsigned int i, const size_t m)
      {
	child(m) = i;
      }


      __forceinline void createLeaf(unsigned int offset, 
				    unsigned int items,
				    const size_t m)
      {
	assert(items <= BVH4Hair::N);
	child(m) = (offset << encodingBits) | BVH4Hair::leaf_mask | items;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { return lower[i].data; }
      __forceinline const NodeRef& child(size_t i) const { return lower[i].data; }

      __forceinline       NodeRef& child_ref(size_t i)       { return ((BVH4Hair::NodeRef*)lower)[i]; }
      __forceinline const NodeRef& child_ref(size_t i) const { return ((BVH4Hair::NodeRef*)lower)[i]; }
     
    };

    
#endif

    struct __aligned(64) AlignedNode
    {
    public:
      struct NodeStruct {
        float x,y,z;           // x,y, and z coordinates of bounds
        NodeRef data;          
      } lower[4], upper[4];    

      template<int PFHINT>
	__forceinline void prefetchNode() const
	{
	  prefetch<PFHINT>((char*)this + 0 * 64);
	  prefetch<PFHINT>((char*)this + 1 * 64);
	}

      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const {
	assert( i < 4 );
        Vec3fa l = *(Vec3fa*)&lower[i];
        Vec3fa u = *(Vec3fa*)&upper[i];
        return BBox3fa(l,u);
      }

      __forceinline mic_f lowerXYZ(size_t i) const {
	return broadcast4to16f(&lower[i]);
      }

      __forceinline mic_f upperXYZ(size_t i) const {
	return broadcast4to16f(&upper[i]);
      }

      __forceinline mic_i getChildren() const
      {
	return load16i((int*)lower);
      }

      __forceinline void setInvalid(size_t i)
      {
	lower[i].x = pos_inf;
	lower[i].y = pos_inf;
	lower[i].z = pos_inf;
	lower[i].data = (unsigned int)BVH4Hair::invalidNode;

	upper[i].x = neg_inf;
	upper[i].y = neg_inf;
	upper[i].z = neg_inf;
	upper[i].data = (unsigned int)BVH4Hair::invalidNode;
      }

      __forceinline void setInvalid()
      {
	for (size_t i=0;i<4;i++)
	  setInvalid(i);
      }


      __forceinline       NodeRef &child(size_t i)       { 
	return  lower[i].data;
      }
      __forceinline const NodeRef &child(size_t i) const { 
	return  lower[i].data;
      }

      __forceinline       NodeRef& child_ref(size_t i)       { return ((NodeRef*)lower)[i]; }
      __forceinline const NodeRef& child_ref(size_t i) const { return ((NodeRef*)lower)[i]; }
      
      __forceinline void setMatrix(const BBox3fa &b, const size_t m)
      {
	lower[m].x = b.lower.x;
	lower[m].y = b.lower.y;
	lower[m].z = b.lower.z;

	upper[m].x = b.upper.x;
	upper[m].y = b.upper.y;
	upper[m].z = b.upper.z;

	lower[m].data = 0;
	upper[m].data = 0;

      }

      __forceinline void setMatrix(const LinearSpace3fa &mat, BBox3fa &b, const size_t m)
      {
	FATAL("not implemented");
      }

    };

    NodeRef root;                      //!< Root node (can also be a leaf).

    const PrimitiveType& primTy;   //!< triangle type stored in BVH
    void* geometry;                    //!< pointer to geometry for intersection
    UnalignedNode *unaligned_nodes;
    void *accel;
    size_t size_node;
    size_t size_accel;

    __forceinline       void* nodePtr()       { return unaligned_nodes; }
    __forceinline const void* nodePtr() const { return unaligned_nodes; }

    __forceinline       void* triPtr()       { return accel; }
    __forceinline const void* triPtr() const { return accel; }


  BVH4Hair(const PrimitiveType& primTy, void* geometry = NULL) : primTy(primTy), 
      geometry(geometry), 
      root(emptyNode), 
      accel(NULL),
      size_node(0),
      size_accel(0)
      {	
	assert( sizeof(UnalignedNode) == 256 );
	unaligned_nodes = NULL;
      }

    
    static Accel* BVH4HairBinnedSAH(Scene* scene);
    


  };

  __forceinline std::ostream& operator<<(std::ostream &o, const BVH4Hair::AlignedNode &n)
    {
      for (size_t i=0;i<4;i++)
	{
	  o << "lower: [" << n.lower[i].x << "," << n.lower[i].y << "," << n.lower[i].z << "," << n.lower[i].data << "] ";
	  o << "upper: [" << n.upper[i].x << "," << n.upper[i].y << "," << n.upper[i].z << "," << n.upper[i].data << "] ";
	  o << std::endl;
	  }
      return o;
    }


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
	  o << n.child(m) << " ";
	  if (n.child(m).isLeaf())
	    o << "(LEAF: index " << n.child(m).offsetIndex() << " items " << n.child(m).items() << ") ";
	  else
	    o << "(NODE) ";
	}
      o << std::endl;

      return o;
    } 

};
