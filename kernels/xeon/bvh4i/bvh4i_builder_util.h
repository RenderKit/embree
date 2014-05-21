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

#include "common/primref.h"
#include "geometry/triangle1.h"
#include "builders/workstack.h"

namespace embree
{
  /* --------------------------------------------------------------- */
  /* --------------------- Centroid_Scene_AABB --------------------- */
  /* --------------------------------------------------------------- */

  class __aligned(64) Centroid_Scene_AABB
  {
    ALIGNED_CLASS;
  public:
    BBox3fa centroid2;
    BBox3fa geometry;

    __forceinline Centroid_Scene_AABB () {}

    __forceinline Centroid_Scene_AABB (const BBox3fa& geom, const BBox3fa& cent)
      : geometry(geom), centroid2(cent) {}

    __forceinline void reset() {
      centroid2 = geometry = empty;
    }

    __forceinline void extend(const BBox3fa& b) {
      assert(!b.empty());
      centroid2.extend(center2(b));
      geometry.extend(b);
    }

    __forceinline void extend(const Centroid_Scene_AABB& v) {
      centroid2.extend(v.centroid2);
      geometry.extend(v.geometry);
    }

    __forceinline void extend_atomic(const Centroid_Scene_AABB& v)
    {
      float *aabb = (float*)this;
      float *v_aabb = (float*)&v;

      atomic_min_f32(&aabb[0] ,v_aabb[0]);
      atomic_min_f32(&aabb[1] ,v_aabb[1]);
      atomic_min_f32(&aabb[2] ,v_aabb[2]);

      atomic_max_f32(&aabb[4] ,v_aabb[4]);
      atomic_max_f32(&aabb[5] ,v_aabb[5]);
      atomic_max_f32(&aabb[6] ,v_aabb[6]);

      atomic_min_f32(&aabb[8] ,v_aabb[8]);
      atomic_min_f32(&aabb[9] ,v_aabb[9]);
      atomic_min_f32(&aabb[10],v_aabb[10]);

      atomic_max_f32(&aabb[12],v_aabb[12]);
      atomic_max_f32(&aabb[13],v_aabb[13]);
      atomic_max_f32(&aabb[14],v_aabb[14]);
    }


    __forceinline void extend_centroid_bounds_atomic(const Centroid_Scene_AABB& v)
    {
      float *aabb = (float*)&centroid2;
      float *v_aabb = (float*)&v.centroid2;

      atomic_min_f32(&aabb[0] ,v_aabb[0]);
      atomic_min_f32(&aabb[1] ,v_aabb[1]);
      atomic_min_f32(&aabb[2] ,v_aabb[2]);

      atomic_max_f32(&aabb[4] ,v_aabb[4]);
      atomic_max_f32(&aabb[5] ,v_aabb[5]);
      atomic_max_f32(&aabb[6] ,v_aabb[6]);
    }

    __forceinline friend std::ostream &operator<<(std::ostream &o, const Centroid_Scene_AABB &cs)
    {
      o << "centroid2 = " << cs.centroid2 << " ";
      o << "geometry = " << cs.geometry << " ";
      return o;
    };

  };

  /* --------------------------------------------------------------- */
  /* ------------------------- BuildRecord ------------------------- */
  /* --------------------------------------------------------------- */

  class __aligned(64) BuildRecord 
  {
  public:
    Centroid_Scene_AABB bounds; //!< geometry and centroid bounds

    unsigned int begin;         //!< start of range
    unsigned int end;           //!< end of range
    unsigned int parentID;      //!< the ID of the node that points to us
    unsigned int depth;         //!< depth from the root of the tree

    unsigned int flags;
    float sArea;
    size_t parentNode; 

    BuildRecord()
    {
	assert(sizeof(BuildRecord) == 128);
      }

    __forceinline void init(const unsigned int _begin, const unsigned int _end)
    {
      begin  = _begin;
      end    = _end;
      parentID = (unsigned int)-1;
      sArea = area(bounds.geometry);
      flags = BUILD_RECORD_NODE;
    }

    __forceinline void init(const Centroid_Scene_AABB& _bounds, const unsigned int _begin, const unsigned int _end)
    {
      bounds = _bounds;
      init(_begin,_end);
    }

    __forceinline unsigned int items() const {
      return end - begin;
    }

    __forceinline float sceneArea() {
      return sArea;
    }

    __forceinline bool operator<(const BuildRecord &br) const { return items() < br.items(); } 
    __forceinline bool operator>(const BuildRecord &br) const { return items() > br.items(); } 

    __forceinline friend std::ostream &operator<<(std::ostream &o, const BuildRecord &br)
    {
      o << "centroid2 = " << br.bounds.centroid2 << " ";
      o << "geometry = " << br.bounds.geometry << " ";
      o << "begin      " << br.begin << " ";
      o << "end        " << br.end << " ";
      o << "items      " << br.end-br.begin << " ";
      o << "parentID     " << br.parentID << " ";
      o << "flags      " << br.flags << " ";
      o << "sArea      " << br.sArea << " ";
      return o;
    };

    /* FIXME: this can be removed */
    enum { BUILD_RECORD_INIT  = 0, BUILD_RECORD_NODE  = 1, BUILD_RECORD_LEAF  = 2 };
    __forceinline void createNode() { flags = BUILD_RECORD_NODE; }
    __forceinline void createLeaf() { flags = BUILD_RECORD_LEAF; }
    __forceinline bool isLeaf() { return flags == BUILD_RECORD_LEAF; }
  };

  template<class T>
    __forceinline void insertionsort_ascending(T *__restrict__ array, const size_t length)
  {
    for(size_t i = 1;i<length;++i)
    {
      T v = array[i];
      size_t j = i;
      while(j > 0 && v < array[j-1])
      {
        array[j] = array[j-1];
        --j;
      }
      array[j] = v;
    }
  }
  
  template<class T>
    __forceinline void insertionsort_decending(T *__restrict__ array, const size_t length)
  {
    for(size_t i = 1;i<length;++i)
    {
      T v = array[i];
      size_t j = i;
      while(j > 0 && v > array[j-1])
      {
        array[j] = array[j-1];
        --j;
      }
      array[j] = v;
    }
  }
  
  template<class T> 
    void quicksort_ascending(T *__restrict__ t, 
			     const ssize_t begin, 
			     const ssize_t end)
  {
    if (likely(begin < end)) 
    {      
      const T pivotvalue = t[begin];
      ssize_t left  = begin - 1;
      ssize_t right = end   + 1;
      
      while(1) 
      {
        while (t[--right] > pivotvalue);
        while (t[++left] < pivotvalue);
        
        if (left >= right) break;
        
        const T temp = t[right];
        t[right] = t[left];
        t[left] = temp;
      }
      
      const int pivot = right;
      quicksort_ascending(t, begin, pivot);
      quicksort_ascending(t, pivot + 1, end);
    }
  }
  
  template<class T> 
    void quicksort_decending(T *__restrict__ t, 
			     const ssize_t begin, 
			     const ssize_t end)
    {
      if (likely(begin < end)) 
	{
	  const T pivotvalue = t[begin];
	  ssize_t left  = begin - 1;
	  ssize_t right = end   + 1;
      
	  while(1) 
	    {
	      while (t[--right] < pivotvalue);
	      while (t[++left] > pivotvalue);
        
	      if (left >= right) break;
        
	      const T temp = t[right];
	      t[right] = t[left];
	      t[left] = temp;
	    }
      
	  const int pivot = right;
	  quicksort_decending(t, begin, pivot);
	  quicksort_decending(t, pivot + 1, end);
	}
    }


  template<class T, int THRESHOLD> 
    void quicksort_insertionsort_ascending(T *__restrict__ t, 
					   const ssize_t begin, 
					   const ssize_t end)
    {
      if (likely(begin < end)) 
	{      
	  const ssize_t size = end-begin+1;
	  if (likely(size <= THRESHOLD))
	    {
	      insertionsort_ascending<T>(&t[begin],size);
	    }
	  else
	    {
	      const T pivotvalue = t[begin];
	      ssize_t left  = begin - 1;
	      ssize_t right = end   + 1;
      
	      while(1) 
		{
		  while (t[--right] > pivotvalue);
		  while (t[++left] < pivotvalue);
        
		  if (left >= right) break;
        
		  const T temp = t[right];
		  t[right] = t[left];
		  t[left] = temp;
		}
      
	      const int pivot = right;
	      quicksort_ascending(t, begin, pivot);
	      quicksort_ascending(t, pivot + 1, end);
	    }
	}
    }
    
  
  template<class T, int THRESHOLD> 
    void quicksort_insertionsort_decending(T *__restrict__ t, 
					   const ssize_t begin, 
					   const ssize_t end)
    {
      if (likely(begin < end)) 
	{
	  const ssize_t size = end-begin+1;
	  if (likely(size <= THRESHOLD))
	    {
	      insertionsort_decending<T>(&t[begin],size);
	    }
	  else
	    {

	      const T pivotvalue = t[begin];
	      ssize_t left  = begin - 1;
	      ssize_t right = end   + 1;
      
	      while(1) 
		{
		  while (t[--right] < pivotvalue);
		  while (t[++left] > pivotvalue);
        
		  if (left >= right) break;
        
		  const T temp = t[right];
		  t[right] = t[left];
		  t[left] = temp;
		}
      
	      const int pivot = right;
	      quicksort_decending(t, begin, pivot);
	      quicksort_decending(t, pivot + 1, end);
	    }
	}
    }

  template<size_t LOCAL_NODE_IDS>
  class AtomicIDBlock
  {
  public:
    AlignedAtomicCounter32& counter;
    unsigned int localNodeID;
    unsigned int localNodeIDs;
    unsigned int maxNodes;

    __forceinline AtomicIDBlock(AlignedAtomicCounter32& global, unsigned int maxNodes) : counter(global), maxNodes(maxNodes)
    {
      localNodeID  = counter.add(LOCAL_NODE_IDS);
      localNodeIDs = 0;      
    }

    __forceinline unsigned int get(const unsigned int i) 
    {
      const unsigned int currentIndex = localNodeID + localNodeIDs;

      if (unlikely(currentIndex >= maxNodes)) {
        FATAL("not enough nodes allocated");
      }
      
      localNodeIDs += i;
      if (unlikely(localNodeIDs == LOCAL_NODE_IDS)) {
        localNodeID = counter.add(LOCAL_NODE_IDS);
        localNodeIDs = 0;	
      }
      return currentIndex;
    }
  };
};
