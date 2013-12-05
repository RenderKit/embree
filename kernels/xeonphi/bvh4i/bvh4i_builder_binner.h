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

#ifndef __BVH4I_BUILDER_BINNING_MIC_H__
#define __BVH4I_BUILDER_BINNING_MIC_H__

#include "bvh4i.h"
#include "bvh4i/bvh4i_builder_util.h"

namespace embree
{
  struct Mapping
  {
  public:
    
    __forceinline Mapping () {}
    
    __forceinline Mapping (const Centroid_Scene_AABB& bounds) 
    {
      const ssef centroid_lower = *(ssef*)&bounds.centroid.lower;
      const ssef centroid_upper = *(ssef*)&bounds.centroid.upper;
      const ssef centroidDiagonal = (centroid_upper-centroid_lower) * 2.0f;
      scale = select(centroidDiagonal != 0.0f,rcp(centroidDiagonal) * ssef(16.0f * 0.99f),ssef(0.0f));
      ofs = (ssef)centroid_lower * 2.0f;
    }

    /*! Computes the bin numbers for each dimension for a box. */
    __forceinline ssei bin_unsafe(const BBox3f& box) const {
      const ssef box_lower = *(ssef*)&box.lower;
      const ssef box_upper = *(ssef*)&box.upper;
      return ssei(ssei_t(floor((ssef(box_lower+box_upper) - ofs)*scale)));
    }
      
    /*! Computes the bin numbers for each dimension for a box. */
    __forceinline ssei bin(const BBox3f& box) const {
      return clamp(bin_unsafe(box),ssei(0),ssei(15)); // FIXME: hardcoded number of bins
    }

  public:
    ssef ofs;        //!< offset to compute bin
    ssef scale;      //!< scaling factor to compute bin
  };

  struct Split 
  {
    __forceinline void reset()
    {
      dim = -1;
      pos = -1;
      numLeft = -1;
      cost = pos_inf;
    }

    __forceinline Split () 
      {
	reset();
      }
    
    /*! stream output */
    friend std::ostream& operator<<(std::ostream& cout, const Split& split) {
      return cout << "Split { " << 
        "dim = " << split.dim << 
        ", pos = " << split.pos << 
        ", numLeft = " << split.numLeft <<
        ", sah = " << split.cost << "}";
    }

  public:
    int dim;
    int pos;
    int numLeft;
    float cost;
  };

  template<int BINS>
    class Binner
  {
    public:

    /*! reset the binner */
    __forceinline void reset()
    {
      for (size_t i=0;i<BINS;i++) 
      {
        bounds[i][0] = empty;
        bounds[i][1] = empty;
        bounds[i][2] = empty;
        counts[i] = 0;
      }
    }
    
    /*! bin an array of primitives */
    void bin(const PrimRef* __restrict__ const prims, const size_t begin, const size_t end, const Mapping& mapping);

    /*! bin an array of primitives and copy to destination array */
    void bin_copy(const PrimRef* __restrict__ const prims, const size_t begin, const size_t end, const Mapping& mapping, PrimRef* __restrict__ const dst);

    /*! merge multiple binning infos into one */
    static void reduce(const Binner binners[], size_t num, Binner& binner_o);

    /*! calculate the best possible split */
    void best(Split& split, const Mapping& mapping);

    /* inplace partitioning of a list of primitives */
    void partition(PrimRef*__restrict__ const prims,
                   const size_t begin,
                   const size_t end,
                   const Split& split,
                   const Mapping& mapping,
                   BuildRecord& left,
                   BuildRecord& right);

  public:
    BBox3f bounds[BINS][4];
    ssei   counts[BINS];
  };

  bool split_fallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);

  template<int BINS>
    struct ParallelBinner
  {
  public:

    /*! parallel binbing of an array of primitives */
    void bin(BuildRecord& current, const PrimRef* src, PrimRef* dst, const size_t threadID, const size_t numThreads);

    /*! calculate the best possible split */
    void best(Split& split);

    /* parallel partitioning of a list of primitives */
    void partition(const PrimRef* src, PrimRef* dst, 
                   Split& split, 
                   BuildRecord &leftChild,
                   BuildRecord &rightChild,
                   const size_t threadID, const size_t numThreads);
    
  private:
    TASK_FUNCTION(ParallelBinner,parallelBinning);
    TASK_FUNCTION(ParallelBinner,parallelPartition);

  public:
    BuildRecord rec;
    Centroid_Scene_AABB left;
    Centroid_Scene_AABB right;
    Mapping mapping;
    Split split;
    const PrimRef* src;
    PrimRef* dst;
    __align(64) AlignedAtomicCounter32 lCounter;
    __align(64) AlignedAtomicCounter32 rCounter;
    Binner<BINS> bin16;
    __align(64) Binner<BINS> global_bin16[MAX_MIC_THREADS];
  };
};

#endif
