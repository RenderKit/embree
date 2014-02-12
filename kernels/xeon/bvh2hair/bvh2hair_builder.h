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

#include "geometry/primitive.h"

namespace embree
{
  class BVH2HairBuilder : public Builder
  {
    ALIGNED_CLASS;
  public:

    /*! Type shortcuts */
    typedef BVH2Hair::NodeRef NodeRef;
    typedef BVH2Hair::AlignedNode AlignedNode;
    typedef BVH2Hair::UnalignedNode UnalignedNode;
    typedef BVH2Hair::Bezier1 Bezier1;
    typedef BVH2Hair::NAABBox3fa NAABBox3fa;
    
  public:

    /*! builder entry point */
    void build(size_t threadIndex, size_t threadCount);

    /*! Constructor. */
    BVH2HairBuilder (BVH2Hair* bvh, Scene* scene);

    /*! Destructor. */
    ~BVH2HairBuilder ();

  private:

    /*! Tries to split hair into two differently aligned hair strands */
    struct StrandSplit
    {
    public:
      StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
                   const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1);

      /*! calculates standard surface area heuristic for the split */
      __forceinline float standardSAH() const {
        return BVH2Hair::intCost*float(num0)*halfArea(bounds0.bounds) + BVH2Hair::intCost*float(num1)*halfArea(bounds1.bounds);
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          (BVH2Hair::travCostUnaligned*bounds0.bounds.lower.w + BVH2Hair::intCost*float(num0))*halfArea(bounds0.bounds) + 
          (BVH2Hair::travCostUnaligned*bounds1.bounds.lower.w + BVH2Hair::intCost*float(num1))*halfArea(bounds1.bounds);
      }
      
      /*! finds the two hair strands */
      static const StrandSplit find(Bezier1* curves, size_t begin, size_t end);
      
      /*! splits hair list into the two strands */
      size_t split(Bezier1* curves, size_t begin, size_t end) const;

      friend std::ostream& operator<<(std::ostream& cout, const StrandSplit& p) {
        return std::cout << "{ " << std::endl << 
          " bounds0 = " << p.bounds0 << ", areaSum0 = " << p.bounds0.bounds.lower.w << ", axis0 = " << p.axis0 << ", num0 = " << p.num0 << std::endl << 
          " bounds1 = " << p.bounds1 << ", areaSum1 = " << p.bounds1.bounds.lower.w << ", axis1 = " << p.axis1 << ", num1 = " << p.num1 << std::endl << 
          "}";
      }

    public:
      NAABBox3fa bounds0, bounds1;  //!< bounds of the strands
      Vec3fa axis0, axis1;          //!< axis the strands are aligned into
      size_t num0, num1;            //!< number of hairs in the strands
    };

    /*! Performs standard object binning */
    struct ObjectSplit
    {
      /*! number of bins */
      static const size_t BINS = 16;

    public:

      /*! default constructor */
      __forceinline ObjectSplit ()
        : dim(-1), pos(0), cost(inf), num0(0), num1(0), bounds0(inf), bounds1(inf) {}
      
      /*! calculates standard surface area heuristic for the split */
      __forceinline float standardSAH() const {
        return BVH2Hair::intCost*float(num0)*halfArea(bounds0.bounds) + BVH2Hair::intCost*float(num1)*halfArea(bounds1.bounds);
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          (BVH2Hair::travCostUnaligned*bounds0.bounds.lower.w + BVH2Hair::intCost*float(num0))*halfArea(bounds0.bounds) + 
          (BVH2Hair::travCostUnaligned*bounds1.bounds.lower.w + BVH2Hair::intCost*float(num1))*halfArea(bounds1.bounds);
      }

      /*! performs object binning to the the best partitioning */
      static ObjectSplit find(Bezier1* curves, size_t begin, size_t end, const AffineSpace3fa& space);

      /*! calculates aligned bounds for left and right split */
      const ObjectSplit alignedBounds(Bezier1* curves, size_t begin, size_t end, const AffineSpace3fa& space);

      /*! calculates the bounds for left and right split */
      const ObjectSplit unalignedBounds(Bezier1* curves, size_t begin, size_t end);

      /*! splits hairs into two sets */
      size_t split(Bezier1* curves, size_t begin, size_t end) const;

      friend std::ostream& operator<<(std::ostream& cout, const ObjectSplit& p) {
        return std::cout << "{ " << std::endl << 
          " space = " << p.space << ", dim = " << p.dim << ", pos = " << p.pos << ", cost = " << p.cost << std::endl << 
          " bounds0 = " << p.bounds0 << ", areaSum0 = " << p.bounds0.bounds.lower.w << ", num0 = " << p.num0 << std::endl << 
          " bounds1 = " << p.bounds1 << ", areaSum1 = " << p.bounds1.bounds.lower.w << ", num1 = " << p.num1 << std::endl << 
          "}";
      }
      
    public:
      AffineSpace3fa space;
      NAABBox3fa bounds0, bounds1;
      int dim;
      int pos;
      float cost;
      size_t num0,num1;
      ssef ofs,scale;
    };

    /*! calculate bounds for range of primitives */
    static const NAABBox3fa computeAlignedBounds(Bezier1* curves, size_t begin, size_t end, const AffineSpace3fa& space);
    
    /*! try to find best non-axis aligned space, where the sum of all bounding areas is minimal */
    static const NAABBox3fa computeUnalignedBounds(Bezier1* curves, size_t begin, size_t end);

    /*! creates a leaf node */
    NodeRef leaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

    /*! recursive build function */
    NodeRef recurse_unaligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

    /*! recursive build function for axis aligned bounds */
    NodeRef recurse_aligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

    /*! recursive build function for aligned and non-aligned bounds */
    NodeRef recurse_aligned_unaligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf
    BVH2Hair* bvh;         //!< output
    Bezier1* curves;       //!< array with all curves
  };
}
